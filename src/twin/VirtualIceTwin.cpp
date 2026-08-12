#include <algorithm>

#include <twin/VirtualIceTwin.h>
#include <twin/SlipLockController.h>
#include <twin/TorqueConverterLaunch.h>
#include <simulator/GearConventions.h>
#include <simulator/EngineSimTypes.h>

namespace twin {

VirtualIceTwin::VirtualIceTwin(IceVehicleProfile profile)
    : profile_(std::move(profile)),
      gearbox_(std::make_unique<AutomaticGearbox>(profile_)),
      throttleSmoother_(profile_.throttleSmoothingTauMs),
      state_(TwinState::OFF) {}

void VirtualIceTwin::reconfigureProfile(const std::vector<double>& gearRatios,
                                          double diffRatio, double tireRadiusM) {
    if (gearRatios.empty()) return;
    profile_.gearRatios = gearRatios;
    profile_.diffRatio = diffRatio;
    profile_.tireRadiusM = tireRadiusM;
    // Auto-generate shift table (same logic as ReplayTelemetryProvider).
    // Recalibrated for the ICE twin's geometry: the ZF8-style box rides on top of
    // a 9.0:1 diff (tesla_y_vehicle), which multiplies every gear ratio. With the
    // old `redline*(0.40+0.45*thr)` band the implied upshift road speeds were too
    // LOW for such a tall final drive and the box climbed to high gears at very
    // low road speed (e.g. gear 7 near 12 mph), dragging engine RPM into the lug
    // zone. The band below targets a higher upshift-RPM envelope so LOW gears are
    // HELD at LOW speed (gear 1-2 around 11 mph) and the engine revs in those
    // gears (high revs in a low gear at low speed is correct — that is the engine
    // revving, not lugging). It still scales with throttle: light throttle shifts
    // earlier (lower RPM/speed), WOT holds the gear to higher RPM/speed.
    // PER-GEAR TARGET SHIFT MAP (ZF8-style, diff 2.82). The single `shiftRpm`
    // knob it replaced OSCILLATED: because each gear ratio differs, one RPM
    // band maps to wildly different road speeds per gear, so at a fixed road
    // speed the implied RPM swung across gears and the box could never settle
    // (DA5-at-40 <-> DA3-at-40). Instead the band CHECK is on ROAD SPEED, and
    // the gear-ratio-implied RPM is only a sanity bound. Upshift gear N->N+1
    // fires at the TOP of N's band; downshift N->N-1 fires below the BOTTOM of
    // N's band minus a small hysteresis, so a gear is HELD across its dead band
    // (no hunting). Bands are in km/h; the upstream signal carries road speed in
    // km/h, so the task's "mph" map is applied 1:1 on signal.speedKmh.
    //   DA1 0-15 | DA2 15-25 | DA3 25-35 | DA4 35-45 | DA5 45-55 | DA6 55-65 | DA7 65+
    // CO-FIX (band units): the band tops were specified in MPH but consumed as
    // KMH (the upstream signal carries road speed in km/h, applied 1:1). At the
    // old 15/25/35/45/55/65 "km/h" the box climbed to DA5 at ~29 mph target while
    // the real engine lugged to ~310 rpm — the death spiral. The tops are now
    // converted to true km/h (mph * 1.60934): 15,25,35,45,55,65 mph ->
    // 24.14, 40.23, 56.33, 72.42, 88.51, 104.61 km/h, so DA5 unlocks only above
    // ~45 mph, not ~18 mph.
    static const std::vector<double> kUpshiftBandTopKmh = {24.14, 40.23, 56.33, 72.42, 88.51, 104.61};
    static constexpr double kDownshiftHysteresisKmh = 5.0;  // held band below each top
    // Mild throttle sensitivity: light throttle upshifts a touch earlier, WOT
    // holds each gear a touch longer — but always centered on the band top so
    // the map still owns steady-state (throttle never breaks convergence).
    auto bandScale = [](double thr) {
        const double t = std::clamp((thr - 0.05) / 0.95, 0.0, 1.0);
        return 0.95 + 0.10 * t;  // 0.95 (light) .. 1.05 (WOT)
    };

    const int numCols = static_cast<int>(gearRatios.size()) - 1;
    std::vector<double> upTops;
    upTops.reserve(numCols);
    for (int i = 0; i < numCols; ++i) {
        if (i < static_cast<int>(kUpshiftBandTopKmh.size())) {
            upTops.push_back(kUpshiftBandTopKmh[i]);
        } else {
            // Engines with more than 6 upshifts: extend the 10 km/h ladder.
            upTops.push_back(kUpshiftBandTopKmh.back() + 10.0 * (i - static_cast<int>(kUpshiftBandTopKmh.size()) + 1));
        }
    }

    profile_.shiftTableThrottleLevels = {0.05,0.15,0.25,0.40,0.55,0.70,0.80,0.90,0.95,1.00};
    profile_.shiftTable.clear();
    for (double thr : profile_.shiftTableThrottleLevels) {
        const double s = bandScale(thr);
        std::vector<double> row;
        for (double top : upTops) {
            row.push_back(top * s);  // upshift N->N+1 at the top of N's band
        }
        profile_.shiftTable.push_back(row);
    }
    profile_.separateDownshiftTableEnabled = true;
    profile_.downshiftTableThrottleLevels = profile_.shiftTableThrottleLevels;
    profile_.downshiftTable.clear();
    // Downshift gear G->G-1 (column == gear G-2) fires at the BOTTOM of G's band
    // (== the top that entered G) minus hysteresis: a symmetric dead band of
    // `hyst` below the upshift that produced G. Hysteresis by construction, so a
    // single speed can never satisfy both an up- and down-shift (no oscillation).
    for (double thr : profile_.downshiftTableThrottleLevels) {
        const double s = bandScale(thr);
        std::vector<double> row;
        for (double top : upTops) {
            row.push_back(std::max(0.0, top - kDownshiftHysteresisKmh) * s);
        }
        profile_.downshiftTable.push_back(row);
    }
    profile_.hysteresisFactor = 0.85;
    // Ground the idle floor in the V3's EMERGENT idle (~500 rpm, per the .mr —
    // the M156 has no idle scalar; it settles ~400–600 on physics). The old
    // hardcoded 750 (IceVehicleProfile default / zf8hp45) was wrong for this
    // engine. The anti-lug guard floor is now declaratively idle + lugFloorMargin
    // (AutomaticGearbox::decide), so this single grounding flows everywhere.
    profile_.idleRpm = 500.0;
    // MIN-RPM FLOOR: the engine must stay above idle under load. The speed map
    // owns steady-state gear selection; an anti-lug guard in AutomaticGearbox
    // (decideGear, idle + margin) forces a single downshift whenever the CURRENT
    // gear's ratio-implied RPM drops below idle+margin, so the engine never lugs
    // (was ~92 rpm at low speed). Keeping downshiftRpmFloor at 0 here is
    // deliberate: the OLD 1500 rpm floor fought the upshift table at cruise
    // (forced DA4 at 40 km/h, then the table re-climbed to DA5 — the very
    // oscillation this map removes). The separate guard is inert for the twin.
    profile_.downshiftRpmFloor = 0.0;
    // Preserve logger across gearbox reconstruction (bug: prior code called
    // setLogger(nullptr) here, which dropped the logger attached at startup
    // and produced empty --gearbox-log CSVs even though the gearbox shifted).
    IGearboxLogger* preservedLogger = gearbox_ ? gearbox_->getLogger() : nullptr;
    gearbox_ = std::make_unique<AutomaticGearbox>(profile_);
    gearbox_->setGearSelector(selector_);
    gearbox_->setLogger(preservedLogger);
}

void VirtualIceTwin::setGearboxLogger(IGearboxLogger* logger) {
    gearbox_->setLogger(logger);
}

double VirtualIceTwin::roadSpeedImpliedRpmFor(double wheelSpeedKmh) const {
    // Clones ReplayTelemetryProvider::handleAutoGearboxDrive's implied-RPM math:
    //   wheelRadS = (v/3.6) / tireRadius
    //   roadSpeedImpliedRpm = wheelRadS * gearRatio * diffRatio * 30/π
    // Fall back to idleRpm when the current gear is out of the ratio vector range.
    // if-with-initializer keeps `gear` scoped to the range check (S6004).
    if (const int gear = gearbox_->getCurrentGear();
        gear >= 1 && gear <= static_cast<int>(profile_.gearRatios.size())) {
        const double speedMs = wheelSpeedKmh / 3.6;
        const double wheelRadS = speedMs / profile_.tireRadiusM;
        return wheelRadS
             * profile_.gearRatios[gear - 1]
             * profile_.diffRatio
             * 30.0 / 3.14159265358979;
    }
    return profile_.idleRpm;
}

TwinOutput VirtualIceTwin::update(double dt, const input::UpstreamSignal& signal) {
    TwinOutput output;
    // Dyno off by default; only CRANKING (FREE mode) re-enables it.
    output.dynoTorqueScale = 0.0;

    if (!signal.isValid || signal.timestampUtcMs == 0) {
        timeWithoutValidTelemetryS_ += dt;
        if (timeWithoutValidTelemetryS_ >= EngineSimDefaults::TELEMETRY_TIMEOUT_S) {
            state_ = TwinState::OFF;
        }
        output.gear = gearbox_->getCurrentGear();
        return output;
    }

    timeWithoutValidTelemetryS_ = 0.0;

    // Ignition off → force to OFF state, engine dies
    if (!ignitionOn_ && state_ != TwinState::OFF) {
        state_ = TwinState::OFF;
        crankingTimerS_ = 0.0;
    }

    // Stay OFF while ignition is off
    if (!ignitionOn_) {
        output.gear = gearbox_->getCurrentGear();
        output.ignition = false;
        output.clutchPressure = clutchPressure_;
        output.gearSelector = selector_;
        return output;
    }

    throttleSmoother_.update(dt, signal.throttleFraction);
    output.throttle = throttleSmoother_.getCurrentValue();

    switch (state_) {
        case TwinState::OFF:
            state_ = TwinState::CRANKING;
            output.starterMotor = true;
            output.ignition = true;
            output.gear = static_cast<int>(bridge::BridgeGear::NEUTRAL);
            clutchPressure_ = 0.0;
            break;

        case TwinState::CRANKING: {
            crankingTimerS_ += dt;
            output.throttle = EngineSimDefaults::CRANKING_THROTTLE;
            // Starter is NOT held here. The twin emits starterMotor as a one-tick
            // EDGE on the OFF->CRANKING transition (the OFF case above pulses it);
            // it must not hold it through CRANKING. The bridge's
            // CrankingController::engageStarter is a momentary toggle -- a held
            // starterButton=true while Cranking forces the phase to Stopped and
            // cuts the starter -- so holding it would re-toggle engageStarter
            // every tick, structurally disable the fast-path catch (its Stopped
            // case resets the exhaust-flow baseline on every Stopped->Cranking),
            // and drive the Stopped<->Cranking oscillation. The bridge's step()
            // owns cranking duration via its own tick counter, so a held starter
            // is redundant for engagement. See AC18.
            output.ignition = true;

            // FREE mode: enable dyno braking during cranking to give the starter
            // a resistive load. Without a vehicle-speed constraint (PIN mode's
            // setVehicleSpeedTarget) the engine has no load path in FREE mode,
            // the starter free-revs without building measurable RPM, and the
            // CrankingController never sees a catch (stays at 0 RPM indefinitely).
            // The dyno provides that load directly on the engine crankshaft.
            if (coupling_->getMode() == WheelCouplingMode::Free) {
                output.dynoTorqueScale = 0.15;
            }

            // CRANKING -> IDLE: the engine catches via EITHER
            //   (a) the physics fast-path — fed-back RPM exceeds the catch
            //       threshold (the closed loop confirms combustion is sustained), OR
            //   (b) the deterministic time fallback — the starter has cranked for
            //       CRANK_FALLBACK_DURATION_S of sim-time. (b) is decoupled from the
            //       RPM value on purpose: with live CSV pacing (one row per frame,
            //       real-time) the fed-back cranking RPM is one tick in arrears and
            //       frequently plateaus below the threshold, so gating the fallback
            //       on the RPM reading (the old `engineRpmFeedback_ == 0.0` guard)
            //       left the twin stuck in CRANKING whenever the plateau was
            //       non-zero-but-low. The time fallback guarantees a deterministic
            //       start: same input frames -> same outcome every run.
            const double CRANK_IDLE_RPM_THRESHOLD = 500.0;
            const double CRANK_FALLBACK_DURATION_S = 3.0;
            const bool rpmCaught = engineRpmFeedback_ > CRANK_IDLE_RPM_THRESHOLD;

            if (const bool fallbackExpired = crankingTimerS_ >= CRANK_FALLBACK_DURATION_S;
                rpmCaught || fallbackExpired) {
                state_ = TwinState::IDLE;
                output.starterMotor = false;
                output.dynoTorqueScale = 0.0;  // dyno off once engine catches
            }
            output.gear = static_cast<int>(bridge::BridgeGear::NEUTRAL);
            clutchPressure_ = 0.0;
            break;
        }

        case TwinState::IDLE:
            // Idle-sustain floor: hold a minimum throttle through IDLE so the engine
            // never coasts through the engine-sim's Stopped latch during the
            // CRANKING->IDLE handoff (the catch releases the forced cranking throttle
            // + starter simultaneously; without a floor the engine decays to Stopped
            // before the driver's throttle arrives, and throttle alone can't restart
            // it). Real engines need idle throttle to sustain combustion; the prior
            // "idles on physics alone" model stalled at the handoff. The IDLE->RUNNING
            // transition is gated on the raw driver signal, so this floor does not
            // false-trigger RUNNING.
            output.throttle = std::max(throttleSmoother_.getCurrentValue(),
                                       EngineSimDefaults::IDLE_SUSTAIN_THROTTLE);
            output.ignition = true;
            output.gear = static_cast<int>(bridge::BridgeGear::NEUTRAL);
            clutchPressure_ = 0.0;

            // IDLE->RUNNING: requires selector in DRIVE AND throttle above idle threshold
            if (selector_ == bridge::GearSelector::DRIVE &&
                signal.throttleFraction > profile_.throttleIdleThreshold) {
                state_ = TwinState::RUNNING;
            }
            break;

        case TwinState::RUNNING: {
            // The clutch slip-lock uses the wheel speed SELECTED BY THE COUPLING
            // STRATEGY: PIN uses the CSV road speed (sim pinned to it); FREE/TORQUE
            // use the ACTUAL simulated wheel speed (their speed is emergent — FREE
            // for the mph-vs-target diagnostic, TORQUE from injected torque) so the
            // slip-lock tracks real engine↔wheel slip.
            const double wheelKmh = coupling_->slipLockWheelSpeedKmh(vehicleSpeedFeedbackKmh_, signal.speedKmh);

            // The GEARBOX SHIFT DECISION uses the UPSTREAM COMMANDED road speed
            // (signal.speedKmh), NOT the coupling/friction-clutch feedback speed.
            // This matches the pre-wheel-coupling behaviour and the contract in
            // VirtualIceTwinTest.FeedbackSpeedDoesNotOverrideSignalSpeedForUpshift:
            // the gearbox governor is driven by the commanded road speed, so it
            // shifts even when no engine-sim wheel feedback is being pumped (the
            // scenario tests and the live-stream path feed speed via the signal
            // only). Routing the gearbox through the feedback source is what stuck
            // the box in 1st — in FREE mode the feedback was 0, so every frame
            // looked like standstill and no shift ever fired. The slip-lock and
            // the shift governor are genuinely two different quantities; they must
            // not be conflated into one wheel-speed variable.
            gearbox_->setTwinContext(static_cast<int>(state_), clutchPressure_, vehicleSpeedFeedbackKmh_, engineRpmFeedback_);
            gearbox_->setGearSelector(selector_);
            gearbox_->update(dt, signal.speedKmh, signal.throttleFraction, drivetrainTorqueNm_);
            output.ignition = true;

            // FIX #3 (idle/stall): the engine must NEVER coast through the
            // engine-sim Stopped latch while ignition is ON. The idle-sustain
            // floor previously existed ONLY in IDLE; at RUNNING/DAN the twin
            // sent ~0 throttle and a falling feedback RPM let the engine decay
            // to 0 and latch Stopped (the mid-drive stall bug). Two guards:
            //
            //  (a) Idle-sustain floor: when the ACTUAL engine RPM feedback is
            //      below idle, floor the throttle at IDLE_SUSTAIN_THROTTLE so
            //      the engine holds ~idle instead of coasting to 0. This keeps
            //      a running engine alive under no-driver-throttle load (e.g.
            //      standstill in DRIVE), exactly mirroring the IDLE-state floor.
            //
            //  (b) Restart-on-stall: if the engine has already stalled
            //      (feedback RPM ~ 0) with ignition ON, re-crank it by emitting
            //      the starter edge + a cranking throttle floor until the engine
            //      catches (RPM recovers). This is the same cranking throttle the
            //      OFF->CRANKING path uses, applied locally so a mid-drive stall
            //      self-heals instead of latching Stopped forever.
            const double kStallRpm = 30.0;  // below this = engine has stopped
            const bool engineStalled = engineRpmFeedback_ <= kStallRpm;
            if (engineStalled) {
                // Re-crank: spin the starter + feed cranking throttle until the
                // engine catches on the next feedback sample.
                output.starterMotor = true;
                output.throttle = std::max(
                    std::max(throttleSmoother_.getCurrentValue(),
                             EngineSimDefaults::CRANKING_THROTTLE),
                    EngineSimDefaults::IDLE_SUSTAIN_THROTTLE);
            } else if (engineRpmFeedback_ < profile_.idleRpm) {
                // Engine alive but dipping below idle: hold the sustain floor so
                // it cannot coast through the Stopped latch.
                output.throttle = std::max(throttleSmoother_.getCurrentValue(),
                                           EngineSimDefaults::IDLE_SUSTAIN_THROTTLE);
            }

            // RUNNING->IDLE: selector moved to NEUTRAL or PARK
            if (selector_ == bridge::GearSelector::NEUTRAL ||
                selector_ == bridge::GearSelector::PARK) {
                state_ = TwinState::IDLE;
            } else if (signal.speedKmh < profile_.standstillThresholdKmh &&
                       signal.throttleFraction < profile_.throttleIdleThreshold) {
                state_ = TwinState::IDLE;
            } else if (gearbox_->requestsShift()) {
                state_ = TwinState::SHIFTING;
                shiftTimerS_ = 0.0;
            }

            output.gear = gearbox_->getCurrentGear();

            // Clutch slip-lock (replaces the old rigid `clutchPressure_ = 1.0;`).
            // Uses the same emergent wheel speed as the gearbox (above).
            const double roadSpeedImpliedRpm = roadSpeedImpliedRpmFor(wheelKmh);
            const auto slip = computeSlipLockPressure(
                twin::SlipLockInput{engineRpmFeedback_,
                                    roadSpeedImpliedRpm,
                                    signal.throttleFraction,
                                    profile_.idleRpm,
                                    profile_.redlineRpm},
                /*maxCreepPressure=*/0.10);
            // PIN uses the BOUNDED clutch pressure from its coupling strategy
            // (engine revs on launch via partial pressure, locks at cruise, never
            // fully open). FREE/TORQUE return -1.0 and defer to the emergent
            // slip-lock computed above.
            const double lockOverride = coupling_->clutchLockOverride(
                engineRpmFeedback_, roadSpeedImpliedRpm, signal.throttleFraction,
                profile_.idleRpm, profile_.redlineRpm);
            clutchPressure_ = (lockOverride >= 0.0) ? lockOverride : slip.clutchPressure;
            // Launch (torque converter): the slip-lock is a velocity-match
            // schedule that only behaves once the wheels are coupled to the
            // engine (road-implied RPM >= idle). At standstill a fixed creep
            // pressure yanks the engine down -> the launch bogs near-stall (the
            // velocity-match catch-22). Instead, while the wheels are still free
            // (road-implied < idle), override the creep with a stall-gated launch
            // pressure that transmits torque at 0 wheel speed without stalling
            // the engine (see TorqueConverterLaunch.h). Once the wheels couple
            // the launch function defers (returns LAUNCH_PRESSURE_DEFER) and the
            // slip-lock lock-up pressure above stands. This applies to every mode
            // that leaves the sim speed INDEPENDENT (Free, Torque) — selected by
            // the strategy so the twin carries no mode conditional. PIN never
            // launches: its vehicle-speed constraint drives the wheels directly.
            if (coupling_->launchAssistAtStandstill()) {
                const double launchPressure = computeLaunchPressure(
                    twin::LaunchPressureInput{engineRpmFeedback_,
                                              roadSpeedImpliedRpm,
                                              signal.throttleFraction,
                                              profile_.idleRpm,
                                              profile_.redlineRpm});
                if (launchPressure != twin::LAUNCH_PRESSURE_DEFER) {
                    clutchPressure_ = launchPressure;
                }
            }

            // Creep-drag relief at standstill: a strategy whose wheels are pinned
            // to the road speed (PIN) couples the engine to a wheel that cannot
            // turn at standstill, and even the slip-lock floor drags the engine
            // down through idle to stall (the creep clutch-drag bug) — and once it
            // stalls the re-crank can't catch because the engaged clutch sends the
            // starter torque to the stationary wheel (the 0<->170 limp). Opening
            // the clutch below the profile's standstill threshold lets the engine
            // idle decoupled (the pinned wheel follows the CSV regardless) and
            // lets a re-crank spin the engine freely. The strategy declares
            // whether it relieves (relievesCreepDragAtStandstill, like
            // launchAssistAtStandstill) so the twin carries no mode conditional;
            // the threshold is the profile's standstillThresholdKmh, so no magic
            // constant. Above the threshold the slip-lock/launch pressure stands,
            // so a moving vehicle still bump-starts and the cruise lock is intact.
            if (signal.speedKmh < profile_.standstillThresholdKmh &&
                coupling_->relievesCreepDragAtStandstill()) {
                clutchPressure_ = 0.0;
            }
            output.pinVehicleSpeedTargetKmh = coupling_->vehicleSpeedTargetKmh(signal.speedKmh);
            // MATCH (Torque) mode: surface the recorded input torque so the
            // simulator injects it at the rotating mass each frame and the solver
            // integrates road speed from it (engine RPM emerges via the clutch
            // coupling, not by fiat). FREE/PIN surface 0.0 (no-op).
            output.drivetrainInputTorqueNm = coupling_->injectedInputTorqueNm(signal.motorTorqueNm);
            break;
        }

        case TwinState::SHIFTING:
            gearbox_->setTwinContext(static_cast<int>(state_), clutchPressure_, vehicleSpeedFeedbackKmh_, engineRpmFeedback_);
            updateShiftExecution(dt);
            output.ignition = true;
            output.gear = gearbox_->getCurrentGear();
            break;
    }

    output.clutchPressure = clutchPressure_;
    output.gearSelector = selector_;
    return output;
}

void VirtualIceTwin::updateShiftExecution(double dt) {
    shiftTimerS_ += dt;

    // Determine shift timing based on throttle and shift direction
    double disengageDuration;
    double pauseDuration;
    double reengageDuration;

    double currentThrottle = throttleSmoother_.getCurrentValue();
    int shiftDirection = gearbox_->getLastShiftDirection();
    (void)currentThrottle;
    (void)shiftDirection;

    // Shift timing read from profile vectors
    disengageDuration = profile_.shiftDisengageMs * EngineSimDefaults::MS_TO_SECONDS;
    pauseDuration = profile_.shiftPauseMs * EngineSimDefaults::MS_TO_SECONDS;
    reengageDuration = profile_.shiftReengageMs * EngineSimDefaults::MS_TO_SECONDS;

    // The clutch is NEVER fully open — the slip-lock floor rule
    // (kSlipLockPressureFloor). During a shift the clutch unloads so the gear
    // can change, but it bottoms out at the floor (not 0): a fully-open clutch
    // through the shift lets the engine free-rev under throttle, and the
    // re-engage then transmits that over-rev torque as a spike the vehicle-speed
    // constraint cannot absorb fast enough (the mph-overshoot transient on every
    // 1->2 / 2->3 shift). Holding the floor keeps the engine loaded through the
    // shift so it cannot run away, and the re-engage ramps floor -> 1.0.
    const double floor = twin::kSlipLockPressureFloor;
    if (shiftTimerS_ <= disengageDuration) {
        // Unload: ramp 1.0 -> floor (keep the engine loaded, never fully open).
        // std::max on the factor guards a discrete timestep that overshoots
        // disengageDuration so the ramp cannot dip below the floor.
        clutchPressure_ = floor + (1.0 - floor) *
                         std::max(0.0, 1.0 - shiftTimerS_ / disengageDuration);
    } else if (shiftTimerS_ <= disengageDuration + pauseDuration) {
        clutchPressure_ = floor;
        if (shiftTimerS_ > disengageDuration + pauseDuration / 2.0) {
            gearbox_->update(0, 0, 0);
        }
    } else if (shiftTimerS_ <= disengageDuration + pauseDuration + reengageDuration) {
        const double reengageProgress =
            (shiftTimerS_ - disengageDuration - pauseDuration) / reengageDuration;
        clutchPressure_ = floor + (1.0 - floor) * reengageProgress;
    } else {
        clutchPressure_ = 1.0;
        state_ = TwinState::RUNNING;
    }
}

}
