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
    profile_.shiftTableThrottleLevels = {0.05,0.15,0.25,0.40,0.55,0.70,0.80,0.90,0.95,1.00};
    profile_.shiftTable.clear();
    for (double thr : profile_.shiftTableThrottleLevels) {
        std::vector<double> row;
        // Upshift RPM envelope: 0.62..0.92 of redline across the throttle sweep
        // (was 0.40..0.85). Higher floor keeps gear 1 engaged to ~11 mph.
        double shiftRpm = profile_.redlineRpm * (0.62 + 0.30 * thr);
        for (size_t i = 0; i + 1 < gearRatios.size(); ++i) {
            double speedMs = shiftRpm / 60.0 * 2.0 * 3.14159265358979 * tireRadiusM
                           / (gearRatios[i] * diffRatio);
            row.push_back(speedMs * 3.6);
        }
        profile_.shiftTable.push_back(row);
    }
    profile_.separateDownshiftTableEnabled = true;
    profile_.downshiftTableThrottleLevels = profile_.shiftTableThrottleLevels;
    profile_.downshiftTable.clear();
    for (const auto& srcRow : profile_.shiftTable) {
        std::vector<double> row;
        for (double upSpeed : srcRow) row.push_back(upSpeed * 0.70);
        profile_.downshiftTable.push_back(row);
    }
    profile_.hysteresisFactor = 0.85;
    // Reconstruct gearbox with matched profile
    profile_.downshiftTable.clear();
    for (const auto& srcRow : profile_.shiftTable) {
        std::vector<double> row;
        for (double upSpeed : srcRow) row.push_back(upSpeed * 0.70);
        profile_.downshiftTable.push_back(row);
    }
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

    if (shiftTimerS_ <= disengageDuration) {
        clutchPressure_ = 1.0 - (shiftTimerS_ / disengageDuration);
    } else if (shiftTimerS_ <= disengageDuration + pauseDuration) {
        clutchPressure_ = 0.0;
        if (shiftTimerS_ > disengageDuration + pauseDuration / 2.0) {
            gearbox_->update(0, 0, 0);
        }
    } else if (shiftTimerS_ <= disengageDuration + pauseDuration + reengageDuration) {
        double reengageProgress = (shiftTimerS_ - disengageDuration - pauseDuration) / reengageDuration;
        clutchPressure_ = reengageProgress;
    } else {
        clutchPressure_ = 1.0;
        state_ = TwinState::RUNNING;
    }
}

}
