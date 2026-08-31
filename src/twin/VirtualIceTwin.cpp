#include <algorithm>
#include <cmath>

#include <twin/VirtualIceTwin.h>
#include <simulation/CrankingController.h>
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
    static const std::vector kUpshiftBandTopKmh = {24.14, 40.23, 56.33, 72.42, 88.51, 104.61};
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
    // Ground the idle floor in the V3's EMERGENT idle. The M156 has no idle
    // scalar; MEASURED at standstill it settles ~930-990 rpm (modal ~950) — NOT
    // the stale "400-600" .mr comment that earlier edits trusted. The anti-lug
    // guard floor is declaratively idle + lugFloorMargin (AutomaticGearbox::decide),
    // so this single grounding flows everywhere (creep-relief ceiling, lug guard,
    // idle-sustain). At 950 the creep-relief fires whenever road-implied < 950
    // (≈ below ~6 km/h in 1st), so the engine idles decoupled through the creep
    // regime instead of lugging under the old 500 floor.
    profile_.idleRpm = 950.0;
    // CREEP-RELIEF CEILING: the road speed at which 1st-gear road-implied RPM
    // reaches idle. Below it a PIN-coupled (road-pinned) wheel would lug the
    // engine under idle through an engaged clutch (the 1-3 mph creep-lug bug);
    // the relief opens the clutch through this whole regime so the engine idles
    // decoupled. Derived from the SAME geometry as roadSpeedImpliedRpmFor
    // (solve roadSpeedImpliedRpm == idleRpm for wheelSpeedKmh) so it is always
    // coherent with the actual ratios — no magic constant, and a tyre/final-
    // drive change recalibrates it automatically. For C63_TeslaY (1st 4.38,
    // diff 2.82, tyre 0.356 m, idle 950) this is ~10.3 km/h (~6.4 mph), covering
    // the full creep band the old standstill-only 1.0 km/h gate missed. Gated on
    // VEHICLE speed (not road-implied RPM) so a moving wheel still bump-starts.
    if (!gearRatios.empty() && diffRatio > 0.0 && tireRadiusM > 0.0) {
        profile_.creepReliefThresholdKmh =
            profile_.idleRpm * tireRadiusM * 3.6 * 3.14159265358979 /
            (30.0 * gearRatios[0] * diffRatio);
    }
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

void VirtualIceTwin::setEffectiveThrottleConfig(const EffectiveThrottleConfig& config) {
    // Reconfigure = fresh derivation: a new config must not inherit the
    // previous configuration's takeover latch (reset() semantics, mirrored by
    // constructing a new derivation).
    effectiveThrottle_ = EffectiveThrottleDerivation(config);
}

void VirtualIceTwin::setTorqueInformedGearboxConfig(const TorqueInformedGearboxConfig& config) {
    torqueInformedGearbox_ = config;
}

std::unique_ptr<ITorqueHint> VirtualIceTwin::gearboxTorqueHint(
    const input::UpstreamSignal& signal) const {
    if (!torqueInformedGearbox_.enabled) {
        return nullptr;
    }
    return std::make_unique<UpstreamTorqueHint>(torqueInformedGearbox_,
                                                signal.motorTorqueNm);
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

    // Single derivation point for the ENGINE DRIVE (--effective-throttle):
    // the effective throttle is computed ONCE per frame from the raw pedal and
    // the upstream commanded motor torque, upstream of the throttle smoother
    // (mirrors the brake-light single-derivation pattern in SimulationLoop).
    // With the feature off (the default) update() returns the pedal unchanged,
    // so the smoother input — and therefore the whole twin — is bit-identical
    // to an unconfigured twin. Scoped to the engine drive ONLY: the gearbox
    // shift decision below keeps reading the raw signal.throttleFraction.
    const double engineDriveThrottle =
        effectiveThrottle_.update(signal.throttleFraction, signal.motorTorqueNm);
    throttleSmoother_.update(dt, engineDriveThrottle);
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
            // PARK-start: the prime/warm-boot advances only the TWIN's state
            // machine — the engine core starts Stopped, and a capture can sit
            // in PARK for seconds (UpLeckHill: ~8 s parked) before the driver
            // selects D. With no starter path in IDLE the engine sat dead for
            // that whole window; a real car cranks and idles in PARK. Same
            // restart-on-stall guard RUNNING uses (one-tick edge + retry
            // cooldown); when stalled it also raises the throttle floor above
            // to cranking level.
            restartIfStalled(output, dt);
            output.ignition = true;
            output.gear = static_cast<int>(bridge::BridgeGear::NEUTRAL);
            clutchPressure_ = 0.0;

            // IDLE->RUNNING: a real auto engages the moment the selector leaves
            // P/N for a drive position (D/R) — it sits in 1st at idle, ready to
            // creep against a slipping/open clutch, NOT in neutral waiting for
            // throttle. The prior throttle>idle gate left the box in IDLE/DAN at
            // a standstill in DRIVE (foot off the gas), where the M156 decayed on
            // the 5% idle-sustain floor and stalled (no re-crank lives in IDLE).
            // D/R -> RUNNING puts the box in 1st (DA1) where the creep-relief
            // opens the clutch and the re-crank can hold the engine. P/N stays
            // IDLE (true neutral).
            if (selector_ == bridge::GearSelector::DRIVE ||
                selector_ == bridge::GearSelector::REVERSE) {
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
            gearbox_->update(dt, signal.speedKmh, signal.throttleFraction,
                             drivetrainTorqueNm_, gearboxTorqueHint(signal));
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
            // Below the engine's own Running->Stopped latch bar = stalled. This
            // MUST be the engine's bar (CrankingController::STOPPED_RPM), not a
            // twin-local guess: the restart pulse below is a ONE-TICK edge, and
            // engageStarter() only honours the button once the phase has latched
            // Stopped. A higher twin threshold (the old 30 rpm) pulses while the
            // phase is still Running on a slow decay — the button is dropped and
            // the retry cooldown then blanks the starter for its full 3 s past
            // the moment the phase does latch. The feedback signal is one step
            // in arrears of the latch, which keeps the ordering safe by one tick.
            if (restartIfStalled(output, dt)) {
                // Stalled: the guard pulsed the starter edge, flushed the
                // idle-hold controller and floored the throttle at cranking
                // level (see restartIfStalled for the edge/cooldown contract).
            } else {
                // Engine recovered above STOPPED_RPM: re-arm the cooldown so the
                // next genuine stall fires a fresh edge promptly.

                // Idle-hold controller (guard (a), upgraded — see
                // idleHoldFloor()): engine alive but sagging below idle. The
                // old guard was a static 5% floor; this is the ECU idle-air
                // equivalent. Returns 0 when it has nothing to add.
                const double idleFloor = idleHoldFloor(dt, engineRpmFeedback_);
                if (idleFloor > 0.0) {
                    // FLOOR semantics, stated in code: the controller can
                    // only ADD throttle, never take it away.
                    output.throttle = std::max(throttleSmoother_.getCurrentValue(),
                                               idleFloor);
                }
            }

            // RUNNING->IDLE: only on a selector move to P/N. A real auto STAYS in
            // 1st at creep / a stoplight (clutch relieved, engine idling decoupled)
            // — it does not drop to neutral at low speed or standstill. The prior
            // eager transition (speed<threshold AND throttle<idle on a SINGLE frame)
            // flipped RUNNING→IDLE→DAN the instant the driver lifted at low speed,
            // then coasted to a stall (the DAN-at-slow-speed bug). In DRIVE the box
            // now stays RUNNING/DA1 through the whole creep band and at a stop; the
            // creep-relief opens the clutch so the engine idles, and the re-crank
            // path (below) holds it there.
            if (selector_ == bridge::GearSelector::NEUTRAL ||
                selector_ == bridge::GearSelector::PARK) {
                state_ = TwinState::IDLE;
            } else if (gearbox_->requestsShift()) {
                state_ = TwinState::SHIFTING;
                shiftTimerS_ = 0.0;
            }

            output.gear = gearbox_->getCurrentGear();

            // Clutch pressure is owned by the coupling MODEL (OCP strategy). The
            // default ClutchMap is a declarative governor curve that NEVER opens
            // the clutch fully and is C1-continuous — there is nothing to bang
            // between, so it cannot oscillate (the legacy binary relief cycled
            // 0↔redline precisely because it was bang-bang). The Legacy model
            // DEFERS (returns kCouplingDeferToLegacy < 0) so the historical
            // slip-lock + launch + binary-relief chain runs unchanged for A/B
            // (--coupling-model legacy reproduces the old oscillation). The
            // TorqueConverter model returns its own smooth fluid pressure. Uses
            // the same emergent wheel speed as the gearbox (above).
            const double roadSpeedImpliedRpm = roadSpeedImpliedRpmFor(wheelKmh);
            const auto couplingOut = couplingModel_->compute(
                twin::CouplingInput{engineRpmFeedback_,
                                    roadSpeedImpliedRpm,
                                    signal.throttleFraction,
                                    profile_.idleRpm,
                                    profile_.redlineRpm,
                                    /*maxClutchTorqueNm=*/0.0,
                                    dt});
            // ---- Creep-drag relief gating (COMMON to every coupling path) ----
            // A coupling that PINS the wheels to the road (PIN) would otherwise
            // lug the engine through 1st-gear road-implied RPM at low speed /
            // standstill (the creep-lug + standstill-stall chain). The relief
            // opens the clutch (desiredPressure -> 0) there. Gated on VEHICLE
            // SPEED (creepRegimeRelief), NOT road-implied RPM -- road-implied RPM
            // is gear-dependent and dips below idle at low gears even on moving
            // wheels, so gating on it wrongly opens the clutch and blocks the
            // moving-wheel bump-start (the creep-fix regression). The slip-band
            // rescue is a secondary road-implied floor. This gating used to live
            // in the LEGACY else-branch only; the declarative model path
            // (ClutchMap default / TorqueConverter) skipped it, so the TC's floor
            // + the rate-limiter's one-frame residual (0.05-0.068 -> ~600-800Nm
            // drag at maxClutchTorque=12kNm) dipped the standstill engine to
            // ~20rpm at t=7.78. Hoisting it here gives every model the same
            // standstill decouple.
            const bool engineLugging = engineRpmFeedback_ <
                profile_.idleRpm + EngineSimDefaults::CREEP_RELIEF_TRIGGER_MARGIN_RPM;
            const bool creepRegimeRelief =
                signal.speedKmh < profile_.creepReliefThresholdKmh;
            const bool slipBandLugRescue =
                roadSpeedImpliedRpm < profile_.idleRpm * twin::kLockEngageIdleFactor;
            output.creepReliefFired =
                coupling_->relievesCreepDragAtStandstill() &&
                engineLugging &&
                (creepRegimeRelief || slipBandLugRescue);

            double desiredPressure;
            const bool tcMode = (couplingModelKind_ == twin::CouplingModelKind::TorqueConverter);
            if (tcMode) {
                // PROPER torque converter (SCS direct-torque fluid coupling). The
                // fluid IS the load path: the engine is ALWAYS loaded by the
                // converter's K*N^2 pump law (stall multiplication at low speed,
                // 1:1 lockup at cruise). The friction clutch is held OPEN by the
                // Transmission (it would rigidly lock the engine to the pinned
                // wheels and stall).
                //
                // The clutch pressure drives the converter's CAPACITY SCALE (see
                // transmission.cpp: the TC mode sets capacityScale = clutchPressure
                // and zeroes the friction clutch). The converter's OWN model
                // (TorqueConverter::compute) already returns the correct smooth,
                // ROAD-DRIVEN pressure: a MODERATE creep capacity at standstill
                // (the fluid loads a flaring engine to its stall speed while
                // slipping against the pinned wheel — no free-rev, no rigid
                // couple), ramping monotonically through the slip band to a full
                // 1.0 LOCKUP at cruise (road-implied > idle*1.6). The ramp is a
                // function of road-implied rpm ONLY (never engine rpm / the speed
                // ratio), so it cannot feed back into engine rpm and cannot
                // chatter. We MUST use that smooth ramp here — forcing 1.0 at
                // standstill (the old code) set capacityScale=1.0, which rigidly
                // coupled the engine to the CSV-pinned stationary wheel and drove
                // the ±345 rpm limit cycle the driveability gate flags
                // (NO_OSCILLATION). Neutral opens it fully (engine free-revs,
                // correct). The shift logic (updateShiftExecution) overrides
                // clutchPressure_ during SHIFTING to keep the engine loaded, so we
                // must NOT touch desiredPressure there.
                desiredPressure = (gearbox_->getCurrentGear() >= 1)
                    ? couplingOut.clutchPressure
                    : 0.0;
                // No creep-relief in TC mode: the converter's fluid slip IS the
                // standstill decouple (gentle load, no stall, no free-rev). Zeroing
                // the pressure here would fully open the fluid path and let the
                // engine free-rev under throttle — the exact failure we replaced.
                output.creepReliefFired = false;
            } else if (twin::modelOwnsPressure(couplingOut)) {
                // Declarative model (ClutchMap default, or TorqueConverter): use
                // its smooth floored pressure directly. The creep-relief (above)
                // applies uniformly below; no per-branch binary relief is needed
                // because the curve itself never bang-bangs.
                desiredPressure = couplingOut.clutchPressure;
            } else {
                // LEGACY inline path (slip-lock + PIN lockOverride + launch). Kept
                // for A/B (--coupling-model legacy reproduces the old oscillation
                // when paired with the legacy slip-lock slam-back). The creep
                // relief now applies uniformly after this block.
                const auto slip = computeSlipLockPressure(
                    twin::SlipLockInput{engineRpmFeedback_,
                                        roadSpeedImpliedRpm,
                                        signal.throttleFraction,
                                        profile_.idleRpm,
                                        profile_.redlineRpm},
                    /*maxCreepPressure=*/0.10);
                const double lockOverride = coupling_->clutchLockOverride(
                    engineRpmFeedback_, roadSpeedImpliedRpm, signal.throttleFraction,
                    profile_.idleRpm, profile_.redlineRpm);
                desiredPressure = (lockOverride >= 0.0) ? lockOverride : slip.clutchPressure;
                // Launch (torque converter): stall-gated launch pressure for the
                // modes whose sim speed is independent (Free/Torque). PIN never
                // launches — its vehicle-speed constraint drives the wheels.
                const double launchPressure = coupling_->launchAssistAtStandstill()
                    ? computeLaunchPressure(
                        twin::LaunchPressureInput{engineRpmFeedback_,
                                                  roadSpeedImpliedRpm,
                                                  signal.throttleFraction,
                                                  profile_.idleRpm,
                                                  profile_.redlineRpm})
                    : twin::LAUNCH_PRESSURE_DEFER;
                if (launchPressure != twin::LAUNCH_PRESSURE_DEFER) {
                    desiredPressure = launchPressure;
                }
            }

            // Apply the creep-drag relief uniformly (every non-TC path). Opening
            // the clutch decouples the engine so it idles instead of lugging
            // against road-implied RPM; the relief-idle-sustain floor holds it
            // near idle through the open clutch (the M156 droops to ~750 on the
            // plain 5% idle-sustain floor alone). TC mode skips this (see above:
            // the converter's fluid slip is the decouple).
            if (!tcMode && output.creepReliefFired) {
                desiredPressure = 0.0;
                output.throttle = std::max(throttleSmoother_.getCurrentValue(),
                                           EngineSimDefaults::RELIEF_IDLE_SUSTAIN_THROTTLE);
            }

            // RATE-LIMIT the actual clutch pressure toward the desired. This is
            // the anti-slam fix: without it the relief released the clutch to 0
            // in one frame and the slip-lock slammed it back to ~100% the next,
            // crashing the engine 4700→771 rpm. Asymmetric — the relief OPEN is
            // fast (a lugging engine must be decoupled before it death-spirals;
            // 0.076→0 in ~1 frame at 10/s), the re-ENGAGE is smooth (0→1 in
            // ~0.33s at 3/s, no slam). std::clamp(delta, -maxRelease, maxEngage)
            // gives the asymmetry: negative deltas (release) are bounded by the
            // fast rate, positive deltas (engage) by the slow rate.
            //
            // The relief (desiredPressure -> 0 above) does NOT need a special
            // instant-release bypass: the fast release rate (10/s -> ~0.16/frame
            // at 60Hz) already drives the TC's small standstill floor (0.05) to
            // EXACTLY 0 in one frame (0.05 < 0.16), so the standstill engine is
            // decoupled the same frame the relief fires. Bypassing the rate-limit
            // would only change the LEGACY high-clutch case (0.83 -> 0), where it
            // WOULD be a slam — so the uniform rate-limit is kept to preserve the
            // no-slam invariant guarded by CreepRelief_RampsClutchPressure.
            const double maxRelease = EngineSimDefaults::CLUTCH_RELEASE_RATE_PER_SEC * dt;
            const double maxEngage  = EngineSimDefaults::CLUTCH_ENGAGE_RATE_PER_SEC  * dt;
            const double clutchDelta = desiredPressure - clutchPressure_;
            if (tcMode) {
                // Torque-converter mode: the converter capacity scale is NOT the
                // friction-clutch pressure. The converter's own fluid slip is what
                // smooths engagement/launch, so the capacity must track the desired
                // value IMMEDIATELY (1.0 in gear, 0 in neutral). Rate-limiting it
                // through the friction-clutch engage rate (3/s -> ~6s to full) left
                // the converter weakly coupled for seconds after a gear engage, so
                // the engine free-revved (7000+ rpm) under throttle until the scale
                // crawled up — the NO_FREE_REV / NO_HI_THROTTLE_FREE_REV failures.
                // Setting it directly couples the engine to the fluid path at once;
                // the converter's stall/lockup physics then does the launch/creep
                // work with no slam (there is no rigid clutch to slam).
                clutchPressure_ = desiredPressure;
            } else {
                clutchPressure_ += std::clamp(clutchDelta, -maxRelease, maxEngage);
            }
            output.roadImpliedRpm = roadSpeedImpliedRpm;
            // PIN compliance (--pin-tau-ms): the coupling surfaces the RAW CSV
            // target; the chase gives the pin finite response so the wheels -
            // and with them the engine rpm/pitch - GLIDE between the CSV's
            // held road-speed levels instead of teleporting at the ~5.5 Hz CAN
            // cadence (the "piano keys" staircase). tau=0 is the raw target
            // untouched. Scoped to THIS line: the gearbox (signal.speedKmh)
            // and the slip-lock (wheelKmh above) keep the raw speed.
            output.pinVehicleSpeedTargetKmh = pinTargetChase_.update(
                dt, coupling_->vehicleSpeedTargetKmh(signal.speedKmh));
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
            // Surface the road-implied RPM during a shift too (computed from the
            // current gear + the coupling wheel speed) so the driveability gate's
            // NO_FREE_REV check sees the real road-implied speed instead of the
            // default 0 and does not false-flag the brief over-rev while the gear
            // changes. Mirrors the RUNNING branch's roadImpliedRpm assignment.
            output.roadImpliedRpm = roadSpeedImpliedRpmFor(
                coupling_->slipLockWheelSpeedKmh(vehicleSpeedFeedbackKmh_, signal.speedKmh));
            break;
    }

    output.clutchPressure = clutchPressure_;
    output.couplingIsTorqueConverter =
        (couplingModelKind_ == twin::CouplingModelKind::TorqueConverter);
    output.gearSelector = selector_;
    return output;
}

bool VirtualIceTwin::restartIfStalled(TwinOutput& output, double dt) {
    // Re-crank period: a crank attempt gets the same 3s budget as the
    // initial OFF->CRANK crank (CRANK_FALLBACK_DURATION_S). The bridge's
    // CrankingController::engageStarter is a momentary TOGGLE -- calling
    // it with starterButton=true while already Cranking forces the phase
    // BACK to Stopped (CrankingController.cpp:27-31) -- so the twin must
    // pulse the starter for ONE tick to ENGAGE, then NOT re-toggle while
    // the bridge cranks. The bridge's step() owns the crank: it keeps
    // starterMotor=true in Cranking until the engine catches. This period
    // bounds RETRY: if the engine is still stalled this long after the
    // last edge (the crank failed to raise rpm above the stall bar), fire
    // one fresh edge for a new clean attempt. Normal cranks catch in well
    // under this, so the cooldown never re-fires in the happy path; it is
    // the safety net the old held-starter gave crudely (every tick) and
    // which a single edge alone does not.
    constexpr double kRecrankPeriodS = 3.0;
    reCrankCooldownS_ = std::max(0.0, reCrankCooldownS_ - dt);
    if (engineRpmFeedback_ > CrankingController::STOPPED_RPM) {
        // Engine alive: re-arm the cooldown so the next genuine stall
        // fires a fresh edge promptly.
        reCrankCooldownS_ = 0.0;
        return false;
    }
    // Pulse the starter for ONE tick, then wait kRecrankPeriodS before
    // retrying -- never every frame. A held starter re-toggles
    // engageStarter each tick and drives a Stopped<->Cranking limit
    // cycle (starter 1/0/1/0 for hundreds of frames -- the very
    // anti-pattern documented in the CRANKING case). One edge engages;
    // the cooldown gives the bridge a clean crank window and retries
    // only on failure.
    if (reCrankCooldownS_ <= 0.0) {
        output.starterMotor = true;  // one-tick edge
        reCrankCooldownS_ = kRecrankPeriodS;
    }
    // Starter/crank handoff = the one place the idle-hold integral is
    // hard-flushed: a wound-up term re-engaging on a fresh catch is the
    // classic idle-flare source. Everywhere else the integral decays
    // (tau ~2s), not flushes.
    idleHoldIntegralPct_ = 0.0;
    idleHoldOutputPct_ = 0.0;
    idleHoldActive_ = false;
    output.throttle = std::max(
        std::max(output.throttle, EngineSimDefaults::CRANKING_THROTTLE),
        EngineSimDefaults::IDLE_SUSTAIN_THROTTLE);
    return true;
}

double VirtualIceTwin::idleHoldFloor(double dt, double feedbackRpm) {
    // Idle-hold controller (the RUNNING case's not-stalled guard): a small PI
    // on (idleRpm - feedback) whose output FLOORS the throttle (see the header
    // for the floor/semantics contract). Replaces the old static 5%
    // IDLE_SUSTAIN_THROTTLE floor, which could not hold the engine against the
    // converter's low-speed capacity drag. All constants are FRACTIONS OF
    // FULL THROTTLE (0.20 = 20%):
    //   - engage below idle, release at idle + 150 rpm hysteresis so the
    //     one-step-arrears feedback cannot chatter the band;
    //   - P = 0.05%/rpm (=5e-4 per rpm), immediate authority, clamped
    //     [5%, 20%] — a -600rpm sag commands +30% -> clamps to 20% same-tick;
    //     the 20% ceiling sits far below kickdown (>=95%) and the WOT gate
    //     (>=90%), so it cannot disturb shift logic. NOTE: doubling this gain
    //     to 0.1%/rpm REGRESSED the sf0 sweep (RNZ 82 -> 332, a 549-frame
    //     dead episode): the marginal cold-start trajectory is entry-state
    //     sensitive. Do not retune without the full sf0/60/95/120 sweep.
    //   - I trims the steady-state drag: tau 1s, rate-limited to 2%/s,
    //     capped at 8%. While disengaged the integral DECAYS (tau 2s)
    //     instead of flushing — the flush reset the hold offset every
    //     sawtooth cycle; the hard flush lives only at the starter/crank
    //     handoff in the RUNNING case's stalled branch (flare risk).
    //   - Release ramps down at 40%/s (0.67%/frame): smooth per frame, but
    //     fast enough that the engine does not coast ~700rpm past the release
    //     point on the residual floor (a 10%/s ramp held ~18% for a full
    //     second and drove the 950->1800 overshoot in the sweep traces).
    constexpr double kReleaseMarginRpm = 150.0;
    constexpr double kMaxFloor = 0.20;
    constexpr double kKpPerRpm = 5e-4;
    constexpr double kIntegralTauS = 1.0;
    constexpr double kIntegralCap = 0.08;
    constexpr double kIntegralRatePerS = 0.02;
    constexpr double kReleaseRatePerS = 0.40;
    constexpr double kIntegralDecayTauS = 2.0;
    const double minFloor = EngineSimDefaults::IDLE_SUSTAIN_THROTTLE;

    // State machine (flat on purpose — this grew out of a 5-deep nested
    // block that Sonar rightly flagged):
    //   release at idle + margin (integral RETAINED — decays below; flushing
    //   here reset the hold offset on every sawtooth cycle and re-created the
    //   droop each time; the hard flush lives at the starter/crank handoff in
    //   the RUNNING case's stalled branch, where the flare risk actually is);
    //   engage below idle; hysteresis in between holds the current state.
    if (idleHoldActive_ && feedbackRpm > profile_.idleRpm + kReleaseMarginRpm) {
        idleHoldActive_ = false;
    }
    if (!idleHoldActive_ && feedbackRpm < profile_.idleRpm) {
        idleHoldActive_ = true;
        idleHoldOutputPct_ = minFloor;
    }

    if (idleHoldActive_) {
        const double errorRpm = profile_.idleRpm - feedbackRpm;
        // P: immediate, un-rate-limited — the engine dies in under a second
        // under a heavy drag; a ramp would arrive after the funeral.
        const double pTerm = kKpPerRpm * errorRpm;
        // I: slow trim, rate-limited both directions, hard-capped.
        const double integIncr = std::clamp(
            kKpPerRpm * errorRpm * dt / kIntegralTauS,
            -kIntegralRatePerS * dt, kIntegralRatePerS * dt);
        idleHoldIntegralPct_ = std::clamp(idleHoldIntegralPct_ + integIncr,
                                          0.0, kIntegralCap);
        const double target = std::clamp(pTerm + idleHoldIntegralPct_,
                                         minFloor, kMaxFloor);
        // Release-direction ramp only: upward steps inside the clamp are
        // harmless (see above), downward ramps keep the gearbox's throttle
        // input smooth.
        idleHoldOutputPct_ =
            (target < idleHoldOutputPct_)
                ? std::max(target, idleHoldOutputPct_ - kReleaseRatePerS * dt)
                : target;
        return idleHoldOutputPct_;
    }

    // Disengaged: exponential integral decay (tau 2s) — short releases
    // across the idle band keep most of the learned hold offset; a genuinely
    // long release forgets it.
    idleHoldIntegralPct_ *= std::exp(-dt / kIntegralDecayTauS);
    if (idleHoldOutputPct_ <= minFloor) {
        return 0.0;  // fully released, nothing to floor
    }
    // Still ramping the residual floor down — the release-direction rate
    // limit above, continued.
    idleHoldOutputPct_ = std::max(minFloor,
                                  idleHoldOutputPct_ - kReleaseRatePerS * dt);
    return idleHoldOutputPct_;
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

    if (const bool tcMode =
            (couplingModelKind_ == twin::CouplingModelKind::TorqueConverter);
        tcMode) {
        // Torque-converter mode: the converter (not the friction clutch) is the
        // coupling path, and the Transmission holds the friction clutch OPEN
        // through the whole shift (transmission.cpp::update zeroes the clutch
        // torque while a converter is installed). So the shift must NOT open the
        // converter's capacity — doing so (the friction-clutch floor/ramp below)
        // weakly coupled the engine during the shift and let it free-rev to
        // 7000+ rpm under throttle (the NO_FREE_REV / NO_HI_THROTTLE_FREE_REV
        // failures). The converter's own fluid slip carries the shift smoothly;
        // holding capacity at 1.0 keeps the engine loaded the entire time. The
        // gear change still happens via gearbox_->update() in the pause below.
        clutchPressure_ = 1.0;
        if (shiftTimerS_ > disengageDuration + pauseDuration / 2.0) {
            gearbox_->update(0, 0, 0);
        }
        if (shiftTimerS_ > disengageDuration + pauseDuration + reengageDuration) {
            state_ = TwinState::RUNNING;
        }
        return;
    }

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
