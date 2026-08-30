#ifndef TWIN_AUTOMATIC_GEARBOX_H
#define TWIN_AUTOMATIC_GEARBOX_H

#include <memory>

#include <twin/IceVehicleProfile.h>
#include <twin/IGearboxLogger.h>
#include <twin/DrivetrainQuantities.h>
#include <twin/ITorqueHint.h>
#include <simulator/GearConventions.h>

namespace twin {

// A declarative automatic gearbox modelled on a 1960s hydraulic unit: a speed
// governor, a throttle valve and a spring-detented shift valve. The shift
// tables are the sole authority — given speed and throttle they interpolate the
// next-gear threshold — and the separate downshift table provides hysteresis by
// construction (its thresholds sit below the upshift thresholds for every gear
// and throttle, so a single speed can never satisfy both and hunting is
// impossible — enforced as an init-time invariant). The only override is
// kickdown: a sudden large throttle increase forces one RPM-safe downshift. A
// short dwell after any shift suppresses sub-frame oscillation; kickdown is
// exempt. An OPTIONAL torque hint (ITorqueHint strategy) biases the table
// lookup; NullTorqueHint (default) yields correct behaviour with no torque data.
class AutomaticGearbox {
public:
    explicit AutomaticGearbox(const IceVehicleProfile& profile);

    // Legacy 3-arg update (throttle/speed only). Equivalent to the torque-aware
    // update with a NullTorqueHint (no torque bias).
    void update(double dt, double speedKmh, double throttleFraction);

    // Torque- and selector-aware update. Torque is wrapped in a SimTorqueHint
    // (bounded throttle-equivalent bias) and consumed by the declarative
    // decision; it touches ONLY gear selection, never speed/torque injection.
    // In NEUTRAL/PARK/REVERSE no shift is requested and the current gear holds.
    void update(double dt, double speedKmh, double throttleFraction,
                double drivetrainTorqueNm);

    int getCurrentGear() const;
    bool requestsShift() const;
    int getTargetGear() const;
    bool isInKickdown() const;

    void setLogger(IGearboxLogger* logger) { logger_ = logger; }
    IGearboxLogger* getLogger() const { return logger_; }
    void setTwinContext(int twinState, double clutchPressure, double speedFeedbackKmh, double rpmFeedback) {
        twinState_ = twinState;
        clutchPressureFeedback_ = clutchPressure;
        speedFeedbackKmh_ = speedFeedbackKmh;
        rpmFeedback_ = rpmFeedback;
        // Store the ACTUAL engine feedback rpm as a strong type so the anti-lug
        // guard can NEVER be fed a road-speed-derived rpm. The CommandedRoadSpeed
        // (signal.target) and ActualEngineRpm (engine-sim feedback) are separate
        // types with no conversion; reintroducing the death-spiral bug — deriving
        // the guard rpm from the commanded road speed — is a compile error.
        actualEngineRpm_ = ActualEngineRpm{rpmFeedback};
        // The rpm feedback is a BOUNDARY input (engine-sim). Until context is set
        // (real sim path always calls setTwinContext), the guard must stay inert
        // rather than treat the default 0.0 as a lug. Unit tests that drive
        // update() directly without context keep the legacy table behaviour.
        rpmFeedbackValid_ = true;
    }
    // Selector drives the neutral/park hold behaviour (AC6). Forward shifting
    // only happens in DRIVE (or a manual forward position).
    void setGearSelector(bridge::GearSelector s) { selector_ = s; }
    bridge::GearSelector getGearSelector() const { return selector_; }

    int getLastShiftDirection() const { return lastShiftDirection_; }

private:
    // A single frame's declarative shift verdict.
    struct ShiftDecision {
        int gear;        // resolved target gear
        int direction;   // +1 upshift, -1 downshift, 0 hold
        bool kickdown;   // true when the shift was a kickdown override
    };

    // All inputs to the pure decision in one value. The decision reads ONLY this
    // aggregate (+ the ITorqueHint + the profile) — never mutable member state —
    // so it is a pure function of its arguments.
    struct ShiftFrame {
        int currentGear;
        CommandedRoadSpeed speed;     // table authority (the "where you're going")
        ActualEngineRpm actualRpm;    // anti-lug guard authority (the real engine)
        ThrottleState throttle;       // conditioned (smoothed) throttle
        bool kickdownActive;
        bool rpmFeedbackValid;        // guard inert until engine feedback is wired
        double timeSinceLastShiftS;
        bool hasShiftedBefore;
    };

    const IceVehicleProfile& profile_;
    int currentGear_ = 1;
    int targetGear_ = 1;
    bool requestsShift_ = false;
    double timeSinceLastShiftS_ = 0.0;
    bool hasShiftedBefore_ = false;
    int lastShiftDirection_ = 0;

    // Throttle input conditioning: a one-pole low-pass rejects sub-percent CAN
    // jitter so the table input is stable, and a rolling delta within the
    // kickdown window detects sudden pedal stabs. This is signal pre-conditioning
    // (the decision itself holds no mutable counters); kept minimal and isolated.
    double smoothedThrottle_ = -1.0;
    double previousThrottle_ = 0.0;
    double throttleDeltaHistory_ = 0.0;
    double throttleDeltaTimeS_ = 0.0;
    bool kickdownActive_ = false;
    // One kickdown per sustained-WOT episode: set when a kickdown shift
    // executes, cleared when the throttle demand falls back below the
    // kickdown threshold (re-arming for the next stab).
    bool wotKickdownConsumed_ = false;

    // OCP torque-hint strategy. Defaults to NullTorqueHint (correct with no
    // torque data); the torque-aware update() swaps in a SimTorqueHint. The
    // decision consumes only ITorqueHint::shiftBias() — no `if (hasTorque)`.
    std::unique_ptr<ITorqueHint> torqueHint_;

    IGearboxLogger* logger_ = nullptr;
    int twinState_ = 0;
    double clutchPressureFeedback_ = 0.0;
    double speedFeedbackKmh_ = 0.0;
    double rpmFeedback_ = 0.0;
    // ACTUAL engine feedback rpm, strongly typed. This is the only quantity the
    // anti-lug guard may read. It is set from the engine-sim feedback
    // (rpmFeedback) in setTwinContext and is INDEPENDENT of the commanded road
    // speed — so the guard can never be fed a road-speed-derived rpm.
    ActualEngineRpm actualEngineRpm_ = ActualEngineRpm{0.0};
    // Whether setTwinContext has been called (engine-sim feedback present). The
    // lug guard is inert until true, so direct unit tests that drive update()
    // without context keep the legacy table-only behaviour. The real sim path
    // always calls setTwinContext before update, so the guard is live there.
    bool rpmFeedbackValid_ = false;
    uint64_t frame_ = 0;

    bridge::GearSelector selector_ = bridge::GearSelector::DRIVE;

    // ---- Pure declarative decision ----
    // The sole shift authority. Pure: a function of (frame, torqueHint, profile)
    // with no member reads/writes. Composed of pure predicates over declarative
    // tables (up/down maps are data); the merged anti-lug guard keys on
    // ActualEngineRpm; the torque hint biases the lookup. Kickdown is the single
    // override; a minimal dwell blocks table shifts sub-frame.
    static ShiftDecision decide(const ShiftFrame& frame,
                                const ITorqueHint& torqueHint,
                                const IceVehicleProfile& profile);

    // Thin wrapper that gathers member state into a ShiftFrame and delegates to
    // the pure decide(). Kept as a private member for ergonomic call-sites.
    ShiftDecision decideGear(CommandedRoadSpeed speed) const;

    // Init-time invariant: every downshift cell < its upshift cell per
    // (gear, throttle). The no-hunting proof, enforced declaratively at
    // construction. Fail-fast (ASSERT) if violated.
    static void validateShiftTables(const IceVehicleProfile& profile);

    // Pure table predicates (no member access). `effThrottle` is the
    // torque-biased effective throttle the caller computed once per frame.
    static double getShiftSpeed(int fromGear, int toGear, double throttle,
                                const IceVehicleProfile& profile);
    static double getDownshiftSpeed(int fromGear, int toGear, double throttle,
                                    const IceVehicleProfile& profile);
    static double getEngineRpm(double speedKmh, int gear,
                               const IceVehicleProfile& profile);
    static bool speedExceedsUpshift(double speedKmh, int gear, double effThrottle,
                                    const IceVehicleProfile& profile);
    static bool speedBelowDownshift(double speedKmh, int gear, double effThrottle,
                                    const IceVehicleProfile& profile);
    static int findSafeGear(int currentGear, double speedKmh, int maxDownshifts,
                            const IceVehicleProfile& profile);
    // Merged anti-lug guard. floor = max(idle + margin, downshiftRpmFloor).
    // A STOPPED engine (actualRpm <= engineStoppedRpm) is not lugging — the
    // stall/restart path owns that. Reads ActualEngineRpm (typed) so a
    // road-speed-derived rpm cannot reach it (compile error).
    static bool isEngineLugging(ActualEngineRpm actualRpm, double floorRpm,
                                double engineStoppedRpm);

    // Effective throttle: smoothed throttle + bounded torque bias, clamped.
    static double effectiveThrottle(ThrottleState throttle, const ITorqueHint& hint) {
        return std::clamp(throttle.fraction + hint.shiftBias(), 0.0, 1.0);
    }

    void applyShift(const ShiftDecision& decision);

    // True when the selector is in a forward position that allows shifting.
    bool isShifterInDrive() const;

    // Throttle pre-processing (smoothing + kickdown-delta tracking).
    void smoothThrottleInput(double throttleFraction, double dt);
    void trackThrottleDelta(double throttleFraction, double dt);
    bool shouldKickdown(double throttleFraction) const;

    // Logger snapshot: populate and emit a GearboxLogEntry when logger_ is set.
    void logShiftState(double throttleFraction, double dt, double speedKmh);

    // Core throttle/speed/torque shift decision (shared by both public updates).
    void runShiftLogic(double dt, double speedKmh, double throttleFraction);
};

}

#endif
