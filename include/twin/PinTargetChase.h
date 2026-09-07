// PinTargetChase.h - compliance filter for the PIN wheel-coupling target.
//
// The road-speed signal the PIN coupling pins the sim vehicle speed to updates
// only ~5.5 Hz in ~0.9 km/h steps (CAN median-hold), so the rigid pin teleports
// the wheel speed - and with it the engine rpm/pitch - onto each held level:
// the audible "piano keys" staircase. This filter gives the pin finite
// response: the target is CHASED by a critically-damped second-order response
// (double pole at -1/tau), so rpm glides between levels instead of stepping.
// Held targets converge exactly; a SUSTAINED ramp is trailed by the textbook
// 2*tau*v (bounded, sub-mph at road ramp rates - see the ramp contract test).
// tau <= 0 (the default) is EXACTLY the rigid passthrough - no state is
// touched and the raw target is returned bit-identical (the --pin-tau-ms 0
// regression contract).
//
// Scope: the PIN TARGET ONLY. The gearbox shift map and the slip-lock math
// still see the raw road speed (VirtualIceTwin feeds them signal.speedKmh);
// interpolating the whole road-speed signal was tried before and flipped gear
// decisions (see ReplayTelemetryProvider's raw-feed note).
#ifndef TWIN_PIN_TARGET_CHASE_H
#define TWIN_PIN_TARGET_CHASE_H

namespace twin {

class PinTargetChase {
public:
    void setTauMs(double tauMs) { tauMs_ = tauMs; }
    double getTauMs() const { return tauMs_; }

    // Advance the chase by dt seconds toward rawTargetKmh and return the
    // target to surface. raw < 0 is the "no pin" sentinel: passed through
    // verbatim and the chase rearms (the next pinned target snaps, matching
    // the rigid behavior on a fresh engage).
    double update(double dt, double rawTargetKmh);

    void reset();

private:
    double tauMs_ = 0.0;
    bool engaged_ = false;
    double valueKmh_ = 0.0;
    double rateKmhPerS_ = 0.0;
};

// --pin-tau-ms stability-window warning (owner directive: tuning toggles are
// never restricted - warn, don't reject). Returns the warning text when tau
// sits outside the stable window, nullptr when it is fine. tau <= 0 is the
// documented rigid passthrough (OFF) and never warns. Any value is ACCEPTED:
// PinTargetChase clamps tau <= 0 to rigid, and every positive value is a legal
// (if ill-advised) experiment the owner may want to run.
// Empirical map (see docs/architecture/pin-tau-compliance.md):
//   20-50 ms  drivetrain bifurcation (50 ms runs away to ~207 mph / 15.5k rpm)
//   60-1000 ms  stable window (recommended; default 150)
//   >3000 ms  over-damped (15000 ms halves road speed)
// Moved from the CLI's TelemetryProviderFactory.h (consolidation wave B) so
// the thresholds + text live beside the filter they describe.
inline const char* pinTauWarningText(double tauMs) {
    if (tauMs > 0.0 && tauMs < 60.0) {
        return "--pin-tau-ms 60-1000 is the stable window; below 60 ms the drivetrain can "
               "bifurcate (20-50 ms bench runs ran away to ~207 mph). Continuing with your value.";
    }
    if (tauMs > 3000.0) {
        return "--pin-tau-ms above 3000 ms is over-damped (15000 ms halves road speed on the "
               "bench). 60-1000 ms is the stable window. Continuing with your value.";
    }
    return nullptr;
}

}  // namespace twin

#endif  // TWIN_PIN_TARGET_CHASE_H
