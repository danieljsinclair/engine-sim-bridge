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

}  // namespace twin

#endif  // TWIN_PIN_TARGET_CHASE_H
