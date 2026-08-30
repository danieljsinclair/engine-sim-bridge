// PinTargetChase.cpp - see PinTargetChase.h for the design contract.
#include "twin/PinTargetChase.h"

#include <cmath>

namespace twin {

double PinTargetChase::update(double dt, double rawTargetKmh) {
    // Rigid pin (the default): surface the raw target untouched. No state is
    // read or written, so tau=0 is bit-identical to the unfiltered pin.
    if (tauMs_ <= 0.0) return rawTargetKmh;

    // "No pin" sentinel: pass through verbatim and rearm so the next pinned
    // target engages fresh (snaps) instead of gliding from a stale value.
    if (rawTargetKmh < 0.0) {
        reset();
        return rawTargetKmh;
    }

    // Fresh engage: adopt the target at once. A moving vehicle re-pinning
    // (e.g. after a neutral spell) must not be braked to a crawl by a ramp
    // from the last-pinned value.
    if (!engaged_) {
        engaged_ = true;
        valueKmh_ = rawTargetKmh;
        rateKmhPerS_ = 0.0;
        return valueKmh_;
    }

    // Critically-damped second-order chase (poles both at -1/tau):
    //     x'' = w^2 (u - x) - 2 w x',   w = 1/tau
    // integrated semi-implicitly (rate first, then value with the new rate).
    // The equilibrium at value == target with zero rate gives zero
    // steady-state error on a held target; the double pole gives a monotonic,
    // overshoot-free glide whose SLOPE is continuous — no kinks for the ear
    // to hear between held levels. On a SUSTAINED ramp the chase trails by
    // the textbook 2*tau*v (a deliberate, bounded trade: the alternative,
    // velocity feedforward, spikes on the quantization steps and overshoots
    // isolated level changes - measured +24% on a 10 km/h step - a worse
    // artifact than the sub-mph ramp lag it removes). Stability:
    // semi-implicit Euler is stable for w*dt < 2, i.e. any tau >= dt/2;
    // twin frames are 10-20 ms, so even tau = 10 ms has margin.
    const double tauS = tauMs_ / 1000.0;
    const double w = 1.0 / tauS;
    rateKmhPerS_ += (w * w * (rawTargetKmh - valueKmh_) - 2.0 * w * rateKmhPerS_) * dt;
    valueKmh_ += rateKmhPerS_ * dt;
    return valueKmh_;
}

void PinTargetChase::reset() {
    engaged_ = false;
    valueKmh_ = 0.0;
    rateKmhPerS_ = 0.0;
}

}  // namespace twin
