#ifndef THROTTLE_SMOOTHER_H
#define THROTTLE_SMOOTHER_H

#include <cmath>

namespace twin {

class ThrottleSmoother {
public:
    explicit ThrottleSmoother(double tauMs = 50.0);

    double update(double dt, double rawThrottle);

    void reset(double initialThrottle = 0.0);

    double getCurrentValue() const { return filteredValue_; }

    void setTauMs(double tauMs) { tauMs_ = tauMs; }

    double getTauMs() const { return tauMs_; }

private:
    double tauMs_;
    double filteredValue_;
};

inline double ThrottleSmoother::update(double dt, double rawThrottle) {
    if (tauMs_ <= 0.0) {
        filteredValue_ = rawThrottle;
        return rawThrottle;
    }

    double alpha = 1.0 - std::exp(-dt * 1000.0 / tauMs_);
    filteredValue_ += (rawThrottle - filteredValue_) * alpha;
    return filteredValue_;
}

}

#endif
