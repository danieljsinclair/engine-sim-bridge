#include <twin/ThrottleSmoother.h>

namespace twin {

ThrottleSmoother::ThrottleSmoother(double tauMs)
    : tauMs_(tauMs), filteredValue_(0.0) {}

void ThrottleSmoother::reset(double initialThrottle) {
    filteredValue_ = initialThrottle;
}

}
