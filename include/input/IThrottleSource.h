#ifndef ITHROTTLE_SOURCE_H
#define ITHROTTLE_SOURCE_H

namespace input {

class IThrottleSource {
public:
    virtual ~IThrottleSource() = default;
    virtual double pollThrottle() = 0;
    virtual bool shouldContinue() const = 0;
};

}

#endif
