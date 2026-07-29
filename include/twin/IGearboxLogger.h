#ifndef TWIN_IGEARBOX_LOGGER_H
#define TWIN_IGEARBOX_LOGGER_H

#include "twin/GearboxLogEntry.h"

namespace twin {

class IGearboxLogger {
public:
    virtual ~IGearboxLogger() = default;
    virtual void log(const GearboxLogEntry& entry) = 0;
};

}

#endif
