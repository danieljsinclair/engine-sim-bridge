#ifndef TWIN_GEARBOX_CSV_LOGGER_H
#define TWIN_GEARBOX_CSV_LOGGER_H

#include "twin/IGearboxLogger.h"
#include <cstdio>
#include <string>

namespace twin {

class GearboxCsvLogger : public IGearboxLogger {
public:
    explicit GearboxCsvLogger(const std::string& filePath);
    ~GearboxCsvLogger() override;

    void log(const GearboxLogEntry& entry) override;

    bool isOpen() const { return file_ != nullptr; }

private:
    FILE* file_ = nullptr;
    bool wroteHeader_ = false;
};

}

#endif
