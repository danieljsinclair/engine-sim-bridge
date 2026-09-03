// ConsoleLogger.cpp - Default console logging implementation
// stdout for debug/info, stderr for warn/error

#include "common/ILogging.h"
#include <cstdio>

const char* ConsoleLogger::levelToString(uint32_t level) const {
    if (level == LogMask::DBG)   return "DEBUG";
    if (level == LogMask::INFO)  return "INFO";
    if (level == LogMask::WARN)  return "WARN";
    if (level == LogMask::ERROR) return "ERROR";
    return "UNKNOWN";
}

FILE* ConsoleLogger::getStream(uint32_t level) const {
    return (level == LogMask::DBG   || level == LogMask::INFO) ? stdout : stderr;
}

bool ConsoleLogger::shouldLog(uint32_t mask) const {
    uint32_t category = mask & 0x0000FFFF;
    uint32_t level = mask & 0xFFFF0000;
    // A half (categories / levels) passes when the mask names the message's
    // bit OR carries the FULL wildcard for that half. The wildcard must test
    // the whole half: a bare AND (mask_ & ALL_LEVELS) is true for ANY level
    // bit, which would make e.g. INFO|WARN|ERROR act as all-levels and defeat
    // level filtering entirely (runMask(false) could never hide DBG).
    bool catOk = (category & mask_) != 0 || (mask_ & 0x0000FFFF) == LogMask::ALL_CATS;
    bool levelOk = (level & mask_) != 0 || (mask_ & 0xFFFF0000) == LogMask::ALL_LEVELS;
    return catOk && levelOk;
}

void ConsoleLogger::_write(uint32_t mask, const std::string& msg) {
    if (!shouldLog(mask)) return;

    uint32_t level = mask & 0xFFFF0000;
    FILE* stream = getStream(level);

    fprintf(stream, "[%s] %s\n", levelToString(level), msg.c_str());
    fflush(stream);
}
