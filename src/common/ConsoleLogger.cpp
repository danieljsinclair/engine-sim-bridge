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
    return ((category & mask_) || (mask_ & LogMask::ALL_CATS)) &&
           ((level & mask_) || (mask_ & LogMask::ALL_LEVELS));
}

void ConsoleLogger::_write(uint32_t mask, const std::string& msg) {
    if (!shouldLog(mask)) return;

    uint32_t level = mask & 0xFFFF0000;
    FILE* stream = getStream(level);

    fprintf(stream, "[%s] %s\n", levelToString(level), msg.c_str());
    fflush(stream);
}
