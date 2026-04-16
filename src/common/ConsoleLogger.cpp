// ConsoleLogger.cpp - Default console logging implementation
// stdout for debug/info, stderr for warn/error

#include "common/ILogging.h"
#include <cstdarg>
#include <cstdio>

const char* ConsoleLogger::levelToString(uint32_t level) {
    if (level == LogMask::DEBUG) return "DEBUG";
    if (level == LogMask::INFO)  return "INFO";
    if (level == LogMask::WARN)  return "WARN";
    if (level == LogMask::ERROR) return "ERROR";
    return "UNKNOWN";
}

FILE* ConsoleLogger::getStream(uint32_t level) {
    return (level == LogMask::DEBUG || level == LogMask::INFO) ? stdout : stderr;
}

bool ConsoleLogger::shouldLog(uint32_t mask) {
    uint32_t category = mask & 0x0000FFFF;
    uint32_t level = mask & 0xFFFF0000;
    return ((category & mask_) || (mask_ & LogMask::ALL_CATS)) &&
           ((level & mask_) || (mask_ & LogMask::ALL_LEVELS));
}

void ConsoleLogger::log(uint32_t mask, const char* format, ...) {
    va_list args;
    va_start(args, format);
    vlog(mask, format, args);
    va_end(args);
}

// Internal method that takes va_list
void ConsoleLogger::vlog(uint32_t mask, const char* format, va_list args) {
    if (!shouldLog(mask)) return;

    uint32_t level = mask & 0xFFFF0000;
    FILE* stream = getStream(level);

    fprintf(stream, "[%s] ", levelToString(level));
    vfprintf(stream, format, args);
    fprintf(stream, "\n");
    fflush(stream);
}

// Convenience methods - OR level with category
void ConsoleLogger::debug(uint32_t category, const char* format, ...) {
    va_list args; va_start(args, format);
    vlog(category | LogMask::DEBUG, format, args);
    va_end(args);
}

void ConsoleLogger::info(uint32_t category, const char* format, ...) {
    va_list args; va_start(args, format);
    vlog(category | LogMask::INFO, format, args);
    va_end(args);
}

void ConsoleLogger::warning(uint32_t category, const char* format, ...) {
    va_list args; va_start(args, format);
    vlog(category | LogMask::WARN, format, args);
    va_end(args);
}

void ConsoleLogger::error(uint32_t category, const char* format, ...) {
    va_list args; va_start(args, format);
    vlog(category | LogMask::ERROR, format, args);
    va_end(args);
}
