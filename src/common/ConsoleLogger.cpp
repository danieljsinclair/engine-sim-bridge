// ConsoleLogger.cpp - Default console logging implementation
// stdout for debug/info, stderr for warn/error

#include "common/ILogging.h"
#include <cstdarg>
#include <cstdio>
#include <vector>

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

void ConsoleLogger::log(uint32_t mask, const char* format, ...) {
    va_list args;
    va_start(args, format);
    writeLog(mask, format, args);
    va_end(args);
}

// Internal method that takes va_list — formats via vsnprintf to avoid
// vfprintf non-literal format string warning (S5281)
void ConsoleLogger::writeLog(uint32_t mask, const char* format, va_list args) const {
    if (!shouldLog(mask)) return;

    uint32_t level = mask & 0xFFFF0000;
    FILE* stream = getStream(level);

    // Format the message into a buffer using va_list, then write with literal "%s"
    va_list argsCopy;
    va_copy(argsCopy, args);
    int needed = vsnprintf(nullptr, 0, format, argsCopy);
    va_end(argsCopy);

    if (needed < 0) {
        fprintf(stream, "[%s] (format error)\n", levelToString(level));
        fflush(stream);
        return;
    }

    std::vector<char> buf(static_cast<size_t>(needed) + 1);
    vsnprintf(buf.data(), buf.size(), format, args);

    fprintf(stream, "[%s] %s\n", levelToString(level), buf.data());
    fflush(stream);
}

// Convenience methods - OR level with category
void ConsoleLogger::debug(uint32_t category, const char* format, ...) {
    va_list args; va_start(args, format);
    writeLog(category | LogMask::DBG,   format, args);
    va_end(args);
}

void ConsoleLogger::info(uint32_t category, const char* format, ...) {
    va_list args; va_start(args, format);
    writeLog(category | LogMask::INFO, format, args);
    va_end(args);
}

void ConsoleLogger::warning(uint32_t category, const char* format, ...) {
    va_list args; va_start(args, format);
    writeLog(category | LogMask::WARN, format, args);
    va_end(args);
}

void ConsoleLogger::error(uint32_t category, const char* format, ...) {
    va_list args; va_start(args, format);
    writeLog(category | LogMask::ERROR, format, args);
    va_end(args);
}
