// ILogging.h - Logging interface for dependency injection
// Bridge users can provide their own logging implementation

#ifndef ILOGGING_H
#define ILOGGING_H

#include <cstdarg>

// Log levels for filtering
enum class LogLevel {
    Debug,
    Info,
    Warning,
    Error
};

// Abstract logging interface - bridge users implement this
class ILogging {
public:
    virtual ~ILogging() = default;

    // Log a message with printf-style formatting
    virtual void log(LogLevel level, const char* format, ...) = 0;

    // Convenience methods
    virtual void debug(const char* format, ...) = 0;
    virtual void info(const char* format, ...) = 0;
    virtual void warning(const char* format, ...) = 0;
    virtual void error(const char* format, ...) = 0;
};

// Default implementation that logs to stderr
class StdErrLogging : public ILogging {
public:
    void log(LogLevel level, const char* format, ...) override;
    void debug(const char* format, ...) override;
    void info(const char* format, ...) override;
    void warning(const char* format, ...) override;
    void error(const char* format, ...) override;

    // Allow setting a minimum log level
    void setMinLevel(LogLevel level) { minLevel_ = level; }

private:
    LogLevel minLevel_ = LogLevel::Info;
    const char* levelToString(LogLevel level);
    bool shouldLog(LogLevel level);
};

#endif // ILOGGING_H
