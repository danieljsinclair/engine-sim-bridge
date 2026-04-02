// ILogging.h - Logging interface for dependency injection
// Single 32-bit bitmask: categories (lower 16) + levels (upper 16)

#ifndef ILOGGING_H
#define ILOGGING_H

#include <cstdarg>
#include <cstdint>
#include <cstdio>

// ============================================================================
// LogMask - Single 32-bit bitmask for filtering
// Lower 16 bits: categories | Upper 16 bits: levels
// ============================================================================
namespace LogMask {
    // Categories (lower 16 bits)
    constexpr uint32_t BRIDGE         = 0x00000001;
    constexpr uint32_t BUFFER         = 0x00000002;
    constexpr uint32_t THREADING      = 0x00000004;
    constexpr uint32_t PHYSICS        = 0x00000008;
    constexpr uint32_t AUDIO          = 0x00000010;
    constexpr uint32_t SCRIPT         = 0x00000020;
    constexpr uint32_t ASSET          = 0x00000040;
    constexpr uint32_t DIAGNOSTICS    = 0x00000080;
    constexpr uint32_t API            = 0x00000100;
    constexpr uint32_t UI             = 0x00000200;
    constexpr uint32_t SYNC_PULL      = 0x00000400;
    constexpr uint32_t THREADED_AUDIO = 0x00000800;
    constexpr uint32_t ALL_CATS       = 0x0000FFFF;

    // Levels (upper 16 bits)
    constexpr uint32_t DEBUG          = 0x00010000;
    constexpr uint32_t INFO           = 0x00020000;
    constexpr uint32_t WARN           = 0x00040000;
    constexpr uint32_t ERROR          = 0x00080000;
    constexpr uint32_t ALL_LEVELS     = 0xFFFF0000;

    // Everything
    constexpr uint32_t ALL         = 0xFFFFFFFF;
}

// Abstract logging interface
class ILogging {
public:
    virtual ~ILogging() = default;

    // Raw log with full mask (category | level)
    virtual void log(uint32_t mask, const char* format, ...) = 0;

    // Convenience methods - level baked in, pass category
    // debug(LogMask::AUDIO, "format", ...) -> log(AUDIO | DEBUG, ...)
    virtual void debug(uint32_t category, const char* format, ...) = 0;
    virtual void info(uint32_t category, const char* format, ...) = 0;
    virtual void warning(uint32_t category, const char* format, ...) = 0;
    virtual void error(uint32_t category, const char* format, ...) = 0;

    // Set filter mask (can combine categories and levels)
    // setMask(LogMask::AUDIO | LogMask::ERROR) -> only audio errors
    virtual void setMask(uint32_t mask) = 0;
    virtual uint32_t getMask() const = 0;
};

// Default console implementation
class ConsoleLogger : public ILogging {
public:
    ConsoleLogger() : mask_(LogMask::ALL) {}
    ~ConsoleLogger() override = default;

    void log(uint32_t mask, const char* format, ...) override;
    void debug(uint32_t category, const char* format, ...) override;
    void info(uint32_t category, const char* format, ...) override;
    void warning(uint32_t category, const char* format, ...) override;
    void error(uint32_t category, const char* format, ...) override;

    void setMask(uint32_t mask) override { mask_ = mask; }
    uint32_t getMask() const override { return mask_; }

private:
    uint32_t mask_;
    const char* levelToString(uint32_t level);
    FILE* getStream(uint32_t level);
    bool shouldLog(uint32_t mask);
    void vlog(uint32_t mask, const char* format, va_list args);
};

#endif // ILOGGING_H
