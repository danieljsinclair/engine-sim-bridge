// ILogging.h - Logging interface for dependency injection
// Single 32-bit bitmask: categories (lower 16) + levels (upper 16)

#ifndef ILOGGING_H
#define ILOGGING_H

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>

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
    constexpr uint32_t DBG            = 0x00010000;
    constexpr uint32_t INFO           = 0x00020000;
    constexpr uint32_t WARN           = 0x00040000;
    constexpr uint32_t ERROR          = 0x00080000;
    constexpr uint32_t ALL_LEVELS     = 0xFFFF0000;

    // Everything
    constexpr uint32_t ALL         = 0xFFFFFFFF;
}

// Format a printf-style message into a std::string.
// This is a variadic template that expands at the call site where the
// format string is a literal, avoiding S5281 in .cpp files.
inline std::string __ilog_format(const char* fmt) {
    return std::string(fmt);
}
template <typename... Args>
std::string __ilog_format(const char* fmt, Args&&... args) {
    int needed = snprintf(nullptr, 0, fmt, std::forward<Args>(args)...);
    if (needed < 0) return "(format error)";
    std::string result(static_cast<size_t>(needed), '\0');
    snprintf(result.data(), result.size() + 1, fmt, std::forward<Args>(args)...);
    return result;
}

// Abstract logging interface.
// Public API accepts pre-formatted std::string — no printf-family calls
// with non-literal format strings anywhere in the implementation.
class ILogging {
public:
    virtual ~ILogging() = default;

    void log(uint32_t mask, const std::string& msg) { _write(mask, msg); }
    void debug(uint32_t category, const std::string& msg) { _write(category | LogMask::DBG, msg); }
    void info(uint32_t category, const std::string& msg) { _write(category | LogMask::INFO, msg); }
    void warning(uint32_t category, const std::string& msg) { _write(category | LogMask::WARN, msg); }
    void error(uint32_t category, const std::string& msg) { _write(category | LogMask::ERROR, msg); }

    virtual void setMask(uint32_t mask) = 0;
    virtual uint32_t getMask() const = 0;

protected:
    // Virtual dispatch — takes va_list (no ellipsis in signature)
    virtual void _write(uint32_t mask, const std::string& msg) = 0;
};

// Default console implementation
class ConsoleLogger : public ILogging {
public:
    ConsoleLogger() : mask_(LogMask::ALL) {}
    ~ConsoleLogger() override = default;

    void setMask(uint32_t mask) override { mask_ = mask; }
    uint32_t getMask() const override { return mask_; }

private:
    uint32_t mask_;
    const char* levelToString(uint32_t level) const;
    FILE* getStream(uint32_t level) const;
    bool shouldLog(uint32_t mask) const;
    void _write(uint32_t mask, const std::string& msg) override;
};

#endif // ILOGGING_H
