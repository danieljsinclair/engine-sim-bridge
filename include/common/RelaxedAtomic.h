// RelaxedAtomic.h — atomic wrapper using default memory order (seq_cst)
// No explicit memory_order arguments to avoid sonar S8417.
// On x86-64, default (seq_cst) and relaxed generate identical instructions.
// On ARM, default adds dmb barriers but provides stronger ordering guarantees.

#ifndef RELAXED_ATOMIC_H
#define RELAXED_ATOMIC_H

#include <atomic>
#include <cstdint>

class RelaxedDouble {
public:
    RelaxedDouble() noexcept : value_{} {}
    explicit RelaxedDouble(double v) noexcept : value_(v) {}

    void store(double v) noexcept { value_.store(v); }
    double load() const noexcept { return value_.load(); }
    operator double() const noexcept { return load(); }
    RelaxedDouble& operator=(double v) noexcept { store(v); return *this; }

private:
    std::atomic<double> value_{};
};

class RelaxedInt {
public:
    RelaxedInt() noexcept : value_(0) {}
    explicit RelaxedInt(int32_t v) noexcept : value_(v) {}

    void store(int32_t v) noexcept { value_.store(v); }
    int32_t load() const noexcept { return value_.load(); }
    operator int32_t() const noexcept { return load(); }
    RelaxedInt& operator=(int32_t v) noexcept { store(v); return *this; }

private:
    std::atomic<int32_t> value_;
};

class RelaxedBool {
public:
    RelaxedBool() noexcept : value_(false) {}
    explicit RelaxedBool(bool v) noexcept : value_(v) {}

    void store(bool v) noexcept { value_.store(v); }
    bool load() const noexcept { return value_.load(); }
    operator bool() const noexcept { return load(); }
    RelaxedBool& operator=(bool v) noexcept { store(v); return *this; }

private:
    std::atomic<bool> value_;
};

#endif // RELAXED_ATOMIC_H
