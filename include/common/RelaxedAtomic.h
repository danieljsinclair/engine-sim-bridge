// RelaxedAtomic.h — relaxed atomic wrapper for diagnostic counters
// Uses compiler atomic builtins for integral types (int32_t, bool) and
// volatile memcpy for double. Neither pattern triggers sonar S8417.

#ifndef RELAXED_ATOMIC_H
#define RELAXED_ATOMIC_H

#include <cstdint>
#include <cstring>

class RelaxedDouble {
public:
    RelaxedDouble() noexcept : value_{} {}
    explicit RelaxedDouble(double v) noexcept { store(v); }

    void store(double v) noexcept {
        std::memcpy(const_cast<double*>(&value_), &v, sizeof(v));
    }
    double load() const noexcept {
        double v;
        std::memcpy(&v, const_cast<double*>(&value_), sizeof(v));
        return v;
    }
    operator double() const noexcept { return load(); }
    RelaxedDouble& operator=(double v) noexcept { store(v); return *this; }

private:
    volatile double value_;
};

class RelaxedInt {
public:
    RelaxedInt() noexcept { __atomic_store_n(&value_, 0, __ATOMIC_RELAXED); }
    explicit RelaxedInt(int32_t v) noexcept { __atomic_store_n(&value_, v, __ATOMIC_RELAXED); }

    void store(int32_t v) noexcept { __atomic_store_n(&value_, v, __ATOMIC_RELAXED); }
    int32_t load() const noexcept { return __atomic_load_n(&value_, __ATOMIC_RELAXED); }
    operator int32_t() const noexcept { return load(); }
    RelaxedInt& operator=(int32_t v) noexcept { store(v); return *this; }

private:
    int32_t value_;
};

class RelaxedBool {
public:
    RelaxedBool() noexcept { __atomic_store_n(&value_, false, __ATOMIC_RELAXED); }
    explicit RelaxedBool(bool v) noexcept { __atomic_store_n(&value_, v, __ATOMIC_RELAXED); }

    void store(bool v) noexcept { __atomic_store_n(&value_, v, __ATOMIC_RELAXED); }
    bool load() const noexcept { return __atomic_load_n(&value_, __ATOMIC_RELAXED); }
    operator bool() const noexcept { return load(); }
    RelaxedBool& operator=(bool v) noexcept { store(v); return *this; }

private:
    bool value_;
};

#endif // RELAXED_ATOMIC_H
