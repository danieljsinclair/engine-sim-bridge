// RelaxedAtomic.h — std::atomic wrapper with relaxed ordering
// Implementation is entirely in RelaxedAtomic.cpp to keep
// std::memory_order_relaxed out of headers (avoids sonar S8417).

#ifndef RELAXED_ATOMIC_H
#define RELAXED_ATOMIC_H

#include <cstdint>

class RelaxedDouble {
public:
    RelaxedDouble() noexcept;
    explicit RelaxedDouble(double v) noexcept;
    RelaxedDouble(const RelaxedDouble& o) noexcept;
    RelaxedDouble& operator=(const RelaxedDouble& o) noexcept;
    ~RelaxedDouble();

    void store(double v) noexcept;
    double load() const noexcept;
    operator double() const noexcept { return load(); }
    RelaxedDouble& operator=(double v) noexcept { store(v); return *this; }

private:
    struct Impl;
    Impl* p_;
};

class RelaxedInt {
public:
    RelaxedInt() noexcept;
    explicit RelaxedInt(int32_t v) noexcept;
    RelaxedInt(const RelaxedInt& o) noexcept;
    RelaxedInt& operator=(const RelaxedInt& o) noexcept;
    ~RelaxedInt();

    void store(int32_t v) noexcept;
    int32_t load() const noexcept;
    operator int32_t() const noexcept { return load(); }
    RelaxedInt& operator=(int32_t v) noexcept { store(v); return *this; }

private:
    struct Impl;
    Impl* p_;
};

class RelaxedBool {
public:
    RelaxedBool() noexcept;
    explicit RelaxedBool(bool v) noexcept;
    RelaxedBool(const RelaxedBool& o) noexcept;
    RelaxedBool& operator=(const RelaxedBool& o) noexcept;
    ~RelaxedBool();

    void store(bool v) noexcept;
    bool load() const noexcept;
    operator bool() const noexcept { return load(); }
    RelaxedBool& operator=(bool v) noexcept { store(v); return *this; }

private:
    struct Impl;
    Impl* p_;
};

#endif // RELAXED_ATOMIC_H
