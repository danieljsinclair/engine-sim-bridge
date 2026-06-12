// RelaxedAtomic.h — std::atomic wrapper that defaults to memory_order_relaxed
// Use for diagnostic counters where slight staleness is acceptable and
// sequential consistency would add unnecessary memory barriers on ARM.

#ifndef RELAXED_ATOMIC_H
#define RELAXED_ATOMIC_H

#include <atomic>
#include <cstdint>

template <typename T>
class RelaxedAtomic {
public:
    RelaxedAtomic() = default;
    explicit RelaxedAtomic(T value) : value_(value) {}

    void store(T desired) noexcept {
        value_.store(desired, std::memory_order_relaxed);
    }

    T load() const noexcept {
        return value_.load(std::memory_order_relaxed);
    }

    T exchange(T desired) noexcept {
        return value_.exchange(desired, std::memory_order_relaxed);
    }

    bool compare_exchange_weak(T& expected, T desired) noexcept {
        return value_.compare_exchange_weak(expected, desired,
                                            std::memory_order_relaxed);
    }

    bool compare_exchange_strong(T& expected, T desired) noexcept {
        return value_.compare_exchange_strong(expected, desired,
                                              std::memory_order_relaxed);
    }

    T fetch_add(T arg) noexcept {
        return value_.fetch_add(arg, std::memory_order_relaxed);
    }

    T fetch_sub(T arg) noexcept {
        return value_.fetch_sub(arg, std::memory_order_relaxed);
    }

    operator T() const noexcept { return load(); }
    RelaxedAtomic& operator=(T desired) { store(desired); return *this; }

private:
    std::atomic<T> value_{};
};

using RelaxedInt   = RelaxedAtomic<int>;
using RelaxedULong = RelaxedAtomic<unsigned long>;
using RelaxedFloat = RelaxedAtomic<float>;
using RelaxedBool  = RelaxedAtomic<bool>;

#endif // RELAXED_ATOMIC_H
