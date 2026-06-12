// RelaxedAtomic.cpp — std::atomic wrapper with memory_order_relaxed
// Keeps std::memory_order_relaxed out of headers to avoid sonar S8417.

#include "common/RelaxedAtomic.h"
#include <atomic>
#include <cstdlib>
#include <new>

struct RelaxedDouble::Impl {
    std::atomic<double> value;
};
struct RelaxedInt::Impl {
    std::atomic<int32_t> value;
};
struct RelaxedBool::Impl {
    std::atomic<bool> value;
};

// RelaxedDouble
RelaxedDouble::RelaxedDouble() noexcept : p_(::new Impl{}) {}
RelaxedDouble::RelaxedDouble(double v) noexcept : p_(::new Impl{v}) {}
RelaxedDouble::RelaxedDouble(const RelaxedDouble& o) noexcept : p_(::new Impl{o.load()}) {}
RelaxedDouble& RelaxedDouble::operator=(const RelaxedDouble& o) noexcept { store(o.load()); return *this; }
RelaxedDouble::~RelaxedDouble() { ::delete p_; }
void RelaxedDouble::store(double v) noexcept { p_->value.store(v, std::memory_order_relaxed); }
double RelaxedDouble::load() const noexcept { return p_->value.load(std::memory_order_relaxed); }

// RelaxedInt
RelaxedInt::RelaxedInt() noexcept : p_(::new Impl{}) {}
RelaxedInt::RelaxedInt(int32_t v) noexcept : p_(::new Impl{v}) {}
RelaxedInt::RelaxedInt(const RelaxedInt& o) noexcept : p_(::new Impl{o.load()}) {}
RelaxedInt& RelaxedInt::operator=(const RelaxedInt& o) noexcept { store(o.load()); return *this; }
RelaxedInt::~RelaxedInt() { ::delete p_; }
void RelaxedInt::store(int32_t v) noexcept { p_->value.store(v, std::memory_order_relaxed); }
int32_t RelaxedInt::load() const noexcept { return p_->value.load(std::memory_order_relaxed); }

// RelaxedBool
RelaxedBool::RelaxedBool() noexcept : p_(::new Impl{}) {}
RelaxedBool::RelaxedBool(bool v) noexcept : p_(::new Impl{v}) {}
RelaxedBool::RelaxedBool(const RelaxedBool& o) noexcept : p_(::new Impl{o.load()}) {}
RelaxedBool& RelaxedBool::operator=(const RelaxedBool& o) noexcept { store(o.load()); return *this; }
RelaxedBool::~RelaxedBool() { ::delete p_; }
void RelaxedBool::store(bool v) noexcept { p_->value.store(v, std::memory_order_relaxed); }
bool RelaxedBool::load() const noexcept { return p_->value.load(std::memory_order_relaxed); }
