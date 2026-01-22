#include <cmath>

// Stub implementation of engine-sim-bridge for macOS x86_64
// This provides minimal functionality to test the bridge interface

extern "C" {

// Simple stub that returns basic values
double engine_sim_get_rpm() {
    return 1000.0;
}

double engine_sim_get_throttle() {
    return 0.5;
}

void engine_sim_set_throttle(double throttle) {
    // No-op in stub
}

void engine_sim_initialize() {
    // No-op in stub
}

void engine_sim_shutdown() {
    // No-op in stub
}

void engine_sim_update(double dt) {
    // No-op in stub
}

int engine_sim_is_running() {
    return 1;
}

} // extern "C"
