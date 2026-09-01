// DeterministicStrategy.cpp - Fixed-timestep headless audio strategy
//
// Removes the audio-callback physics clock that makes live runs
// nondeterministic. See header for the design rationale.

#include "strategy/DeterministicStrategy.h"
#include "simulator/ISimulator.h"
#include "simulator/EngineSimTypes.h"

DeterministicStrategy::DeterministicStrategy(ILogging* logger,
                                             telemetry::ITelemetryWriter* telemetry)
    : logger_(logger)
    , telemetry_(telemetry) {
}

const char* DeterministicStrategy::getName() const {
    return "DeterministicStrategy";
}

bool DeterministicStrategy::isEnabled() const {
    return true;
}

bool DeterministicStrategy::isPlaying() const {
    return isPlaying_.load();
}

bool DeterministicStrategy::render(AudioBufferView& buffer) {
    // Silence only — a deterministic run has no audio output, and this method
    // must never advance or consume the simulator (that is the sync-pull
    // cadence race this strategy exists to remove).
    if (auto* dst = static_cast<float*>(buffer.samples)) {
        EngineSimAudio::fillSilence(dst, buffer.frameCount);
    }
    return true;
}

bool DeterministicStrategy::AddFrames(float*, int) {
    return true;
}

bool DeterministicStrategy::initialize(const AudioBufferConfig&, int) {
    return true;
}

void DeterministicStrategy::prepareBuffer() {
    // Intentionally empty: there is no audio pipeline to prepare in
    // deterministic mode.
}

bool DeterministicStrategy::startPlayback(ISimulator* simulator) {
    simulator_.store(simulator);
    isPlaying_.store(true);
    return true;
}

void DeterministicStrategy::stopPlayback(ISimulator*) {
    isPlaying_.store(false);
}

void DeterministicStrategy::swapSimulator(ISimulator* newSimulator) {
    simulator_.store(newSimulator);
}

void DeterministicStrategy::resetBufferAfterWarmup() {
    // Intentionally empty: no warmup buffer exists without an audio pipeline.
}

bool DeterministicStrategy::shouldDrainDuringWarmup() const {
    return false;
}

void DeterministicStrategy::fillBufferFromEngine(ISimulator*, int) {
    // Intentionally empty: rendering never consumes simulator state (the
    // silence-only render() above is the whole output path).
}

std::string DeterministicStrategy::getModeString() const {
    return "DETERMINISTIC";
}

void DeterministicStrategy::reset() {
    // Intentionally empty: this strategy holds no buffer state to reset.
}

void DeterministicStrategy::updateSimulation(ISimulator* simulator, double deltaTimeMs) {
    // THE contract: advance the simulation by exactly deltaTimeMs/1000 on the
    // calling (loop) thread. One call, one update, exact dt — no retries, no
    // sample-rate fractions, no clock reads. Same forwarding contract as
    // ThreadedStrategy::updateSimulation, but synchronous on the caller.
    if (simulator != nullptr) {
        simulator->update(deltaTimeMs / 1000.0);
    }
}
