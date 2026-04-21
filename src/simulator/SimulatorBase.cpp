#include "simulator/SimulatorBase.h"

void SimulatorBase::update(double deltaTime) {
    if (!isReady()) return;
    advanceFixedSteps(getSimulator(), simulationFrequency_, deltaTime, true);
    pushTelemetry(getStats());
}

bool SimulatorBase::renderOnDemand(float* buffer, int32_t frames, int32_t* written) {
    if (!isReady() || !buffer || frames <= 0) {
        if (written) *written = 0;
        return false;
    }

    Simulator* sim = getSimulator();
    const double dt = static_cast<double>(frames) / sampleRate_;
    advanceFixedSteps(sim, simulationFrequency_, dt, false);
    sim->synthesizer().renderAudioOnDemand();

    int16_t* conversionBuffer = ensureAudioConversionBufferSize(frames);
    int samplesRead = sim->readAudioOutput(frames, conversionBuffer);
    EngineSimAudio::convertInt16ToStereoFloat(conversionBuffer, buffer, samplesRead);

    if (samplesRead < frames) {
        EngineSimAudio::fillSilence(buffer + samplesRead * 2, frames - samplesRead);
    }
    if (written) *written = samplesRead;
    return true;
}

bool SimulatorBase::readAudioBuffer(float* buffer, int32_t frames, int32_t* read) {
    if (!isReady() || !buffer || frames <= 0) {
        if (read) *read = 0;
        return false;
    }

    Simulator* sim = getSimulator();
    int16_t* conversionBuffer = ensureAudioConversionBufferSize(frames);
    int samplesRead = sim->readAudioOutput(frames, conversionBuffer);
    EngineSimAudio::convertInt16ToStereoFloat(conversionBuffer, buffer, samplesRead);

    if (samplesRead < frames) {
        EngineSimAudio::fillSilence(buffer + samplesRead * 2, frames - samplesRead);
    }
    if (read) *read = samplesRead;
    return true;
}

bool SimulatorBase::start() {
    if (!isReady()) return false;
    Simulator* sim = getSimulator();
    drainSynthesizerBuffer(sim);
    sim->startAudioRenderingThread();
    return true;
}

void SimulatorBase::stop() {
    if (!isReady()) return;
    getSimulator()->endAudioRenderingThread();
}

EngineSimStats SimulatorBase::getStats() const {
    EngineSimStats stats = {};
    const Simulator* sim = getSimulator();
    if (sim->getEngine()) {
        stats.currentRPM = sim->getEngine()->getSpeed() * 60.0 / (2.0 * M_PI);
        stats.exhaustFlow = sim->getTotalExhaustFlow();
        stats.processingTimeMs = sim->getAverageProcessingTime() * 1000.0;
    }
    return stats;
}

void SimulatorBase::setThrottle(double position) {
    if (!isReady()) return;
    if (position < 0.0) position = 0.0;
    if (position > 1.0) position = 1.0;
    Simulator* sim = getSimulator();
    if (sim->getEngine()) {
        sim->getEngine()->setSpeedControl(position);
    }
}

void SimulatorBase::setIgnition(bool on) {
    if (!isReady()) return;
    Simulator* sim = getSimulator();
    if (sim->getEngine()) {
        sim->getEngine()->getIgnitionModule()->m_enabled = on;
    }
}

void SimulatorBase::setStarterMotor(bool on) {
    if (!isReady()) return;
    getSimulator()->m_starterMotor.m_enabled = on;
}
