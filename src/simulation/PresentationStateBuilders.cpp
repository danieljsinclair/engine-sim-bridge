// PresentationStateBuilders.cpp - Domain-specific EngineState construction helpers
// Each function builds one domain sub-struct from its authoritative source

#include "simulation/PresentationStateBuilders.h"
#include "simulation/SimulationLoop.h"
#include "simulator/ISimulator.h"
#include "strategy/IAudioBuffer.h"

namespace presentation::builders {

        EngineState::Engine buildEngineState(const EngineSimStats& stats, const CrankingController::State& cranking) {
            EngineState::Engine engine;
            // The tach reads the FILTERED sensor (same as the gate CSV's rpm
            // column); the raw crank speed rides along as rpmRaw so evidence
            // is never destroyed by the measurement model.
            engine.rpm = stats.filteredRPM;
            engine.rpmRaw = stats.currentRPM;
            engine.load = stats.currentLoad;
            engine.exhaustFlow = stats.exhaustFlow;
            engine.synthOutputRms = stats.synthOutputRms;
            engine.engineTorqueNm = stats.engineTorqueNm;
            engine.drivetrainTorqueNm = stats.drivetrainTorqueNm;
            engine.starterEngaged = cranking.starterEngaged;
            engine.phase = cranking.phase;
            return engine;
        }

        EngineState::Drivetrain buildDrivetrainState(const EngineSimStats& stats,
                                                      const input::EngineInput& input) {
            EngineState::Drivetrain drivetrain;
            drivetrain.speedMph = stats.speedMph();
            drivetrain.vehicleSpeedKmh = stats.vehicleSpeedKmh;
            drivetrain.gear = stats.gear;
            drivetrain.dynoTorque = stats.dynoTorque;
            drivetrain.dynoTargetRPM = stats.dynoTargetRPM;
            drivetrain.replayTimestampS = input.replayTimestampS;
            drivetrain.clutchPressure = input.clutchPressure;
            drivetrain.roadImpliedRpm = input.roadImpliedRpm;
            drivetrain.creepReliefFired = input.creepReliefFired;
            drivetrain.couplingIsTorqueConverter = input.couplingIsTorqueConverter;
            return drivetrain;
        }

        EngineState::Controls buildControlState(
                const input::EngineInput& input, const CrankingController::State& cranking) {
            EngineState::Controls controls;
            controls.throttle = cranking.startingThrottle;
            controls.ignition = input.ignition;
            controls.brakeLight = input.brakeLight;
            controls.gearSelector = input.gearSelector;
            controls.gearAutoMode = input.gearAutoMode;
            // Surface the commanded road-speed target so it is visible even in
            // neutral (where the vehicle speed readout reflects physics only).
            controls.commandedSpeedKmh = input.roadSpeedKmh;
            return controls;
        }

        EngineState::Audio buildAudioState(const telemetry::AudioTimingTelemetry& timing,
                                        const telemetry::ITelemetryReader* telemetryReader,
                                        const IAudioBuffer& audioBuffer, const SimulationConfig& config,
                                        double currentTime, const ISimulator& simulator) {
            EngineState::Audio audio;
            audio.timestamp = currentTime;
            audio.simulationFrequency = simulator.getSimulationFrequency();
            audio.underrunCount = telemetryReader ? telemetryReader->getAudioDiagnostics().underrunCount : 0;
            audio.audioMode = audioBuffer.getModeString();
            audio.renderMs = timing.renderMs;
            audio.headroomMs = timing.headroomMs;
            audio.budgetPct = timing.budgetPct;
            audio.framesRequested = timing.framesRequested;
            audio.framesRendered = timing.framesRendered;
            audio.callbackRateHz = timing.callbackRateHz;
            audio.generatingRateFps = timing.generatingRateFps;
            audio.trendPct = timing.trendPct;
            audio.sampleRate = config.sampleRate();
            return audio;
        }

} // namespace presentation::builders
