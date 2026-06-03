// Phase 0 Spike 3: Real Ferrari F136 V8 engine audio with dyno RPM sweep
// Build: cmake -B build-test && cmake --build build-test --target real_engine_audio_spike

#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>

#include "engine-sim/scripting/include/compiler.h"
#include "engine-sim/include/piston_engine_simulator.h"
#include "engine-sim/include/engine.h"
#include "engine-sim/include/vehicle.h"
#include "engine-sim/include/transmission.h"
#include "engine-sim/include/simulator.h"
#include "engine-sim/include/exhaust_system.h"
#include "engine-sim/include/impulse_response.h"
#include "engine-sim/dependencies/dr_libs/dr_wav.h"
#include "simulator/EngineSimTypes.h"

#pragma pack(push,1)
struct WavHeader {
    char riff[4]={'R','I','F','F'};
    uint32_t chunkSize;
    char wave[4]={'W','A','V','E'};
    char fmt[4]={'f','m','t',' '};
    uint32_t subchunk1Size=16;
    uint16_t audioFormat=1;
    uint16_t numChannels=2;
    uint32_t sampleRate=44100;
    uint32_t byteRate=44100*2*2;
    uint16_t blockAlign=4;
    uint16_t bitsPerSample=16;
    char data[4]={'d','a','t','a'};
    uint32_t subchunk2Size;
};
#pragma pack(pop)

void writeWav(const std::string& path, const std::vector<int16_t>& samples, int sr) {
    WavHeader h{}; h.sampleRate=sr; h.byteRate=sr*2*2; h.blockAlign=4;
    h.subchunk2Size=static_cast<uint32_t>(samples.size()*2);
    h.chunkSize=36+h.subchunk2Size;
    std::ofstream f(path,std::ios::binary);
    f.write(reinterpret_cast<const char*>(&h),sizeof(h));
    f.write(reinterpret_cast<const char*>(samples.data()),samples.size()*2);
}

void loadImpulseResponses(Simulator* sim, Engine* engine, const std::string& engineSimDir) {
    const std::string soundLibDir = engineSimDir + "/es/sound-library/";
    Synthesizer& synth = sim->synthesizer();

    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        ImpulseResponse* ir = engine->getExhaustSystem(i)->getImpulseResponse();
        if (!ir) continue;

        std::string filename = ir->getFilename();
        std::string fullPath = soundLibDir + filename;

        printf("  Loading impulse response [%d]: %s\n", i, fullPath.c_str());
        fflush(stdout);

        unsigned int wavChannels, wavSampleRate;
        drwav_uint64 totalFrames;
        int16_t* wavData = drwav_open_file_and_read_pcm_frames_s16(
            fullPath.c_str(), &wavChannels, &wavSampleRate, &totalFrames, nullptr);

        if (!wavData) {
            printf("  WARNING: Failed to load: %s\n", fullPath.c_str());
            fflush(stdout);
            continue;
        }

        std::vector<int16_t> monoData(totalFrames);
        if (wavChannels == 1) {
            memcpy(monoData.data(), wavData, totalFrames * sizeof(int16_t));
        } else {
            for (drwav_uint64 f = 0; f < totalFrames; ++f) {
                monoData[f] = wavData[f * wavChannels];
            }
        }

        synth.initializeImpulseResponse(
            monoData.data(),
            static_cast<unsigned int>(monoData.size()),
            static_cast<float>(ir->getVolume()),
            i
        );

        printf("  Loaded: %u samples, %u Hz, %u ch, vol=%.4f\n",
               static_cast<unsigned>(totalFrames), wavSampleRate, wavChannels,
               static_cast<float>(ir->getVolume()));
        fflush(stdout);

        drwav_free(wavData, nullptr);
    }
}

TEST(Phase0Spikes, RealEngineAudio_FerrariF136) {
    const std::string engineSimDir = "/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/engine-sim";
    const std::string scriptPath = "../es/spike_ferrari_f136.mr";
    const std::string outDir = "/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/build-test/spikes/real_engine_audio/";
    const std::string wavPath = outDir + "ferrari_f136_sweep.wav";
    system(("mkdir -p " + outDir).c_str());

    printf("CWD: %s\n", std::filesystem::current_path().c_str());
    fflush(stdout);

    es_script::Compiler compiler;
    compiler.initialize();

    printf("Compiling Ferrari F136 script...\n");
    fflush(stdout);

    bool ok = compiler.compile(scriptPath.c_str());
    printf("Compile result: %d\n", ok);
    fflush(stdout);

    if (!ok) {
        compiler.destroy();
        ASSERT_TRUE(false) << "Failed to compile script: " << scriptPath;
    }

    es_script::Compiler::Output output = compiler.execute();
    Engine* engine = output.engine;

    if (!engine) {
        compiler.destroy();
        ASSERT_NE(engine, nullptr) << "Script did not create an engine";
    }
    printf("Engine: %s\n", engine->getName().c_str());
    fflush(stdout);

    Vehicle* vehicle = output.vehicle;
    Transmission* transmission = output.transmission;

    compiler.destroy();

    // 2. Create PistonEngineSimulator
    auto pistonSim = std::make_unique<PistonEngineSimulator>();
    Simulator::Parameters sp;
    sp.systemType = Simulator::SystemType::NsvOptimized;
    pistonSim->initialize(sp);
    pistonSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
    pistonSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
    pistonSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);

    pistonSim->loadSimulation(engine, vehicle, transmission);

    Simulator* sim = pistonSim.get();

    // 3. Load impulse responses (required for ConvolutionFilter)
    printf("Loading impulse responses...\n");
    fflush(stdout);
    loadImpulseResponses(sim, engine, engineSimDir);
    printf("Impulse responses loaded (%d exhaust systems)\n",
           engine->getExhaustSystemCount());
    fflush(stdout);

    // 4. Enable dyno mode
    sim->m_dyno.m_enabled = true;
    sim->m_dyno.m_hold = true;
    sim->m_dyno.m_ks = 10.0;
    sim->m_dyno.m_kd = 1.0;
    sim->m_dyno.m_maxTorque = 5000.0;

    engine->getIgnitionModule()->m_enabled = true;
    sim->m_starterMotor.m_enabled = true;

    // 5. Simulation: physics at 10kHz, audio at 44.1kHz
    //    The synthesizer ring buffer builds up as physics produces input samples.
    //    We batch physics steps, then drain all available audio.
    const double simFreq = EngineSimDefaults::SIMULATION_FREQUENCY;
    const double sampleRate = EngineSimDefaults::SAMPLE_RATE;
    const double dt = 1.0 / simFreq;
    const double radPerRpm = M_PI / 30.0;

    // Run 15 seconds of simulation time
    const double totalDuration = 15.0;
    const int totalPhysicsSteps = static_cast<int>(totalDuration * simFreq);
    const double minRpm = 1000.0;
    const double maxRpm = 8500.0;

    // Batch physics into chunks matching the audio update rate (~60Hz)
    // simFreq / 60 = ~167 physics steps per audio batch
    const int stepsPerBatch = 167;
    const int totalBatches = totalPhysicsSteps / stepsPerBatch;
    const int maxReadPerBatch = 1024;

    std::vector<int16_t> audioBuffer;

    printf("Starting Ferrari F136 dyno sweep (%.0fs, %.0f-%.0f RPM, %d batches)\n",
           totalDuration, minRpm, maxRpm, totalBatches);
    fflush(stdout);

    int physicsStep = 0;
    for (int batch = 0; batch < totalBatches; ++batch) {
        // RPM ramp for this batch
        double progress = static_cast<double>(batch) / totalBatches;
        double targetRpm = minRpm + (maxRpm - minRpm) * progress;

        engine->setSpeedControl(1.0);
        sim->m_dyno.m_rotationSpeed = targetRpm * radPerRpm;

        // Run physics batch
        for (int s = 0; s < stepsPerBatch; ++s) {
            sim->startFrame(dt);
            sim->simulateStep();
            sim->endFrame();
            ++physicsStep;
        }

        // Render and drain synthesizer
        sim->synthesizer().renderAudioOnDemand();
        std::vector<int16_t> chunk(maxReadPerBatch);
        int read = sim->readAudioOutput(maxReadPerBatch, chunk.data());
        for (int i = 0; i < read; ++i) {
            audioBuffer.push_back(chunk[i]);
        }

        // Progress every ~1 second (6 batches)
        if (batch % 6 == 0) {
            double rpm = std::abs(engine->getOutputCrankshaft()->m_body.v_theta * 60.0 / (2.0 * M_PI));
            printf("  batch %d/%d  target=%.0f  actual=%.0f RPM  audio=%zu samples (%.2fs)\n",
                   batch, totalBatches, targetRpm, rpm, audioBuffer.size(),
                   static_cast<double>(audioBuffer.size()) / sampleRate);
            fflush(stdout);
        }
    }

    // Final drain
    for (int drain = 0; drain < 20; ++drain) {
        sim->synthesizer().renderAudioOnDemand();
        std::vector<int16_t> chunk(maxReadPerBatch);
        int read = sim->readAudioOutput(maxReadPerBatch, chunk.data());
        for (int i = 0; i < read; ++i) {
            audioBuffer.push_back(chunk[i]);
        }
        if (read == 0) break;
    }

    // 6. Compute stats before destroying simulator
    ASSERT_GT(audioBuffer.size(), 0u) << "No audio captured";

    double sum = 0.0;
    for (int16_t x : audioBuffer) {
        double n = x / 32768.0;
        sum += n * n;
    }
    double rms = std::sqrt(sum / audioBuffer.size());
    size_t wavBytes = audioBuffer.size() * 2 + sizeof(WavHeader);
    double durationSec = static_cast<double>(audioBuffer.size()) / sampleRate;

    // 7. Destroy simulator before assertions to avoid destructor assertion
    pistonSim->destroy();
    pistonSim.reset();

    // 8. Write WAV (after simulator destroyed, using captured buffer)
    writeWav(wavPath, audioBuffer, sampleRate);

    printf("Ferrari F136 V8 Audio:\n");
    printf("  Samples: %zu (%.2f seconds)\n", audioBuffer.size(), durationSec);
    printf("  WAV file: %s\n", wavPath.c_str());
    printf("  WAV size: %zu bytes (%.2f MB)\n", wavBytes, wavBytes / (1024.0 * 1024.0));
    printf("  RMS level: %.4f\n", rms);

    EXPECT_GT(wavBytes, 1024u * 1024u) << "WAV file too small (< 1MB)";
    EXPECT_GT(rms, 0.01) << "Audio RMS too low, engine may not be running";
}
