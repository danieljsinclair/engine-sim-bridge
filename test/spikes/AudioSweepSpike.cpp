// Phase 0 Spike 1: Dyno-mode audio sweep
// BUILD: cmake -B build-test && cmake --build build-test --target audio_sweep_spike

#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <string>

#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"

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

TEST(Phase0Spikes, AudioSweep_RecordsWav) {
    auto sineSim=std::make_unique<SineSimulator>();
    Simulator::Parameters p; p.systemType=Simulator::SystemType::NsvOptimized;
    sineSim->initialize(p);
    sineSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
    sineSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
    sineSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);
    sineSim->loadSimulation(new SineEngine(), new SineVehicle(), new SineTransmission());

    auto bridge=std::make_unique<BridgeSimulator>(std::move(sineSim));
    ISimulatorConfig cfg;
    cfg.sampleRate=EngineSimDefaults::SAMPLE_RATE;
    cfg.simulationFrequency=EngineSimDefaults::SIMULATION_FREQUENCY;
    ASSERT_TRUE(bridge->create(cfg,nullptr,nullptr));

    Simulator* s=bridge->getInternalSimulator();
    ASSERT_NE(s,nullptr);
    // NOTE: Use realistic dyno gains — default ks=10.0, kd=1.0
    s->m_dyno.m_enabled=true; s->m_dyno.m_hold=true;
    s->m_dyno.m_ks=10.0;  // spring gain (default)
    s->m_dyno.m_kd=1.0;   // damping gain (default)
    s->m_dyno.m_maxTorque=5000.0;

    bridge->setIgnition(true);
    bridge->setStarterMotor(false);
    bridge->setThrottle(0.5);

    const double dt=1.0/EngineSimDefaults::SIMULATION_FREQUENCY;
    const int totalSteps=static_cast<int>(10.0/dt);
    std::vector<int16_t> caps;
    const int fpb = EngineSimDefaults::FRAMES_PER_UPDATE;

    for(int step=0; step<totalSteps; ++step){
        double phase=(double)step/totalSteps;
        double tgt=1200.0 + phase*(6500.0-1200.0);
        s->m_dyno.m_rotationSpeed=tgt*(M_PI/30.0);
        std::vector<float> fbuf(fpb*2);
        int32_t written=0;
        ASSERT_TRUE(bridge->renderOnDemand(fbuf.data(), fpb, &written)) << "renderOnDemand failed";
        for(int i=0;i<written*2;++i){
            float v = fbuf[i]; if(v>1.0f) v=1.0f; if(v<-1.0f) v=-1.0f;
            caps.push_back(static_cast<int16_t>(v*32767.0f));
        }
    }

    bridge->destroy();
    ASSERT_GT(caps.size(),0u);

    std::string outDir="/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/build-test/spikes/audio_sweep/";
    system(("mkdir -p "+outDir).c_str());
    writeWav(outDir+"audio_sweep.wav",caps,44100);

    double sum=0.0;
    for(int16_t x: caps){ double n=x/32768.0; sum+=n*n; }
    EXPECT_GT(std::sqrt(sum/caps.size()),0.01);
}
