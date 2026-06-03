// Phase 0 Spike 3: Telemetry jitter test
// BUILD: cmake -B build-test && cmake --build build-test --target jitter_test_spike

#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <string>

#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"

inline double expectedRpmFromSpeed(double speedKmh, double g, double d, double tireRadiusM){
    double v=speedKmh*(1000.0/3600.0);
    double omega=v/tireRadiusM*g*d;
    return omega*(60.0/(2.0*M_PI));
}

TEST(Phase0Spikes, TelemetryJitter){
    const std::string outDir="build/spikes/jitter_test/";
    system(("mkdir -p "+outDir).c_str());

    auto sineSim=std::make_unique<SineSimulator>();
    Simulator::Parameters sp;
    sp.systemType=Simulator::SystemType::NsvOptimized;
    sineSim->initialize(sp);
    sineSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
    sineSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
    sineSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);
    sineSim->loadSimulation(nullptr,nullptr,nullptr);

    auto bridge=std::make_unique<BridgeSimulator>(std::move(sineSim));
    ISimulatorConfig cfg;
    cfg.sampleRate=EngineSimDefaults::SAMPLE_RATE;
    cfg.simulationFrequency=EngineSimDefaults::SIMULATION_FREQUENCY;
    ASSERT_TRUE(bridge->create(cfg,nullptr,nullptr));

    Simulator* s=bridge->getInternalSimulator();
    ASSERT_NE(s,nullptr);
    // Use realistic gains (default constructor: ks=10, kd=1, maxTorque=10000 ft-lb ≈ 13556 Nm)
    s->m_dyno.m_enabled=true; s->m_dyno.m_hold=true;
    s->m_dyno.m_ks=10.0; s->m_dyno.m_kd=1.0;

    bridge->setIgnition(true);
    bridge->setStarterMotor(false);

    const double dt=1.0/EngineSimDefaults::SIMULATION_FREQUENCY;
    const double T=8.0;
    const int totalSteps=static_cast<int>(T/dt);
    const int stride=static_cast<int>((1.0/10.0)/dt);

    struct Samp{ double t,tgt,act; };
    std::vector<Samp> samples;
    bool first=true;

    for(int step=0; step<totalSteps; ++step){
        double t=step*dt;
        double spd=std::min(80.0, t*10.0);
        double tgt=expectedRpmFromSpeed(spd,3.5,3.73,0.33);
        if(step%stride==0 || first){ s->m_dyno.m_rotationSpeed=tgt*(M_PI/30.0); first=false; }
        int32_t written=0;
        float buf[EngineSimDefaults::FRAMES_PER_UPDATE*2];
        ASSERT_TRUE(bridge->renderOnDemand(buf, EngineSimDefaults::FRAMES_PER_UPDATE, &written));
        double actual = s->filteredEngineSpeed();
        if(step%1000==0){ samples.push_back({t,tgt,actual}); }
    }

    bridge->destroy();

    std::ofstream csv(outDir+"jitter_data.csv");
    csv<<"timeSec,targetRpm,actualRpm,errRpm,absPctErr\n";
    double sumAbs=0.0,maxAbs=0.0;
    for(const auto& s2: samples){
        double err=s2.act-s2.tgt;
        double pct=s2.tgt>1.0?std::abs(err)/s2.tgt*100.0:0.0;
        csv<<s2.t<<','<<s2.tgt<<','<<s2.act<<','<<err<<','<<pct<<"\n";
        double e=std::abs(err); sumAbs+=e; if(e>maxAbs)maxAbs=e;
    }
    printf("mean err=%.1f RPM, max=%.1f RPM\n", sumAbs/samples.size(), maxAbs);

    ASSERT_GT(samples.size(),10u);
    SUCCEED();
}
