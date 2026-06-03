// Phase 0 Spike: Dyno RPM tracking with real engine
#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>

#include "engine-sim/include/piston_engine_simulator.h"
#include "engine-sim/include/engine.h"
#include "engine-sim/include/vehicle.h"
#include "engine-sim/include/transmission.h"
#include "engine-sim/include/simulator.h"

TEST(Phase0Spikes, DynoTracking) {
    const std::string outDir = "/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/build-test/spikes/dyno_tracking/";
    const std::string csvPath = outDir + "tracking_data.csv";
    system(("mkdir -p " + outDir).c_str());

    // Open CSV for writing
    std::ofstream csv(csvPath);
    csv << "timeSec,targetRpm,actualRpm,errRpm\n";

    // Engine I6 (COPY FROM ClutchParameterSweepSpike.cpp lines 27-44)
    Engine::Parameters ep{};
    ep.cylinderBanks = 1;
    ep.cylinderCount = 6;
    ep.crankshaftCount = 1;
    ep.exhaustSystemCount = 1;
    ep.intakeCount = 1;
    ep.name = "DynoTestI6";
    ep.starterTorque = 90 * units::ft_lb;
    ep.starterSpeed = 200;
    ep.redline = 6500;
    ep.dynoMinSpeed = 1000;
    ep.dynoMaxSpeed = 6500;
    ep.dynoHoldStep = 100;
    Throttle* thr = new Throttle();
    ep.throttle = thr;

    Engine engine;
    engine.initialize(ep);

    // Vehicle (COPY FROM ClutchParameterSweepSpike.cpp lines 48-55)
    Vehicle::Parameters vp{};
    vp.mass = 1500 * units::kg;
    vp.diffRatio = 3.73;
    vp.tireRadius = units::distance(0.33, units::m);
    vp.dragCoefficient = 0.30;
    vp.crossSectionArea = 2.2;
    vp.rollingResistance = 0.015;
    Vehicle vehicle;
    vehicle.initialize(vp);

    // Transmission (6-speed, gear 3)
    Transmission trans;
    Transmission::Parameters tp{};
    double ratios[6] = {3.50, 2.10, 1.45, 1.10, 0.90, 0.75};
    tp.GearCount = 6;
    tp.GearRatios = ratios;
    tp.MaxClutchTorque = 8000;
    trans.initialize(tp);
    trans.changeGear(3);

    // PistonEngineSimulator with dyno ENABLED
    PistonEngineSimulator sim;
    Simulator::Parameters sp;
    sp.systemType = Simulator::SystemType::NsvOptimized;
    sim.initialize(sp);
    sim.setSimulationFrequency(10000);
    sim.setFluidSimulationSteps(8);
    sim.loadSimulation(&engine, &vehicle, &trans);
    sim.m_dyno.m_enabled = true;
    sim.m_dyno.m_hold = true;
    sim.m_dyno.m_ks = 10.0;
    sim.m_dyno.m_kd = 1.0;

    engine.getIgnitionModule()->m_enabled = true;
    engine.setSpeedControl(0.5);

    const double dt = 1.0 / 10000.0;
    const double radPerRpm = M_PI / 30.0;

    // Scenario 1: Steady-state tracking at 3000 RPM
    const double steadyTarget = 3000.0;
    sim.m_dyno.m_rotationSpeed = steadyTarget * radPerRpm;

    std::vector<double> settledErrors;
    const int steadySteps = static_cast<int>(5.0 / dt);
    const int settleSteps = static_cast<int>(2.0 / dt);
    const int sampleInterval = 100;

    for (int step = 0; step < steadySteps; ++step) {
        sim.startFrame(dt);
        sim.simulateStep();

        double v_theta = engine.getOutputCrankshaft()->m_body.v_theta;
        double actualRpm = std::abs(v_theta) * (60.0 / (2.0 * M_PI));
        double err = actualRpm - steadyTarget;

        if (step >= settleSteps) {
            settledErrors.push_back(std::abs(err));
        }

        if (step % sampleInterval == 0) {
            csv << (step * dt) << "," << steadyTarget << ","
                << actualRpm << "," << err << "\n";
        }
    }

    double meanErr = std::accumulate(settledErrors.begin(), settledErrors.end(), 0.0) / settledErrors.size();
    auto [minIt, maxIt] = std::minmax_element(settledErrors.begin(), settledErrors.end());
    double maxErr = *maxIt;

    printf("Steady-state (3000 RPM): mean_err=%.1f RPM, max_err=%.1f RPM\n", meanErr, maxErr);
    EXPECT_LT(meanErr, 150.0) << "Mean tracking error exceeds 150 RPM threshold";

    // Scenario 2: Telemetry jitter at 10Hz ramp
    std::vector<double> rampErrors;
    const int rampSteps = static_cast<int>(8.0 / dt);
    const int targetUpdateInterval = 1000;
    const int rampSampleInterval = 100;

    const double startRpm = 1000.0;
    const double endRpm = 6000.0;
    const double rampDuration = 8.0;

    for (int step = 0; step < rampSteps; ++step) {
        double t = step * dt;

        if (step % targetUpdateInterval == 0) {
            double progress = std::min(1.0, t / rampDuration);
            double targetRpm = startRpm + (endRpm - startRpm) * progress;
            sim.m_dyno.m_rotationSpeed = targetRpm * radPerRpm;
        }

        sim.startFrame(dt);
        sim.simulateStep();

        if (step % rampSampleInterval == 0) {
            double v_theta = engine.getOutputCrankshaft()->m_body.v_theta;
            double actualRpm = std::abs(v_theta) * (60.0 / (2.0 * M_PI));
            double targetRpm = sim.m_dyno.m_rotationSpeed / radPerRpm;
            double err = actualRpm - targetRpm;
            rampErrors.push_back(err);

            csv << (t + 5.0) << "," << targetRpm << ","
                << actualRpm << "," << err << "\n";
        }
    }

    double rampMean = std::accumulate(rampErrors.begin(), rampErrors.end(), 0.0) / rampErrors.size();
    double rampSqSum = std::inner_product(rampErrors.begin(), rampErrors.end(),
                                          rampErrors.begin(), 0.0);
    double rampStddev = std::sqrt(rampSqSum / rampErrors.size() - rampMean * rampMean);

    printf("Ramp (1000-6000 RPM): mean_err=%.1f RPM, stddev=%.1f RPM\n",
           std::abs(rampMean), rampStddev);

    EXPECT_LT(std::abs(rampMean), 300.0) << "Mean ramp error exceeds 300 RPM threshold";
    EXPECT_LT(rampStddev, 100.0) << "Ramp error stddev exceeds 100 RPM threshold";

    csv.close();
    printf("CSV: %s\n", csvPath.c_str());
}
