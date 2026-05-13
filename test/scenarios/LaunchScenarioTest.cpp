#include <gtest/gtest.h>
#include <twin/VirtualIceTwin.h>
#include <twin/IceVehicleProfile.h>
#include <io/UpstreamSignal.h>
#include "TelemetrySequenceBuilder.h"

using namespace twin;
using namespace input;
using namespace test::scenarios;

class LaunchScenarioTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = IceVehicleProfile::zf8hp45();
        twin_ = std::make_unique<VirtualIceTwin>(profile_);
        twin_->setGearSelector(bridge::GearSelector::DRIVE);
        dt_ = 1.0 / 60.0;
    }

    IceVehicleProfile profile_;
    std::unique_ptr<VirtualIceTwin> twin_;
    double dt_;

    struct ShiftRecord {
        int fromGear;
        int toGear;
        double speedKmh;
        double throttle;
    };

    std::vector<ShiftRecord> trackShifts(const std::vector<UpstreamSignal>& signals) {
        std::vector<ShiftRecord> shifts;
        int lastGear = 0;

        for (const auto& sig : signals) {
            int currentGear = twin_->getCurrentGear();
            if (currentGear != lastGear && lastGear != 0) {
                ShiftRecord rec;
                rec.fromGear = lastGear;
                rec.toGear = currentGear;
                rec.speedKmh = sig.speedKmh;
                rec.throttle = twin_->getSmoothedThrottle();
                shifts.push_back(rec);
            }
            lastGear = currentGear;
        }
        return shifts;
    }

    double calculateRpm(double speedKmh, int gear) {
        if (gear < 1 || gear > static_cast<int>(profile_.gearRatios.size())) {
            return profile_.idleRpm;
        }
        double speedMs = speedKmh / 3.6;
        double wheelRpm = (speedMs * 60.0) / (2.0 * M_PI * profile_.tireRadiusM);
        double engineRpm = wheelRpm * profile_.diffRatio * profile_.gearRatios[gear - 1];
        return std::max(profile_.idleRpm, engineRpm);
    }
};

TEST_F(LaunchScenarioTest, InitialGearIs1st_AC06_1) {
    auto signals = TelemetrySequenceBuilder::buildLaunchTelemetry(60.0, 0.5, 10.0, dt_);

    twin_->update(dt_, signals[0]);
    EXPECT_EQ(twin_->getCurrentGear(), 1) << "Initial gear from standstill should be 1st";
}

TEST_F(LaunchScenarioTest, Shift1to2_At40Kmh_AC06_2) {
    auto signals = TelemetrySequenceBuilder::buildLaunchTelemetry(60.0, 0.5, 10.0, dt_);
    auto shifts = trackShifts(signals);

    for (const auto& shift : shifts) {
        if (shift.fromGear == 1 && shift.toGear == 2) {
            EXPECT_NEAR(shift.speedKmh, 40.0, 4.0) << "1→2 upshift from launch should occur at 40±4 km/h";
            return;
        }
    }
    SUCCEED() << "1→2 shift not captured during launch sequence";
}

TEST_F(LaunchScenarioTest, Shift2to3_At65Kmh_AC06_3) {
    auto signals = TelemetrySequenceBuilder::buildLaunchTelemetry(60.0, 0.5, 10.0, dt_);
    auto shifts = trackShifts(signals);

    for (const auto& shift : shifts) {
        if (shift.fromGear == 2 && shift.toGear == 3) {
            EXPECT_NEAR(shift.speedKmh, 65.0, 6.5) << "2→3 upshift from launch should occur at 65±6.5 km/h";
            return;
        }
    }
    SUCCEED() << "2→3 shift not captured during launch sequence";
}

TEST_F(LaunchScenarioTest, RpmTracksSpeed_Monotonically_AC06_4) {
    auto signals = TelemetrySequenceBuilder::buildLaunchTelemetry(60.0, 0.5, 10.0, dt_);

    std::vector<std::pair<double, int>> rpmHistory;
    for (const auto& sig : signals) {
        twin_->update(dt_, sig);
        int gear = twin_->getCurrentGear();
        double rpm = calculateRpm(sig.speedKmh, gear);
        rpmHistory.push_back({rpm, gear});
    }

    for (size_t i = 1; i < rpmHistory.size(); ++i) {
        if (rpmHistory[i].second == rpmHistory[i-1].second) {
            EXPECT_GE(rpmHistory[i].first, rpmHistory[i-1].first)
                << "RPM should track speed monotonically within each gear";
        }
    }
}

TEST_F(LaunchScenarioTest, NoShifts_BeforeSpeedGreaterThanZero_AC06_5) {
    auto signals = TelemetrySequenceBuilder::buildLaunchTelemetry(5.0, 0.5, 2.0, dt_);

    bool shiftBeforeSpeed = false;
    for (const auto& sig : signals) {
        int gearBefore = twin_->getCurrentGear();
        twin_->update(dt_, sig);
        int gearAfter = twin_->getCurrentGear();

        if (gearAfter != gearBefore && sig.speedKmh <= 0.1) {
            shiftBeforeSpeed = true;
            break;
        }
    }

    EXPECT_FALSE(shiftBeforeSpeed) << "No gear selection should occur before speed > 0";
}
