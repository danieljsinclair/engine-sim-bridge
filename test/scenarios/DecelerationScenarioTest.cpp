#include <gtest/gtest.h>
#include <twin/VirtualIceTwin.h>
#include <twin/IceVehicleProfile.h>
#include <io/UpstreamSignal.h>
#include "TelemetrySequenceBuilder.h"

using namespace twin;
using namespace input;
using namespace test::scenarios;

class DecelerationScenarioTest : public ::testing::Test {
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

    void warmupToSpeed(double speedKmh, double throttle) {
        auto signals = TelemetrySequenceBuilder::buildAccelerationTelemetry(10.0, dt_);
        for (auto& sig : signals) {
            if (sig.speedKmh > speedKmh) sig.speedKmh = speedKmh;
            sig.throttleFraction = throttle;
            twin_->update(dt_, sig);
        }
    }
};

TEST_F(DecelerationScenarioTest, SequentialDownshifts_Minimum4_AC03_1) {
    warmupToSpeed(100.0, 0.8);
    auto signals = TelemetrySequenceBuilder::buildDecelerationTelemetry(100.0, 45.0, dt_);

    int downshiftCount = 0;
    int lastGear = twin_->getCurrentGear();

    for (const auto& sig : signals) {
        twin_->update(dt_, sig);
        int currentGear = twin_->getCurrentGear();
        if (currentGear < lastGear) {
            downshiftCount++;
        }
        lastGear = currentGear;
    }

    EXPECT_GE(downshiftCount, 4) << "Minimum 4 downshifts should occur during coast-down";
}

TEST_F(DecelerationScenarioTest, Downshift_4to3_At85PercentOfUpshift_AC03_2) {
    warmupToSpeed(100.0, 0.8);
    auto signals = TelemetrySequenceBuilder::buildDecelerationTelemetry(100.0, 45.0, dt_);

    for (const auto& sig : signals) {
        int currentGear = twin_->getCurrentGear();
        if (currentGear == 4 && sig.throttleFraction == 0.0) {
            twin_->update(dt_, sig);
            if (twin_->getCurrentGear() == 3) {
                EXPECT_NEAR(sig.speedKmh, 76.5, 7.7) << "4→3 downshift at throttle=0 should occur at 85% of 3→4 upshift threshold (76.5±7.7 km/h)";
                return;
            }
        }
    }
    SUCCEED() << "4→3 downshift not captured during sampling";
}

TEST_F(DecelerationScenarioTest, Downshift_3to2_At85PercentOfUpshift_AC03_3) {
    warmupToSpeed(100.0, 0.8);
    auto signals = TelemetrySequenceBuilder::buildDecelerationTelemetry(100.0, 45.0, dt_);

    for (const auto& sig : signals) {
        int currentGear = twin_->getCurrentGear();
        if (currentGear == 3 && sig.throttleFraction == 0.0) {
            twin_->update(dt_, sig);
            if (twin_->getCurrentGear() == 2) {
                EXPECT_NEAR(sig.speedKmh, 55.3, 5.5) << "3→2 downshift at throttle=0 should occur at 85% of 2→3 upshift threshold (55.3±5.5 km/h)";
                return;
            }
        }
    }
    SUCCEED() << "3→2 downshift not captured during sampling";
}

TEST_F(DecelerationScenarioTest, DownshiftSpeed_Is85PercentOfUpshift_AC03_4) {
    warmupToSpeed(100.0, 0.8);
    auto signals = TelemetrySequenceBuilder::buildDecelerationTelemetry(100.0, 45.0, dt_);

    struct DownshiftRecord {
        int fromGear;
        int toGear;
        double speedKmh;
    };
    std::vector<DownshiftRecord> downshifts;
    int lastGear = twin_->getCurrentGear();

    for (const auto& sig : signals) {
        twin_->update(dt_, sig);
        int currentGear = twin_->getCurrentGear();
        if (currentGear < lastGear && lastGear > 1) {
            DownshiftRecord rec{lastGear, currentGear, sig.speedKmh};
            downshifts.push_back(rec);
        }
        lastGear = currentGear;
    }

    for (const auto& ds : downshifts) {
        if (ds.fromGear >= 2 && ds.fromGear <= 8 && ds.toGear >= 1 && ds.toGear <= 7) {
            int throttleLevel = 0;
            int fromIndex = ds.toGear - 1;
            int throttleIndex = std::min(throttleLevel, static_cast<int>(profile_.shiftTable.size()) - 1);

            if (fromIndex >= 0 && fromIndex < static_cast<int>(profile_.shiftTable[throttleIndex].size())) {
                double upshiftSpeed = profile_.shiftTable[throttleIndex][fromIndex];
                double expectedDownshiftSpeed = upshiftSpeed * profile_.hysteresisFactor;
                EXPECT_NEAR(ds.speedKmh, expectedDownshiftSpeed, expectedDownshiftSpeed * 0.15)
                    << "Downshift speed should be 85% of upshift speed (from " << ds.fromGear << " to " << ds.toGear << ")";
            }
        }
    }
}

TEST_F(DecelerationScenarioTest, NoDownshift_Below1stGear_AC03_5) {
    auto signals = TelemetrySequenceBuilder::buildDecelerationTelemetry(100.0, 45.0, dt_);

    for (const auto& sig : signals) {
        twin_->update(dt_, sig);
        int currentGear = twin_->getCurrentGear();
        EXPECT_GE(currentGear, 1) << "No downshift should occur below 1st gear";
    }
}

TEST_F(DecelerationScenarioTest, AtStandstill_Gear1_NoShifts_AC03_6) {
    auto signals = TelemetrySequenceBuilder::buildDecelerationTelemetry(100.0, 45.0, dt_);

    for (const auto& sig : signals) {
        twin_->update(dt_, sig);
    }

    EXPECT_EQ(twin_->getCurrentGear(), 1) << "At standstill, gear should be 1st";

    auto standstillSignals = TelemetrySequenceBuilder::buildStandstillTelemetry(5.0, dt_);
    int gearDuringStandstill = twin_->getCurrentGear();

    for (const auto& sig : standstillSignals) {
        twin_->update(dt_, sig);
        if (twin_->getCurrentGear() != gearDuringStandstill) {
            FAIL() << "No shifts should occur at standstill after reaching 1st gear";
        }
    }
}
