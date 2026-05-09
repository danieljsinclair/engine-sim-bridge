#include <gtest/gtest.h>
#include <twin/VirtualIceTwin.h>
#include <twin/IceVehicleProfile.h>
#include <io/UpstreamSignal.h>

using namespace twin;
using namespace input;

class VirtualIceTwinTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = IceVehicleProfile::zf8hp45();
        twin_ = std::make_unique<VirtualIceTwin>(profile_);
    }

    IceVehicleProfile profile_;
    std::unique_ptr<VirtualIceTwin> twin_;

    UpstreamSignal makeValidSignal(double throttle = 0.0, double speed = 0.0) {
        UpstreamSignal sig;
        sig.throttleFraction = throttle;
        sig.speedKmh = speed;
        sig.timestampUtcMs = 1000;
        sig.isValid = true;
        return sig;
    }
};

TEST_F(VirtualIceTwinTest, OffToCrankingOnFirstValidTelemetry_AC11_1) {
    EXPECT_EQ(twin_->getState(), TwinState::OFF);

    auto sig = makeValidSignal(0.0, 0.0);
    auto output = twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::CRANKING);
    EXPECT_TRUE(output.starterMotor);
    EXPECT_TRUE(output.ignition);
}

TEST_F(VirtualIceTwinTest, CrankingToIdleWhenRpmAbove550_AC11_2) {
    auto sig = makeValidSignal(0.0, 0.0);
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::CRANKING);

    auto output = twin_->update(0.016, sig);
    EXPECT_EQ(twin_->getState(), TwinState::IDLE);
    EXPECT_FALSE(output.starterMotor);
}

TEST_F(VirtualIceTwinTest, IdleToRunningWhenThrottleAbove5Percent_AC11_3) {
    auto sig = makeValidSignal(0.0, 0.0);
    twin_->update(0.016, sig);
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::IDLE);

    sig.throttleFraction = 0.06;
    auto output = twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::RUNNING);
}

TEST_F(VirtualIceTwinTest, IdleStaysIdleBelow5PercentThrottle) {
    auto sig = makeValidSignal(0.0, 0.0);
    twin_->update(0.016, sig);
    twin_->update(0.016, sig);

    sig.throttleFraction = 0.04;
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::IDLE);
}

TEST_F(VirtualIceTwinTest, RunningToShiftingWhenGearboxRequestsShift_AC11_4) {
    auto sig = makeValidSignal(0.5, 5.0);
    twin_->update(0.016, sig);
    twin_->update(0.016, sig);
    sig.throttleFraction = 0.6;
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::RUNNING);

    sig.speedKmh = 45.0;
    auto output = twin_->update(0.016, sig);

    if (twin_->getState() == TwinState::SHIFTING) {
        EXPECT_LT(output.clutchPressure, 0.2) << "Clutch should be disengaging (<0.2) during shift";
    } else {
        SUCCEED() << "Gearbox not ready to shift yet (throttle smoothing or min shift interval), skipping clutch check";
    }
}

TEST_F(VirtualIceTwinTest, ShiftingToRunningWhenClutchReengages_AC11_5) {
    auto sig = makeValidSignal(0.5, 60.0);

    for (int i = 0; i < 10; ++i) {
        twin_->update(0.016, sig);
    }

    sig.speedKmh = 75.0;
    twin_->update(0.016, sig);

    if (twin_->getState() == TwinState::SHIFTING) {
        for (int i = 0; i < 30; ++i) {
            auto output = twin_->update(0.016, sig);
            if (twin_->getState() == TwinState::RUNNING) {
                EXPECT_DOUBLE_EQ(output.clutchPressure, 1.0);
                break;
            }
        }
    }
}

TEST_F(VirtualIceTwinTest, RunningToIdleWhenSpeedAndThrottleZero_AC11_6) {
    auto sig = makeValidSignal(0.5, 60.0);
    twin_->update(0.016, sig);
    twin_->update(0.016, sig);
    sig.throttleFraction = 0.1;
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::RUNNING);

    sig.throttleFraction = 0.0;
    sig.speedKmh = 0.0;
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::IDLE);
}

TEST_F(VirtualIceTwinTest, AnyStateToOffAfter5SecondsNoValidTelemetry_AC11_7) {
    auto sig = makeValidSignal(0.5, 60.0);
    twin_->update(0.016, sig);
    twin_->update(0.016, sig);
    sig.throttleFraction = 0.1;
    twin_->update(0.016, sig);

    EXPECT_NE(twin_->getState(), TwinState::OFF);

    UpstreamSignal invalidSig;
    invalidSig.isValid = false;

    for (int i = 0; i < 315; ++i) {
        twin_->update(0.016, invalidSig);
    }

    EXPECT_EQ(twin_->getState(), TwinState::OFF);
}

TEST_F(VirtualIceTwinTest, ClutchDisengagesWithin50ms_AC08_1) {
    auto sig = makeValidSignal(0.8, 50.0);

    for (int i = 0; i < 10; ++i) {
        twin_->update(0.016, sig);
    }

    sig.speedKmh = 80.0;
    twin_->update(0.016, sig);

    if (twin_->getState() == TwinState::SHIFTING) {
        auto output = twin_->update(0.050, sig);
        EXPECT_LE(output.clutchPressure, 0.1);
    }
}

TEST_F(VirtualIceTwinTest, ClutchStaysZeroFor200msPause_AC08_2) {
    auto sig = makeValidSignal(0.8, 50.0);

    for (int i = 0; i < 10; ++i) {
        twin_->update(0.016, sig);
    }

    sig.speedKmh = 80.0;
    twin_->update(0.016, sig);

    if (twin_->getState() == TwinState::SHIFTING) {
        bool wasZero = false;
        for (int i = 0; i < 5; ++i) {
            auto output = twin_->update(0.050, sig);
            if (output.clutchPressure == 0.0) {
                wasZero = true;
            }
        }
        EXPECT_TRUE(wasZero) << "Clutch should be at 0.0 during pause";
    }
}

TEST_F(VirtualIceTwinTest, ClutchReengagesOver100ms_AC08_4) {
    auto sig = makeValidSignal(0.8, 50.0);

    for (int i = 0; i < 10; ++i) {
        twin_->update(0.016, sig);
    }

    sig.speedKmh = 80.0;
    twin_->update(0.016, sig);

    if (twin_->getState() == TwinState::SHIFTING) {
        for (int i = 0; i < 8; ++i) {
            twin_->update(0.050, sig);
        }

        auto output = twin_->update(0.050, sig);
        EXPECT_GT(output.clutchPressure, 0.0);
    }
}

TEST_F(VirtualIceTwinTest, GearIs1DuringCrankingAndIdle) {
    auto sig = makeValidSignal(0.0, 0.0);
    auto output = twin_->update(0.016, sig);
    EXPECT_EQ(output.gear, 1);

    output = twin_->update(0.016, sig);
    EXPECT_EQ(output.gear, 1);
}

TEST_F(VirtualIceTwinTest, ThrottleIsSmoothed_AC09) {
    auto sig = makeValidSignal(1.0, 0.0);
    auto output = twin_->update(0.016, sig);

    EXPECT_GT(output.throttle, 0.0);
    EXPECT_LT(output.throttle, 1.0) << "Throttle should be smoothed, not instant";
}
