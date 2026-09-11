#include <gtest/gtest.h>
#include <twin/AutomaticGearbox.h>
#include <twin/IceVehicleProfile.h>

using namespace twin;

namespace {

IceVehicleProfile makeReworkProfile() {
    IceVehicleProfile profile = IceVehicleProfile::zf8hp45();
    profile.gearboxRework.enabled = true;
    return profile;
}

IceVehicleProfile makeCustomLegacyProfile() {
    IceVehicleProfile profile = IceVehicleProfile::zf8hp45();
    profile.shiftTableThrottleLevels = {0.0, 1.0};
    profile.shiftTable = {{11.0, 48.0}, {11.0, 48.0}};
    profile.separateDownshiftTableEnabled = true;
    profile.downshiftTableThrottleLevels = {0.0, 1.0};
    profile.downshiftTable = {{9.0, 34.0}, {9.0, 34.0}};
    profile.shiftDwellS = 0.4;
    return profile;
}

struct ReworkFixture : ::testing::Test {
    IceVehicleProfile profile = makeReworkProfile();
    AutomaticGearbox gearbox{profile};

    void settle() {
        for (int i = 0; i < 40; ++i) {
            gearbox.update(0.1, 60.0, 0.30, 100.0, false, 100.0);
        }
    }
};

}  // namespace

TEST(GearboxReworkTest, DefaultProfileLeavesReworkDisabled) {
    const IceVehicleProfile profile = IceVehicleProfile::zf8hp45();

    EXPECT_FALSE(profile.gearboxRework.enabled);
}

TEST(GearboxReworkTest, LightThrottleRaisesFirstUpshiftSpeed) {
    IceVehicleProfile legacyProfile = IceVehicleProfile::zf8hp45();
    IceVehicleProfile reworkProfileValue = makeReworkProfile();
    AutomaticGearbox legacy{legacyProfile};
    AutomaticGearbox rework{reworkProfileValue};

    const double legacySpeed = [&]() {
        for (double speed = 10.0; speed <= 60.0; speed += 1.0) {
            legacy.update(0.1, speed, 0.25, 0.0, false, 0.0);
            if (legacy.getCurrentGear() > 1) return speed;
        }
        return 60.0;
    }();
    const double reworkSpeed = [&]() {
        for (double speed = 10.0; speed <= 60.0; speed += 1.0) {
            rework.update(0.1, speed, 0.25, 0.0, false, 0.0);
            if (rework.getCurrentGear() > 1) return speed;
        }
        return 60.0;
    }();

    EXPECT_GT(reworkSpeed, legacySpeed);
    EXPECT_GE(reworkSpeed, 30.0);
}

TEST_F(ReworkFixture, DwellHoldsLightThrottleGearForAtLeastTwoAndAHalfSeconds) {
    gearbox.update(0.1, 40.0, 0.25);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    gearbox.update(2.4, 20.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
    gearbox.update(0.1, 60.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
    gearbox.update(0.1, 60.0, 0.25);
    EXPECT_GT(gearbox.getCurrentGear(), 2);
}

TEST_F(ReworkFixture, ClosedThrottleDownshiftIsDelayedUntilHoldExpires) {
    settle();
    const int cruiseGear = gearbox.getCurrentGear();
    ASSERT_GE(cruiseGear, 3);

    for (int i = 0; i < 15; ++i) {
        gearbox.update(0.1, 20.0, 0.0, 0.0, false, 0.0);
        EXPECT_EQ(gearbox.getCurrentGear(), cruiseGear);
    }
    for (int i = 0; i < 10; ++i) {
        gearbox.update(0.1, 20.0, 0.0, 0.0, false, 0.0);
    }
    EXPECT_LT(gearbox.getCurrentGear(), cruiseGear);
}

TEST_F(ReworkFixture, BrakeLightBypassesClosedThrottleDownshiftDelay) {
    settle();
    const int cruiseGear = gearbox.getCurrentGear();
    ASSERT_GE(cruiseGear, 3);

    for (int i = 0; i < 15; ++i) {
        gearbox.update(0.1, 20.0, 0.0, 0.0, true, 0.0);
    }

    EXPECT_LT(gearbox.getCurrentGear(), cruiseGear);
}

TEST_F(ReworkFixture, SustainedPositiveMotorTorqueHoldsGearAtLowSpeed) {
    settle();
    const int cruiseGear = gearbox.getCurrentGear();
    ASSERT_GE(cruiseGear, 3);

    for (int i = 0; i < 150; ++i) {
        gearbox.update(0.1, 20.0, 0.0, 0.0, false, 1200.0);
    }

    EXPECT_EQ(gearbox.getCurrentGear(), cruiseGear);
}

TEST(GearboxReworkTest, SustainedNegativeMotorTorqueAllowsEarlierUpshift) {
    IceVehicleProfile profile = makeReworkProfile();
    profile.gearboxRework.motorTorqueHoldS = 0.0;
    AutomaticGearbox neutral{profile};
    AutomaticGearbox regen{profile};

    const auto findUpshiftSpeed = [](AutomaticGearbox& gearbox, double motorTorqueNm) {
        for (double speed = 10.0; speed <= 60.0; speed += 1.0) {
            gearbox.update(0.1, speed, 0.25, 0.0, false, motorTorqueNm);
            if (gearbox.getCurrentGear() > 1) return speed;
        }
        return 60.0;
    };
    const double neutralSpeed = findUpshiftSpeed(neutral, 0.0);
    const double regenSpeed = findUpshiftSpeed(regen, -1200.0);

    EXPECT_LT(regenSpeed, neutralSpeed);
}

TEST(GearboxReworkTest, ToggleOffIgnoresReworkTablesDwellAndInputs) {
    IceVehicleProfile legacyProfile = makeCustomLegacyProfile();
    IceVehicleProfile disabledProfile = legacyProfile;
    disabledProfile.gearboxRework.enabled = false;
    disabledProfile.gearboxRework.shiftDwellLightS = 99.0;
    disabledProfile.gearboxRework.closedThrottleDownshiftHoldS = 99.0;
    disabledProfile.gearboxRework.motorTorqueUpshiftBias = 0.99;
    AutomaticGearbox legacy{legacyProfile};
    AutomaticGearbox disabled{disabledProfile};

    const std::vector<std::tuple<double, double, double>> frames{
        {0.1, 12.0, 0.25},
        {0.1, 30.0, 0.25},
        {0.4, 45.0, 0.25},
        {0.1, 50.0, 0.25},
        {0.1, 20.0, 0.0},
        {0.1, 20.0, 0.0},
    };
    for (const auto& [dt, speed, throttle] : frames) {
        legacy.update(dt, speed, throttle, 0.0, false, 1200.0);
        disabled.update(dt, speed, throttle, 0.0, true, -1200.0);
        EXPECT_EQ(disabled.getCurrentGear(), legacy.getCurrentGear());
    }
    EXPECT_EQ(disabled.getCurrentGear(), legacy.getCurrentGear());
}
