#include <gtest/gtest.h>
#include <io/UpstreamSignal.h>

using namespace input;

TEST(UpstreamSignal, DefaultConstruction)
{
    UpstreamSignal signal;
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.0);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 0.0);
    EXPECT_DOUBLE_EQ(signal.accelerationG, 0.0);
    EXPECT_DOUBLE_EQ(signal.brakeFraction, 0.0);
    EXPECT_EQ(signal.timestampUtcMs, 0);
    EXPECT_FALSE(signal.isValid);
}

TEST(UpstreamSignal, FieldAssignment)
{
    UpstreamSignal signal;
    signal.throttleFraction = 0.75;
    signal.speedKmh = 120.5;
    signal.accelerationG = 0.3;
    signal.brakeFraction = 0.2;
    signal.timestampUtcMs = 1715260800000;
    signal.isValid = true;

    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.75);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 120.5);
    EXPECT_DOUBLE_EQ(signal.accelerationG, 0.3);
    EXPECT_DOUBLE_EQ(signal.brakeFraction, 0.2);
    EXPECT_EQ(signal.timestampUtcMs, 1715260800000);
    EXPECT_TRUE(signal.isValid);
}

TEST(UpstreamSignal, CopyConstruction)
{
    UpstreamSignal original;
    original.throttleFraction = 0.5;
    original.speedKmh = 60.0;
    original.accelerationG = 0.1;
    original.brakeFraction = 0.0;
    original.timestampUtcMs = 1234567890;
    original.isValid = true;

    UpstreamSignal copy(original);

    EXPECT_DOUBLE_EQ(copy.throttleFraction, 0.5);
    EXPECT_DOUBLE_EQ(copy.speedKmh, 60.0);
    EXPECT_DOUBLE_EQ(copy.accelerationG, 0.1);
    EXPECT_DOUBLE_EQ(copy.brakeFraction, 0.0);
    EXPECT_EQ(copy.timestampUtcMs, 1234567890);
    EXPECT_TRUE(copy.isValid);
}

TEST(UpstreamSignal, AssignmentOperator)
{
    UpstreamSignal original;
    original.throttleFraction = 0.9;
    original.speedKmh = 180.0;
    original.accelerationG = 0.5;
    original.brakeFraction = 0.8;
    original.timestampUtcMs = 9876543210;
    original.isValid = false;

    UpstreamSignal copy;
    copy = original;

    EXPECT_DOUBLE_EQ(copy.throttleFraction, 0.9);
    EXPECT_DOUBLE_EQ(copy.speedKmh, 180.0);
    EXPECT_DOUBLE_EQ(copy.accelerationG, 0.5);
    EXPECT_DOUBLE_EQ(copy.brakeFraction, 0.8);
    EXPECT_EQ(copy.timestampUtcMs, 9876543210);
    EXPECT_FALSE(copy.isValid);
}
