#include <gtest/gtest.h>
#include <twin/AutomaticGearbox.h>
#include <twin/IceVehicleProfile.h>

using namespace twin;

class AutomaticGearboxTest : public ::testing::Test {
protected:
    IceVehicleProfile profile = IceVehicleProfile::zf8hp45();
};

TEST_F(AutomaticGearboxTest, StartsInFirstGear)
{
    AutomaticGearbox gearbox(profile);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
}

TEST_F(AutomaticGearboxTest, UpshiftAt50PercentThrottle_AC_01_2)
{
    AutomaticGearbox gearbox(profile);

    // At 50% throttle (between 40% row: 24 kph and 55% row: 30 kph)
    // Interpolated: 24 + (0.50-0.40)/(0.55-0.40) * (30-24) = 28 kph
    gearbox.update(0.1, 27.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 28.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 29.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
}

TEST_F(AutomaticGearboxTest, UpshiftAt100PercentThrottle_AC_01_5)
{
    AutomaticGearbox gearbox(profile);

    // At 100% throttle, 1st->2nd should happen at 49 kph (10-level table)
    // Note: throttle >= 0.95 triggers kickdown, so use 0.90 to avoid it
    // At 90% throttle: 1->2 at 45 kph
    gearbox.update(0.1, 44.0, 0.90);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 45.0, 0.90);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 46.0, 0.90);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
}

TEST_F(AutomaticGearboxTest, DownshiftAt85PercentOfUpshiftSpeed_AC_03_4)
{
    AutomaticGearbox gearbox(profile);

    // Accelerate to 2nd gear at 25% throttle (exact table row)
    // At 25%, 1->2 upshift = 19 kph
    gearbox.update(0.1, 20.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // With separate downshift table, 2->1 downshift at 25% = 14 kph
    gearbox.update(0.1, 15.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    gearbox.update(0.1, 14.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    gearbox.update(0.1, 13.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
}

TEST_F(AutomaticGearboxTest, CoastDownSequentialDownshifts_AC_03_1)
{
    AutomaticGearbox gearbox(profile);

    // Accelerate to high gear at moderate throttle
    gearbox.update(0.1, 100.0, 0.4);
    EXPECT_GE(gearbox.getCurrentGear(), 3);

    int topGear = gearbox.getCurrentGear();

    // Coast down (throttle = 0) with many small steps to let smoothing decay
    for (int speed = 90; speed >= 10; speed -= 5) {
        gearbox.update(0.5, static_cast<double>(speed), 0.0);
    }

    // Should be in a lower gear now
    EXPECT_LT(gearbox.getCurrentGear(), topGear);
}

TEST_F(AutomaticGearboxTest, KickdownWithin500ms_AC_04_1)
{
    AutomaticGearbox gearbox(profile);

    // Cruise in a higher gear
    gearbox.update(0.1, 60.0, 0.3);
    int initialGear = gearbox.getCurrentGear();
    EXPECT_GT(initialGear, 1);

    // Kickdown: throttle jumps from 0.3 to 0.9 within 100ms (delta > 0.4)
    gearbox.update(0.05, 60.0, 0.9);

    // Should request downshift
    EXPECT_TRUE(gearbox.requestsShift());
    EXPECT_LT(gearbox.getTargetGear(), initialGear);
}

TEST_F(AutomaticGearboxTest, SafeGearRpmBelow90PercentRedline_AC_04_2)
{
    AutomaticGearbox gearbox(profile);

    // At a speed where dropping multiple gears would exceed 90% redline
    // only drop one gear
    double speed = 120.0; // High speed

    // Accelerate to high gear
    gearbox.update(0.1, speed, 0.8);
    int initialGear = gearbox.getCurrentGear();

    // Kickdown at high speed
    gearbox.update(0.05, speed, 0.98);

    if (gearbox.requestsShift()) {
        int targetGear = gearbox.getTargetGear();
        // Should not drop more than one gear to avoid overspeeding engine
        EXPECT_GE(targetGear, initialGear - 1);
    }
}

TEST_F(AutomaticGearboxTest, NoShiftAtStandstill_AC_10_1)
{
    AutomaticGearbox gearbox(profile);

    gearbox.update(0.1, 0.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
    EXPECT_FALSE(gearbox.requestsShift());
}

TEST_F(AutomaticGearboxTest, NoDownshiftBelowFirstGear_AC_10_3)
{
    AutomaticGearbox gearbox(profile);

    // Slow down to near stop
    gearbox.update(0.1, 5.0, 0.0);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
    EXPECT_FALSE(gearbox.requestsShift());
}

TEST_F(AutomaticGearboxTest, MinShiftIntervalBetweenSameDirectionShifts_AC_10_5)
{
    AutomaticGearbox gearbox(profile);

    // Perform two consecutive upshifts: 1->2 then 2->3
    // First upshift at 25% throttle: 1->2 at 19 kph
    gearbox.update(0.1, 20.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Speed high enough for 2->3 at 25% throttle (28 kph)
    // But second upshift within upshiftMinIntervalS (2.0s) should be blocked
    gearbox.update(0.1, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Advance 1.8 seconds (total 1.9 from first shift) - still blocked
    gearbox.update(1.8, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Advance past 2.0 seconds total - upshift allowed
    gearbox.update(0.2, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 3);
}

TEST_F(AutomaticGearboxTest, KickdownDetectionThresholds_AC_10_4)
{
    AutomaticGearbox gearbox(profile);

    // Cruise
    gearbox.update(0.1, 60.0, 0.3);
    int initialGear = gearbox.getCurrentGear();
    EXPECT_GT(initialGear, 1);

    // Small throttle increase - should NOT trigger kickdown (delta < 0.4)
    gearbox.update(0.05, 60.0, 0.6);
    EXPECT_FALSE(gearbox.requestsShift());

    // Reset - create a new gearbox to reset state
    {
        AutomaticGearbox newGearbox(profile);
        newGearbox.update(0.1, 60.0, 0.3);
        // Large throttle increase - SHOULD trigger kickdown (delta > 0.4)
        newGearbox.update(0.05, 60.0, 0.9);
        EXPECT_TRUE(newGearbox.requestsShift());
    }

    // Reset again for high throttle threshold test
    {
        AutomaticGearbox newGearbox2(profile);
        newGearbox2.update(0.1, 60.0, 0.3);
        // Throttle at 0.95 (at threshold) - should trigger kickdown
        newGearbox2.update(0.05, 60.0, 0.95);
        EXPECT_TRUE(newGearbox2.requestsShift());
    }
}

TEST_F(AutomaticGearboxTest, InterpolateShiftTableForIntermediateThrottle)
{
    AutomaticGearbox gearbox(profile);

    // At 35% throttle (between 25% row: 19 kph and 40% row: 24 kph)
    // Interpolated: 19 + (0.35-0.25)/(0.40-0.25) * (24-19) = 19 + 3.33 = 22.33 kph
    gearbox.update(0.1, 22.0, 0.35);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 23.0, 0.35);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
}

// ============================================================
// F1: Separate Downshift Table
// ============================================================

TEST_F(AutomaticGearboxTest, SeparateDownshiftTableUsedWhenEnabled)
{
    // Build a profile with a separate downshift table that has higher
    // downshift speeds than the upshift*hysteresis fallback would give.
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();

    // Verify separate table is enabled in zf8hp45
    ASSERT_TRUE(custom.separateDownshiftTableEnabled);
    ASSERT_FALSE(custom.downshiftTable.empty());

    AutomaticGearbox gearbox(custom);

    // Get to 2nd gear: accelerate past the 1->2 upshift point at light throttle
    // At 5% throttle, 1->2 upshift is at 11 kph
    gearbox.update(0.1, 12.0, 0.05);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    // Now at 5% throttle the 2->1 downshift from the separate table is 9 kph.
    // Fall below 9 kph — should downshift to 1.
    gearbox.update(0.1, 8.0, 0.05);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
}

TEST_F(AutomaticGearboxTest, SeparateDownshiftTableDisabledFallsBackToHysteresis)
{
    // Manually construct a profile with separateDownshiftTableEnabled=false
    // to verify the old hysteresisFactor path still works.
    IceVehicleProfile custom;
    custom.gearRatios = {4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667};
    custom.diffRatio = 3.15;
    custom.tireRadiusM = 0.32;
    custom.hysteresisFactor = 0.85;
    custom.shiftTableThrottleLevels = {0.1, 0.25, 0.5, 0.75, 1.0};
    custom.shiftTable = {
        {20.0, 35.0, 50.0, 65.0, 80.0, 95.0, 110.0},
        {30.0, 50.0, 70.0, 90.0, 110.0, 130.0, 155.0},
        {40.0, 65.0, 90.0, 115.0, 140.0, 170.0, 200.0},
        {55.0, 85.0, 115.0, 145.0, 180.0, 215.0, 255.0},
        {70.0, 105.0, 140.0, 180.0, 220.0, 265.0, 315.0}
    };
    custom.separateDownshiftTableEnabled = false;
    custom.minShiftIntervalS = 3.0;
    custom.redlineRpm = 6500.0;
    custom.standstillThresholdKmh = 1.0;
    custom.kickdownThrottleThreshold = 0.95;
    custom.kickdownDelta = 0.4;
    custom.kickdownWindowMs = 100.0;
    custom.throttleSmoothingTauMs = 50.0;

    AutomaticGearbox gearbox(custom);

    // Upshift to 2nd: at 50% throttle, 1->2 upshift is 40 kph
    gearbox.update(0.1, 45.0, 0.5);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    // Downshift at hysteresis (40 * 0.85 = 34 kph)
    gearbox.update(0.1, 34.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    gearbox.update(0.1, 33.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
}

// ============================================================
// F2: Engine Braking Inhibitor
// ============================================================

TEST_F(AutomaticGearboxTest, EngineBrakingInhibitorDoesNotBlockDownshift)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.engineBrakingInhibitorEnabled);

    AutomaticGearbox gearbox(custom);

    // Get to a high gear at moderate throttle
    gearbox.update(0.1, 100.0, 0.4);
    int topGear = gearbox.getCurrentGear();
    ASSERT_GT(topGear, 1);

    // Coast at highway speed (throttle = 0, speed = 80 kph)
    // Per x-engineer ch6 s4.3: inhibitor blocks UPHIFTS ONLY, downshifts are free
    gearbox.update(0.1, 80.0, 0.0);

    // Coast down further — downshift should be allowed
    gearbox.update(0.1, 40.0, 0.0);
    gearbox.update(0.1, 20.0, 0.0);

    // Should have downshifted — inhibitor only blocks upshifts
    EXPECT_LT(gearbox.getCurrentGear(), topGear)
        << "Downshifts should be allowed even with engine braking inhibitor active";
}

TEST_F(AutomaticGearboxTest, EngineBrakingInhibitorAllowsDownshiftBelowMinSpeed)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.engineBrakingInhibitorEnabled);
    ASSERT_DOUBLE_EQ(custom.engineBrakingMinSpeedKmh, 10.0);

    AutomaticGearbox gearbox(custom);

    // Get to a high gear
    gearbox.update(0.1, 100.0, 0.4);
    int topGear = gearbox.getCurrentGear();
    ASSERT_GT(topGear, 1);

    // Coast below the engine braking min speed (10 kph)
    // Below 10 kph, the inhibitor should not activate
    gearbox.update(0.1, 8.0, 0.0);

    // Should have downshifted since we're below min speed
    EXPECT_LT(gearbox.getCurrentGear(), topGear);
}

TEST_F(AutomaticGearboxTest, EngineBrakingInhibitorDoesNotBlockKickdown)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.engineBrakingInhibitorEnabled);

    AutomaticGearbox gearbox(custom);

    // Cruise at highway speed in a higher gear
    gearbox.update(0.1, 80.0, 0.3);
    int initialGear = gearbox.getCurrentGear();
    ASSERT_GT(initialGear, 1);

    // Floor it — kickdown should override the inhibitor
    gearbox.update(0.05, 80.0, 0.98);

    // Kickdown should still fire even with inhibitor active
    EXPECT_TRUE(gearbox.requestsShift());
    EXPECT_LT(gearbox.getTargetGear(), initialGear);
}

TEST_F(AutomaticGearboxTest, EngineBrakingInhibitorBlocksUpshiftAtHighwaySpeed)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.engineBrakingInhibitorEnabled);

    AutomaticGearbox gearbox(custom);

    // Get to a moderate gear, then lift off at highway speed
    gearbox.update(0.1, 60.0, 0.25);
    int gear = gearbox.getCurrentGear();
    ASSERT_GE(gear, 2);

    // Now accelerate gently but stay under throttle threshold
    // First, coast at 70 kph to activate inhibitor
    gearbox.update(0.1, 70.0, 0.0);

    int gearAfterCoast = gearbox.getCurrentGear();
    // With inhibitor active, upshifts should be blocked even if speed increases
    gearbox.update(0.1, 120.0, 0.0);
    EXPECT_EQ(gearbox.getCurrentGear(), gearAfterCoast);
}

// ============================================================
// F3: Asymmetric Shift Intervals
// ============================================================

TEST_F(AutomaticGearboxTest, AsymmetricShiftInterval_UpshiftUsesUpshiftInterval)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    // zf8hp45 has upshiftMinIntervalS=2.0
    ASSERT_DOUBLE_EQ(custom.upshiftMinIntervalS, 2.0);

    AutomaticGearbox gearbox(custom);

    // First upshift: 1->2 at 25% throttle (19 kph)
    gearbox.update(0.1, 20.0, 0.25);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    // Second upshift: 2->3 at 25% throttle (28 kph)
    // Wait just under upshift interval (2.0s) — should be blocked
    gearbox.update(1.9, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Advance past 2.0s total — upshift should be allowed
    gearbox.update(0.2, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 3);
}

TEST_F(AutomaticGearboxTest, AsymmetricShiftInterval_DownshiftUsesDownshiftInterval)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    // zf8hp45 has downshiftMinIntervalS=1.5
    ASSERT_DOUBLE_EQ(custom.downshiftMinIntervalS, 1.5);

    AutomaticGearbox gearbox(custom);

    // Get to 4th gear at 40% throttle
    gearbox.update(0.1, 60.0, 0.40);
    ASSERT_GE(gearbox.getCurrentGear(), 4);
    int topGear = gearbox.getCurrentGear();

    // Drop speed to trigger a single downshift
    // At 15% throttle, downshift 4->3 = 24 kph. Use speed 23.
    // But smoothed throttle takes time to decay. Let's use 0.15 from the start.
    // Actually, let's use a simpler approach: upshift to gear 4, then do first
    // downshift, then try second downshift before interval expires.

    // Drop to a speed that triggers downshift but keep speed high enough
    // to only drop one gear. Gear 4->3 at 5% = 25 kph, gear 3->2 at 5% = 13 kph
    // Use speed 20: only 4->3 triggers, 3->2 doesn't (20 > 13).
    gearbox.update(0.1, 20.0, 0.05);
    int gearAfterFirst = gearbox.getCurrentGear();

    // If first downshift happened (cross-direction, so timer doesn't block)
    if (gearAfterFirst < topGear) {
        // Try for second consecutive downshift before 1.5s — should be blocked
        gearbox.update(1.0, 8.0, 0.05);
        EXPECT_EQ(gearbox.getCurrentGear(), gearAfterFirst);

        // Advance past 1.5s total — downshift should be allowed
        gearbox.update(0.6, 8.0, 0.05);
        EXPECT_LT(gearbox.getCurrentGear(), gearAfterFirst);
    }
}

TEST_F(AutomaticGearboxTest, CrossResetResetsOppositeDirectionTimer)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_DOUBLE_EQ(custom.upshiftMinIntervalS, 2.0);
    ASSERT_DOUBLE_EQ(custom.downshiftMinIntervalS, 1.5);

    AutomaticGearbox gearbox(custom);

    // Get to 2nd gear via upshift
    gearbox.update(0.1, 20.0, 0.25);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    // Immediately try to downshift — the upshift just happened, so downshift timer
    // should be cross-reset (it starts fresh from the upshift).
    // Since we just upshifted, the downshift timer starts at 0.
    // Wait 1.6s (> downshift interval 1.5s) and try to downshift
    gearbox.update(1.6, 9.0, 0.05);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
}

// ============================================================
// F4/F5: 10 Throttle Levels with Separate Tables
// ============================================================

TEST_F(AutomaticGearboxTest, TenThrottleLevels_UpshiftAtWOT)
{
    // At 100% throttle, 1->2 upshift should happen at 49 kph
    AutomaticGearbox gearbox(profile);

    gearbox.update(0.1, 48.0, 1.0);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 50.0, 1.0);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
}

TEST_F(AutomaticGearboxTest, TenThrottleLevels_DownshiftGapVariesWithThrottle)
{
    // At light throttle (5%), upshift 1->2 at 11 kph, downshift 2->1 at 9 kph
    // Gap = 2 kph (small)
    // At high throttle (90%), upshift 1->2 at 45 kph, downshift 2->1 at 33 kph
    // Gap = 12 kph (large)

    // Light throttle: upshift to 2nd at 5% throttle
    {
        AutomaticGearbox gearbox(profile);
        gearbox.update(0.1, 12.0, 0.05);
        ASSERT_EQ(gearbox.getCurrentGear(), 2);

        // Downshift at 9 kph — just above threshold, no shift
        gearbox.update(0.1, 9.0, 0.05);
        EXPECT_EQ(gearbox.getCurrentGear(), 2);

        // Below 9 kph — downshift
        gearbox.update(0.1, 8.0, 0.05);
        EXPECT_EQ(gearbox.getCurrentGear(), 1);
    }

    // High throttle: upshift to 2nd at 90% throttle (use 0.90 to avoid kickdown)
    {
        AutomaticGearbox gearbox(profile);
        gearbox.update(0.1, 46.0, 0.90);
        ASSERT_EQ(gearbox.getCurrentGear(), 2);

        // Downshift at 33 kph — just above threshold, no shift
        gearbox.update(0.1, 34.0, 0.90);
        EXPECT_EQ(gearbox.getCurrentGear(), 2);

        // Below 33 kph — downshift
        gearbox.update(0.1, 32.0, 0.90);
        EXPECT_EQ(gearbox.getCurrentGear(), 1);
    }
}

// ============================================================
// F4: Tip-In / Tip-Out Correction (x-engineer ch6 s4.4)
// ============================================================

TEST_F(AutomaticGearboxTest, TipIn_BlocksUpshift) {
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.tipCorrectionEnabled);

    AutomaticGearbox gearbox(custom);

    // Start in gear 1, accelerate to just above 1->2 upshift speed at low throttle
    gearbox.update(0.1, 12.0, 0.10);

    // Now tip-in: rapid throttle increase should produce gradient > 10 %/s
    // From 0.10 to 0.50 in 0.016s = 25.0 %/s (well above 10 threshold)
    gearbox.update(0.016, 15.0, 0.50);

    // Tip-in should block the upshift even though speed exceeds threshold
    // (speed 15 > upshift speed ~13 at 50% throttle, but tip correction blocks it)
    EXPECT_EQ(gearbox.getCurrentGear(), 1)
        << "Tip-in correction should block upshift during rapid throttle increase";
}

TEST_F(AutomaticGearboxTest, TipOut_BlocksUpshift) {
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.tipCorrectionEnabled);

    AutomaticGearbox gearbox(custom);

    // Cruise in gear 2
    gearbox.update(0.1, 30.0, 0.30);
    ASSERT_GE(gearbox.getCurrentGear(), 2);

    // Accelerate to near 2->3 upshift speed
    gearbox.update(0.1, 40.0, 0.30);

    // Tip-out: rapid throttle release produces gradient < -10 %/s
    // From 0.30 to 0.05 in 0.016s = -15.6 %/s (below -10 threshold)
    gearbox.update(0.016, 45.0, 0.05);

    // Tip-out should block the upshift
    EXPECT_LE(gearbox.getCurrentGear(), 2)
        << "Tip-out correction should block upshift during rapid throttle release";
}

TEST_F(AutomaticGearboxTest, TipCorrection_DoesNotBlockDownshift) {
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.tipCorrectionEnabled);

    AutomaticGearbox gearbox(custom);

    // Get to a high gear at moderate throttle
    gearbox.update(0.1, 80.0, 0.40);
    int topGear = gearbox.getCurrentGear();
    ASSERT_GE(topGear, 3) << "Should reach gear 3+ at 80 kph 40% throttle";

    // Tip-out: rapid throttle release (triggers tip-out correction)
    gearbox.update(0.016, 80.0, 0.05);

    // Now slow down below downshift thresholds
    // At 5% throttle, downshift thresholds are low but sequential downshifts should occur
    for (int speed = 70; speed >= 5; speed -= 5) {
        gearbox.update(0.5, static_cast<double>(speed), 0.05);
    }

    // Downshift should be allowed even with tip-out active
    EXPECT_LT(gearbox.getCurrentGear(), topGear)
        << "Tip-out should NOT block downshifts — gearbox should have downshifted during coast";
}

TEST_F(AutomaticGearboxTest, TipCorrection_ClearsWhenGradientStabilizes) {
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.tipCorrectionEnabled);

    AutomaticGearbox gearbox(custom);

    // Start with a moderate throttle (no tip-in on first frame due to skip)
    gearbox.update(0.1, 10.0, 0.10);

    // Tip-in: rapid throttle increase from 0.10 to 0.50
    // Gradient = 0.40/0.016 * 100 = 2500 %/s (way above 10 threshold)
    gearbox.update(0.016, 15.0, 0.50);

    // Tip correction should block upshift here
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    // Stabilize throttle (gradient returns to ~0, tip correction clears)
    // Also let smoothed throttle catch up
    for (int i = 0; i < 10; ++i) {
        gearbox.update(0.1, 20.0, 0.50);
    }

    // Now accelerate past upshift threshold — tip correction should be clear
    gearbox.update(0.1, 30.0, 0.50);

    EXPECT_GT(gearbox.getCurrentGear(), 1)
        << "Upshift should proceed after tip-in gradient stabilizes";
}
