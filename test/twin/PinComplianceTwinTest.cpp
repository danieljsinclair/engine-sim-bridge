// PinComplianceTwinTest - the twin seam of the PIN-coupling compliance.
//
// Guards the OWNER-facing behavior at the point where the twin surfaces the
// vehicle-speed pin (TwinOutput.pinVehicleSpeedTargetKmh):
//   - tau=0 (default): the pin target IS the raw CSV road speed, frame for
//     frame — the rigid pin, bit-identical (the --pin-tau-ms 0 contract),
//   - tau>0: the pin target CHASES the staircase (finite response) instead
//     of teleporting between held levels,
//   - SCOPE: the raw road speed still drives the road-implied/slip-lock path
//     immediately (output.roadImpliedRpm) and the gearbox shift map — only
//     the PIN TARGET is chased. Interpolating the whole speed signal was
//     tried historically and flipped gear decisions; this must not regress
//     to that.
#include <gtest/gtest.h>
#include <twin/VirtualIceTwin.h>
#include <twin/IceVehicleProfile.h>
#include <io/UpstreamSignal.h>
#include <simulator/GearConventions.h>
#include <cmath>

using namespace twin;

namespace {
constexpr double kFrameDt = 0.016;
constexpr double kStepKmh = 0.9;
constexpr int kFramesPerHold = 11;  // ~181 ms hold, the measured CAN cadence
}  // namespace

class PinComplianceTwinTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = IceVehicleProfile::zf8hp45();
        twin_ = std::make_unique<VirtualIceTwin>(profile_);
        twin_->setWheelCouplingMode(WheelCouplingMode::Pin);
    }

    IceVehicleProfile profile_;
    std::unique_ptr<VirtualIceTwin> twin_;

    input::UpstreamSignal makeSignal(double throttle, double speedKmh) {
        input::UpstreamSignal sig;
        sig.throttleFraction = throttle;
        sig.speedKmh = speedKmh;
        sig.timestampUtcMs = 1000;
        sig.isValid = true;
        return sig;
    }

    // OFF -> CRANKING -> IDLE, selector to DRIVE, then one frame to RUNNING.
    void advanceToRunning() {
        auto sig = makeSignal(0.6, 0.0);
        twin_->update(kFrameDt, sig);           // OFF -> CRANKING
        twin_->setEngineRpmFeedback(800.0);
        twin_->update(kFrameDt, sig);           // CRANKING -> IDLE
        twin_->setGearSelector(bridge::GearSelector::DRIVE);
        twin_->update(kFrameDt, sig);           // IDLE -> RUNNING (D engages 1st)
        ASSERT_EQ(twin_->getState(), TwinState::RUNNING);
    }
};

TEST_F(PinComplianceTwinTest, Tau0_PinTargetIsRawCsvSpeed_FrameForFrame) {
    twin_->setPinTauMs(0.0);
    advanceToRunning();

    // The measured staircase: +0.9 km/h held levels. With tau=0 the surfaced
    // pin must equal the raw signal speed on EVERY frame — no filtering, no
    // lag, no rounding (the rigid regression contract).
    for (int level = 0; level <= 8; ++level) {
        const double raw = level * kStepKmh;
        for (int f = 0; f < kFramesPerHold; ++f) {
            const auto out = twin_->update(kFrameDt, makeSignal(0.3, raw));
            ASSERT_EQ(out.pinVehicleSpeedTargetKmh, raw)
                << "tau=0 must surface the raw CSV speed untouched";
        }
    }
}

TEST_F(PinComplianceTwinTest, Tau150_PinTargetChasesInsteadOfTeleporting) {
    twin_->setPinTauMs(150.0);
    advanceToRunning();

    double maxRawJump = 0.0;
    double maxPinJump = 0.0;
    double prevRaw = 0.0;
    double prevPin = 0.0;
    bool firstFrame = true;
    for (int level = 1; level <= 10; ++level) {
        const double raw = level * kStepKmh;
        for (int f = 0; f < kFramesPerHold; ++f) {
            const auto out = twin_->update(kFrameDt, makeSignal(0.3, raw));
            const double pin = out.pinVehicleSpeedTargetKmh;
            if (firstFrame) {
                // Engage snap: the first pinned frame is the raw speed.
                EXPECT_EQ(pin, raw);
                firstFrame = false;
            } else {
                maxRawJump = std::max(maxRawJump, std::abs(raw - prevRaw));
                maxPinJump = std::max(maxPinJump, std::abs(pin - prevPin));
            }
            prevRaw = raw;
            prevPin = pin;
        }
    }
    EXPECT_NEAR(maxRawJump, kStepKmh, 1e-9);
    EXPECT_LT(maxPinJump, 0.45 * kStepKmh)
        << "the pin target must glide, not teleport between held levels";
}

TEST_F(PinComplianceTwinTest, Tau150_RawSpeedStillDrivesRoadImpliedRpm) {
    twin_->setPinTauMs(150.0);
    advanceToRunning();

    // Cruise at 4 km/h, then a step to 10. At the step frame the pin target
    // must LAG (still chasing), but the road-implied RPM — the slip-lock /
    // shift-map path — must jump to the NEW raw speed at once. If the chase
    // leaked into the raw path, gear decisions would start flipping (the
    // historical interpolation regression).
    for (int i = 0; i < 60; ++i) {
        twin_->update(kFrameDt, makeSignal(0.2, 4.0));
    }
    const auto before = twin_->update(kFrameDt, makeSignal(0.2, 4.0));
    const auto after = twin_->update(kFrameDt, makeSignal(0.2, 10.0));

    ASSERT_EQ(before.gear, after.gear) << "test validity: same gear across the step";
    EXPECT_NEAR(after.pinVehicleSpeedTargetKmh, 4.0, 0.3)
        << "the pin target lags the step (chasing, not teleporting)";
    EXPECT_GT(after.roadImpliedRpm, before.roadImpliedRpm * 1.5)
        << "road-implied rpm must reflect the RAW stepped speed immediately";
}

TEST_F(PinComplianceTwinTest, FreeMode_SentinelUnpinned_RegardlessOfTau) {
    twin_->setPinTauMs(150.0);
    twin_->setWheelCouplingMode(WheelCouplingMode::Free);
    advanceToRunning();
    // FREE never pins: the sentinel must surface verbatim even with a chase
    // tau configured (the chase is scoped to the PIN path only).
    for (int i = 0; i < 20; ++i) {
        const auto out = twin_->update(kFrameDt, makeSignal(0.3, 30.0));
        EXPECT_EQ(out.pinVehicleSpeedTargetKmh, -1.0);
    }
}
