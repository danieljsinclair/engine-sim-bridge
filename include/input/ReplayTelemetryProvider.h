// ReplayTelemetryProvider.h
//
// Replays a timecoded telemetry CSV as the simulation's input source. Designed
// for bench-driving the simulator from a recorded log (real vehicle CSV) or a
// hand-authored script for spikes/diagnostics.
//
// CSV is header-driven. Recognised columns (any subset; unknown columns ignored):
//   time_s / timestamp_ms  relative seconds (or epoch ms -> auto-converted)
//   throttle_pct / throttle_percent / throttle  0..100 -> normalised to 0..1
//   road_speed_kmh / speed_kmh   km/h. Blank/negative = dyno OFF (RPM emergent)
//   gear            blank/-1 = unchanged; 0 = neutral; 1..8 = forward (set directly)
//   gear_selector   PRNDL string (P/R/N/D) — followed by the auto gearbox
//   clutch_pct      0..100 -> clutch pressure 0..1. Blank/-1 = unchanged
//
// autoStart fires starterButton once on the first frame so the CrankingController
// cranks the engine. autoGearbox owns an AutomaticGearbox that decides gears from
// the CSV road speed. Q (quit) + P (preset cycle) work during replay if a keyboard
// + session are wired via setKeyboardInput / setSession.

#ifndef INPUT_REPLAY_TELEMETRY_PROVIDER_H
#define INPUT_REPLAY_TELEMETRY_PROVIDER_H

#include "io/IInputProvider.h"
#include "simulator/EngineSimTypes.h"
#include "twin/IceVehicleProfile.h"
#include "input/IKeyboardInput.h"
#include "session/ISimulatorSession.h"

#include <memory>
#include <string>
#include <vector>

namespace twin { class AutomaticGearbox; }

namespace input {

class ReplayTelemetryProvider : public IInputProvider {
public:
    explicit ReplayTelemetryProvider(std::string csvPath, bool autoStart = true,
                                     bool autoGearbox = false);
    ~ReplayTelemetryProvider() override;

    bool Initialize() override;
    void Shutdown() override {}
    bool IsConnected() const override { return connected_; }
    EngineInput OnUpdateSimulation(double dt) override;
    void provideFeedback(const EngineSimStats& stats) override;
    std::string GetProviderName() const override { return "ReplayTelemetry"; }
    std::string GetLastError() const override { return lastError_; }

    // Enable Q (quit) + P (preset cycle) during replay.
    void setKeyboardInput(IKeyboardInput* keyboard) { keyboard_ = keyboard; }
    void setSession(ISimulatorSession* session) { session_ = session; }

    // Total span of the parsed trace, in seconds (last sample time). 0 if empty.
    double durationS() const;

    // Reconfigure the gearbox profile to match the ACTUAL engine preset's ratios.
    // Auto-generates a shift table from the ratios + redline so shift points are
    // correct regardless of the transmission (7-speed AMG, 8-speed ZF, etc.).
    void reconfigureProfile(const std::vector<double>& gearRatios,
                             double diffRatio, double tireRadiusM);

private:
    struct Sample {
        double timeS = 0.0;
        double throttle = 0.0;       // 0..1
        double roadSpeedKmh = -2.0;  // -2 sentinel = not commanded (dyno off)
        int gear = -1;               // -1 = unchanged; 0 = neutral; 1..8 = forward
        double clutchPct = -1.0;     // -1 = unchanged; 0..1
        std::string gearSelector;    // PRNDL string from vehicle-sim captures
    };

    bool parseCsv();
    const Sample& sampleAt(double t) const;

    std::string csvPath_;
    bool autoStart_;
    bool autoGearbox_;
    bool connected_ = false;
    std::string lastError_;
    std::vector<Sample> samples_;
    double elapsedS_ = 0.0;
    bool startFired_ = false;
    twin::IceVehicleProfile gearboxProfile_;  // OWNED: the gearbox holds a reference to this
    std::unique_ptr<twin::AutomaticGearbox> gearbox_;

    // Keyboard input support for Q/P during replay
    IKeyboardInput* keyboard_ = nullptr;
    ISimulatorSession* session_ = nullptr;
    double engineRpmFeedback_ = 0.0;  // last engine RPM from the physics (for slip calc)
    bool ignitionOn_ = true;          // toggled by 'I' key during replay
};

} // namespace input

#endif // INPUT_REPLAY_TELEMETRY_PROVIDER_H
