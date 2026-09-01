// WheelCoupling.h - Strategy for which wheel speed feeds the live clutch slip-lock.
//
// The live (VirtualIceTwin) clutch slip-lock can be driven by either the ACTUAL
// simulated wheel speed (from EngineSimStats feedback - FREE mode, default) or
// the upstream CSV road speed (PIN mode - mirrors replay, which pins the sim
// vehicle speed to the CSV road speed). These are two strategies behind one
// interface so the twin carries NO scattered conditionals; the mode is selected
// at runtime by a CLI toggle and injected via a factory (Open/Closed).

#ifndef TWIN_WHEEL_COUPLING_H
#define TWIN_WHEEL_COUPLING_H

#include <memory>

#include <twin/SlipLockController.h>

namespace twin {

// Which wheel-speed source drives the slip-lock / vehicle-speed pin.
enum class WheelCouplingMode { Free, Pin, Torque };

// Strategy: given the live feedback wheel speed and the upstream CSV road speed,
// decide (a) which speed the slip-lock controller should treat as the wheel
// speed, and (b) what vehicleSpeedTargetKmh to surface to the simulator
// (-1.0 means "don't pin" - leaves the sim speed independent).
class IWheelCoupling {
public:
    virtual ~IWheelCoupling() = default;

    // The wheel speed (km/h) to feed the slip-lock implied-RPM math.
    virtual double slipLockWheelSpeedKmh(double actualSimKmh, double csvKmh) const = 0;

    // The vehicleSpeedTargetKmh to surface for the simulator (-1.0 = no pin).
    virtual double vehicleSpeedTargetKmh(double csvKmh) const = 0;

    // Does this mode need the torque-converter launch assist at standstill?
    // Modes that leave the sim vehicle speed INDEPENDENT (Free, Torque) hit the
    // velocity-match catch-22 at standstill and need the stall-gated launch
    // pressure so the clutch can transmit torque at 0 wheel speed. PIN drives
    // the wheels directly (no catch-22) and returns false. Expressing this on
    // the strategy keeps the twin free of mode-specific conditionals (OCP).
    virtual bool launchAssistAtStandstill() const = 0;

    // Does this mode open the clutch to relieve creep-drag at standstill? PIN
    // pins the sim wheels to the upstream road speed, so at standstill an engaged
    // clutch (even at the slip-lock floor) couples the engine to a wheel that
    // cannot turn and drags it down through idle to stall (the creep clutch-drag
    // bug). PIN relieves by opening the clutch so the engine idles decoupled
    // (the pinned wheel follows the CSV regardless). FREE/TORQUE do NOT relieve:
    // their vehicle speed is independent/emergent, so at standstill their
    // launch-assist path owns the clutch and an open clutch would free-rev them.
    // The twin applies the relief uniformly below the profile's standstill
    // threshold (no mode conditional in the twin) — this method only declares
    // the strategy's behaviour, mirroring launchAssistAtStandstill above.
    virtual bool relievesCreepDragAtStandstill() const = 0;

    // Recorded drivetrain torque (Nm) to inject at the transmission input this
    // frame (MATCH mode). Torque returns the recorded value verbatim so the
    // solver integrates road speed from it; Free/Pin return 0 (no injection — a
    // true no-op on the rotating mass).
    virtual double injectedInputTorqueNm(double recordedTorqueNm) const = 0;

    // Which mode this strategy implements (used by the twin for mode-specific
    // behaviour such as the FREE-mode standstill clutch-pressure floor).
    virtual WheelCouplingMode getMode() const = 0;

    // Clutch pressure for PIN mode. Returns a concrete clutch pressure that the
    // twin uses INSTEAD of the emergent slip-lock pass — PIN pins the wheels, so
    // the clutch must be the bounded slip-lock (engine revs on launch via partial
    // pressure, locks at cruise) rather than the rigid 1.0 full-lock or the
    // FREE/TORQUE emergent path. Returns -1.0 to defer to the slip-lock
    // controller (FREE/TORQUE default).
    virtual double clutchLockOverride(double engineRpm,
                                      double roadSpeedImpliedRpm,
                                      double throttleFraction,
                                      double idleRpm,
                                      double redlineRpm) const = 0;
};

// FREE (default): the slip-lock uses the ACTUAL simulated wheel speed, and we do
// NOT pin the sim vehicle speed (so the mph-vs-target diagnostic stays visible).
class FreeWheelCoupling : public IWheelCoupling {
public:
    double slipLockWheelSpeedKmh(double actualSimKmh, double /*csvKmh*/) const override {
        return actualSimKmh;
    }
    double vehicleSpeedTargetKmh(double /*csvKmh*/) const override {
        return -1.0;
    }
    bool launchAssistAtStandstill() const override { return true; }
    bool relievesCreepDragAtStandstill() const override { return false; }
    double injectedInputTorqueNm(double /*recordedTorqueNm*/) const override { return 0.0; }
    WheelCouplingMode getMode() const override { return WheelCouplingMode::Free; }
    double clutchLockOverride(double /*engineRpm*/, double /*roadSpeedImpliedRpm*/,
                              double /*throttleFraction*/, double /*idleRpm*/,
                              double /*redlineRpm*/) const override {
        return -1.0;  // Free mode: slip-lock controls clutch pressure
    }
};

// PIN: mirrors replay - the slip-lock uses the CSV road speed and the sim
// vehicle speed is pinned to that same CSV road speed.
class PinWheelCoupling : public IWheelCoupling {
public:
    double slipLockWheelSpeedKmh(double /*actualSimKmh*/, double csvKmh) const override {
        return csvKmh;
    }
    double vehicleSpeedTargetKmh(double csvKmh) const override {
        return csvKmh;
    }
    bool launchAssistAtStandstill() const override { return false; }
    bool relievesCreepDragAtStandstill() const override { return true; }
    double injectedInputTorqueNm(double /*recordedTorqueNm*/) const override { return 0.0; }
    WheelCouplingMode getMode() const override { return WheelCouplingMode::Pin; }
    double clutchLockOverride(double engineRpm, double roadSpeedImpliedRpm,
                              double throttleFraction, double idleRpm,
                              double redlineRpm) const override {
        // PIN mode: the clutch uses the BOUNDED slip-lock controller (not the
        // rigid full-lock). Because PIN pins the wheels, the wheel-speed source
        // fed to the slip-lock is the CSV road speed, so the slip-lock modulates
        // pressure from the engine↔road mismatch: at launch (high slip) it slips
        // with PARTIAL pressure (engine revs, not lugged, not rigidly locked);
        // as road catches up (slip→0) pressure→1.0 and the engine locks to
        // wheel×gear×diff. The re-tuned controller's pressure floor guarantees
        // the clutch is NEVER fully open, so the engine can never free-rev to the
        // redline (the old bug). This is the "bounded slip-lock" for PIN.
        //
        // Standstill creep-drag relief is NOT applied here (this function lacks
        // the vehicle speed) — the twin opens the clutch below the profile's
        // standstill threshold via relievesCreepDragAtStandstill() above.
        return computeSlipLockPressure(
                   SlipLockInput{engineRpm, roadSpeedImpliedRpm,
                                 throttleFraction, idleRpm, redlineRpm},
                   /*maxCreepPressure=*/0.10)
            .clutchPressure;
    }
};

// TORQUE (MATCH mode): the sim vehicle speed is NOT pinned - it EMERGES from
// the recorded motor_torque_nm injected at the drivetrain input (the twin calls
// setDrivetrainInputTorque(signal.motorTorqueNm) each frame and the solver
// integrates speed through gearbox -> diff -> wheels). Because the speed
// emerges, BOTH the slip-lock clutch AND the gearbox shift points must respond
// to the ACTUAL simulated wheel speed (like FREE), NOT the CSV target - else the
// gearbox races up the ratios on the target speed while the actual vehicle lags,
// lugging the engine in a high gear and stalling it. Launch assist is on (the
// standstill catch-22 applies, same as FREE) so the clutch couples the engine to
// the torque-driven rotating mass and engine RPM emerges from the balance.
class TorqueWheelCoupling : public IWheelCoupling {
public:
    double slipLockWheelSpeedKmh(double actualSimKmh, double /*csvKmh*/) const override {
        return actualSimKmh;
    }
    double vehicleSpeedTargetKmh(double /*csvKmh*/) const override {
        return -1.0;  // no pin - speed emerges from torque
    }
    bool launchAssistAtStandstill() const override { return true; }
    bool relievesCreepDragAtStandstill() const override { return false; }
    double injectedInputTorqueNm(double recordedTorqueNm) const override {
        return recordedTorqueNm;  // inject recorded torque -> solver integrates speed
    }
    WheelCouplingMode getMode() const override { return WheelCouplingMode::Torque; }
    double clutchLockOverride(double /*engineRpm*/, double /*roadSpeedImpliedRpm*/,
                              double /*throttleFraction*/, double /*idleRpm*/,
                              double /*redlineRpm*/) const override {
        return -1.0;  // Torque mode: slip-lock controls clutch pressure (emergent speed)
    }
};

// Factory mapping mode -> strategy (Open/Closed: add a mode by adding a branch
// here and a class - no conditional in the twin itself).
inline std::unique_ptr<IWheelCoupling> makeWheelCoupling(WheelCouplingMode mode) {
    switch (mode) {
        case WheelCouplingMode::Free:
            return std::make_unique<FreeWheelCoupling>();
        case WheelCouplingMode::Pin:
            return std::make_unique<PinWheelCoupling>();
        case WheelCouplingMode::Torque:
            return std::make_unique<TorqueWheelCoupling>();
    }
    return std::make_unique<FreeWheelCoupling>();  // defensive: unreachable enum
}

}  // namespace twin

#endif  // TWIN_WHEEL_COUPLING_H
