# Component Architecture Diagram

**Date:** 2026-05-12
**Purpose:** Visual reference for the engine-sim-bridge component hierarchy, data flow, and interface contracts.

---

## Main Pipeline

```mermaid
graph TD
    subgraph Input Sources
        KBD["Keyboard (1-0 keys)"]
        OBD["OBD2 / BLE (vehicle-sim)"]
        PHY["DemoVehiclePhysics"]
    end

    subgraph IInputProvider Implementations
        MTP["ManualTwinProvider"]
        VIP["VirtualIceInputProvider"]
        DIP["DemoInputProvider"]
    end

    subgraph IVehicleTwin Implementations
        MT["ManualTwin"]
        VIT["VirtualIceTwin"]
    end

    subgraph Core Loop
        SL["SimulationLoop"]
        SIM["BridgeSimulator"]
        ES["engine-sim (upstream)"]
        AUDIO["Audio Output"]
    end

    subgraph Output
        PRES["IPresentation"]
        TELW["ITelemetryWriter"]
    end

    KBD -->|"throttle 0-1"| PHY
    PHY -->|"UpstreamSignal"| VIP
    KBD -->|"throttle 0-1"| MTP
    OBD -->|"UpstreamSignal"| VIP

    MTP -->|"EngineInput"| SL
    VIP -->|"EngineInput"| SL
    DIP -->|"EngineInput"| SL

    SL -->|"setThrottle, setGear, setClutchPressure"| SIM
    SIM -->|"physics steps"| ES
    ES -->|"int16 audio samples"| AUDIO

    SL -->|"EngineState"| PRES
    SIM -->|"EngineSimStats"| TELW
```

---

## Feedback Loop

```mermaid
graph LR
    SIM["ISimulator"] -->|"EngineSimStats"| LOOP["SimulationLoop"]
    LOOP -->|"provideFeedback(stats)"| IP["IInputProvider"]
    IP -->|"TwinFeedback {engineRpm, vehicleSpeedKmh}"| TWIN["IVehicleTwin"]
    TWIN -->|"TwinOutput {throttle, gear, clutchPressure, ...}"| IP
    IP -->|"EngineInput"| LOOP
    LOOP -->|"setThrottle, setGear, setClutchPressure"| SIM
```

---

## Interfaces and Implementations

```mermaid
classDiagram
    class IInputProvider {
        <<interface>>
        +Initialize() bool
        +Shutdown()
        +IsConnected() bool
        +OnUpdateSimulation(dt) EngineInput
        +provideFeedback(stats)
        +GetProviderName() string
    }

    class IVehicleTwin {
        <<interface>>
        +update(dt, TwinFeedback) TwinOutput
        +getState() TwinState
    }

    class ISimulator {
        <<interface>>
        +create(config, logger, telemetry) bool
        +destroy()
        +update(deltaTime)
        +getStats() EngineSimStats
        +setThrottle(position)
        +setGear(gear)
        +setClutchPressure(pressure)
        +setIgnition(on)
        +setStarterMotor(on)
        +getEngineRpm() double
        +renderOnDemand(buffer, frames, written) bool
        +start() bool
        +stop()
    }

    class IPresentation {
        <<interface>>
        +Initialize(config) bool
        +Shutdown()
        +ShowEngineState(EngineState)
        +ShowMessage(message)
        +ShowError(error)
        +Update(dt)
    }

    class ITelemetryWriter {
        <<interface>>
        +writeEngineState(EngineStateTelemetry)
        +writeFramePerformance(FramePerformanceTelemetry)
        +writeAudioDiagnostics(AudioDiagnosticsTelemetry)
        +writeAudioTiming(AudioTimingTelemetry)
        +writeVehicleInputs(VehicleInputsTelemetry)
        +writeSimulatorMetrics(SimulatorMetricsTelemetry)
        +reset()
    }

    class ITelemetryReader {
        <<interface>>
        +getEngineState() EngineStateTelemetry
        +getFramePerformance() FramePerformanceTelemetry
        +getAudioDiagnostics() AudioDiagnosticsTelemetry
        +getAudioTiming() AudioTimingTelemetry
        +getVehicleInputs() VehicleInputsTelemetry
        +getSimulatorMetrics() SimulatorMetricsTelemetry
    }

    class ManualTwinProvider {
        -throttleSource_: IThrottleSource
        -simulator_: ISimulator
        -twin_: ManualTwin
    }

    class VirtualIceInputProvider {
        -profile_: IceVehicleProfile
        -twin_: VirtualIceTwin
        -currentSignal_: UpstreamSignal
        +setUpstreamSignal(signal)
        +provideFeedback(stats)
    }

    class DemoInputProvider {
        -throttleSource_: IThrottleSource
        -twinProvider_: VirtualIceInputProvider
        -physics_: DemoVehiclePhysics
        +getDemoRoadSpeedKmh() double
        +getDemoGear() int
        +provideFeedback(stats)
    }

    class ManualTwin {
        -inputThrottle_: double
        -inputGear_: int
        +setThrottle(throttle)
        +setGear(gear)
        +requestGearUp()
        +requestGearDown()
    }

    class VirtualIceTwin {
        -profile_: IceVehicleProfile
        -gearbox_: AutomaticGearbox
        -throttleSmoother_: ThrottleSmoother
        +setEngineRpmFeedback(rpm)
        +getCurrentGear() int
        +getSmoothedThrottle() double
    }

    class BridgeSimulator {
        -m_simulator: Simulator
        +getInternalSimulator() Simulator
    }

    class ConsolePresentation {
    }

    class InMemoryTelemetry {
    }

    IInputProvider <|.. ManualTwinProvider
    IInputProvider <|.. VirtualIceInputProvider
    IInputProvider <|.. DemoInputProvider
    IVehicleTwin <|.. ManualTwin
    ISimulator <|.. BridgeSimulator
    IPresentation <|.. ConsolePresentation
    ITelemetryWriter <|.. InMemoryTelemetry
    ITelemetryReader <|.. InMemoryTelemetry

    ManualTwinProvider --> ManualTwin : owns
    VirtualIceInputProvider --> VirtualIceTwin : owns
    DemoInputProvider --> VirtualIceInputProvider : owns
```

---

## Data Flow Structs

```mermaid
classDiagram
    class UpstreamSignal {
        <<struct>>
        +throttleFraction: double
        +speedKmh: double
        +accelerationG: double
        +brakeFraction: double
        +timestampUtcMs: uint64
        +isValid: bool
    }

    class TwinFeedback {
        <<struct>>
        +engineRpm: double
        +vehicleSpeedKmh: double
        +isValid: bool
    }

    class TwinOutput {
        <<struct>>
        +throttle: double
        +gear: int
        +clutchPressure: double
        +starterMotor: bool
        +ignition: bool
    }

    class EngineInput {
        <<struct>>
        +throttle: double
        +ignition: bool
        +starterMotor: bool
        +shouldContinue: bool
        +gearDelta: int
        +dynoTorqueScale: double
        +gearAbsolute: int
        +clutchPressure: double
        +vehicleSpeedTargetKmh: double
    }

    class EngineSimStats {
        <<struct>>
        +currentRPM: double
        +currentLoad: double
        +exhaustFlow: double
        +manifoldPressure: double
        +activeChannels: int32
        +processingTimeMs: double
        +dynoTorque: double
        +dynoTargetRPM: double
        +dynoTorqueScale: double
        +gear: int
    }

    class EngineState {
        <<struct>>
        +timestamp: double
        +rpm: double
        +throttle: double
        +load: double
        +speed: double
        +gear: int
        +ignition: bool
        +starterMotor: bool
        +exhaustFlow: double
        +dynoTorque: double
        +dynoTargetRPM: double
        +audioMode: string
        +underrunCount: int
        +renderMs: double
        +headroomMs: double
        +budgetPct: double
        +sampleRate: int
    }
```

---

## Gear Selector Flow

```mermaid
graph TD
    subgraph "User Input"
        KB["Keyboard / Demo Physics"]
    end

    subgraph "Twin Layer"
        AG["AutomaticGearbox"]
        MT2["ManualTwin"]
        VT["VirtualIceTwin"]
    end

    subgraph "Data Flow"
        TO["TwinOutput.gear"]
        EI["EngineInput.gearAbsolute"]
    end

    subgraph "Simulation"
        SL2["SimulationLoop"]
        BS["BridgeSimulator"]
        TX["Transmission"]
    end

    KB -->|"throttle"| AG
    KB -->|"gear request"| MT2
    AG -->|"shift decision"| VT
    VT -->|"TwinOutput"| TO
    MT2 -->|"TwinOutput"| TO

    TO --> EI
    EI -->|"gearAbsolute >= 0"| SL2
    SL2 -->|"setGear()"| BS
    BS -->|"changeGear()"| TX

    KB -->|"gearDelta ([/] keys)"| SL2
    SL2 -->|"applyGearChange()"| TX
```

---

## Gear Convention Mapping

```mermaid
graph LR
    subgraph "Bridge Convention"
        BG["BridgeGear: 0=Neutral, 1=1st, ..., 8=8th"]
    end
    subgraph "Engine-Sim Convention"
        EG["EngineSimGear: -1=Neutral, 0=1st, ..., 7=8th"]
    end
    BG <-->|"toEngineSim() / toBridge()"| EG
```

---

## Dependency Flow (Architecture Boundaries)

```mermaid
graph TD
    ES["engine-sim (upstream, unchanged)"]
    BRIDGE["engine-sim-bridge"]
    VS["vehicle-sim (standalone)"]
    CLI["CLI (composition root)"]
    IOS["escli-ios (composition root)"]

    BRIDGE -->|"depends on"| ES
    CLI -->|"depends on"| BRIDGE
    CLI -->|"depends on"| VS
    CLI -->|"VehicleSignalAdapter"| VS
    IOS -->|"depends on"| BRIDGE
    IOS -->|"depends on"| VS

    BRIDGE -.->|"no dependency"| VS
```

**Key constraint:** Neither bridge nor vehicle-sim depends on the other. Both are independent libraries. The composition root (CLI or iOS app) depends on both and writes the adapter.

---

## Demo Mode Data Flow

```mermaid
sequenceDiagram
    participant KB as Keyboard
    participant DVP as DemoVehiclePhysics
    participant VIP as VirtualIceInputProvider
    participant VIT as VirtualIceTwin
    participant SL as SimulationLoop
    participant SIM as BridgeSimulator
    participant ES as engine-sim

    loop Every 60Hz tick
        KB->>DVP: throttle 0-1
        DVP->>VIP: UpstreamSignal(throttle, speed, accel)
        VIP->>VIT: update(dt, UpstreamSignal)
        VIT->>VIP: TwinOutput(throttle, gear, clutch)
        VIP->>SL: EngineInput(throttle, gearAbsolute, clutchPressure)
        SL->>SIM: setThrottle, setGear, setClutchPressure
        SIM->>ES: physics steps
        ES->>SIM: EngineSimStats(RPM, gear, exhaust)
        SIM->>SL: getStats()
        SL->>VIP: provideFeedback(EngineSimStats)
        VIP->>VIT: setEngineRpmFeedback(rpm)
    end
```
