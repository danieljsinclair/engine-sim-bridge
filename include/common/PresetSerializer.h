// PresetSerializer.h - Engine serialization to JSON
// Provides shared serialization functions used by preset_compiler and tests

#ifndef ENGINE_SIM_BRIDGE_PRESET_SERIALIZER_H
#define ENGINE_SIM_BRIDGE_PRESET_SERIALIZER_H

#include "common/JsonWriter.h"
#include <string>

// Forward declarations
class Engine;
class Vehicle;
class Transmission;
class Crankshaft;
class ConnectingRod;
class ExhaustSystem;
class Intake;
class Camshaft;
class CylinderHead;
class CylinderBank;
class CombustionChamber;

namespace PresetSerializer {

// Individual component serializers
void serializeCrankshaft(JsonWriter& j, Crankshaft* cs);
void serializeConnectingRod(JsonWriter& j, const ConnectingRod* rod);
void serializeExhaustSystem(JsonWriter& j, const ExhaustSystem* es);
void serializeIntake(JsonWriter& j, const Intake* intake);
void serializeCamshaft(JsonWriter& j, const Camshaft* cam, int cylinderCount);
void serializeCylinderHead(JsonWriter& j, CylinderHead* head, int cylinderCount);
void serializeCylinderBank(JsonWriter& j, const CylinderBank* bank, const Engine* engine);
void serializeCombustionChamber(JsonWriter& j, CombustionChamber* chamber);
void serializeEngine(JsonWriter& j, Engine* engine);
void serializeVehicle(JsonWriter& j, const Vehicle* vehicle);
void serializeTransmission(JsonWriter& j, const Transmission* trans);

// Convenience: serialize complete engine preset to JSON string
// Returns JSON string with engine, vehicle, and transmission (if provided)
std::string serializeEngineToJson(Engine* engine, const Vehicle* vehicle = nullptr, const Transmission* transmission = nullptr);

} // namespace PresetSerializer

#endif // ENGINE_SIM_BRIDGE_PRESET_SERIALIZER_H