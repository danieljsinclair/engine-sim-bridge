#include "preset/PresetDeserializer.h"

#include "preset/EngineDeserializer.h"
#include "preset/VehicleDeserializer.h"
#include "preset/TransmissionDeserializer.h"

#include "common/JsonParser.h"

#include <string>

PresetLoadResult PresetDeserializer::deserialize(const json::JsonValue& root,
                                                   const std::string& sourceName) {
    PresetLoadResult result;

    // Preset name
    if (root.has("presetName")) {
        result.presetName = root["presetName"].asString();
    } else {
        size_t lastSlash = sourceName.find_last_of('/');
        size_t lastDot = sourceName.find_last_of('.');
        size_t start = (lastSlash != std::string::npos) ? lastSlash + 1 : 0;
        size_t end = (lastDot > start) ? lastDot : std::string::npos;
        result.presetName = sourceName.substr(start, end - start);
    }

    // Engine (required)
    if (!root.has("engine") || !root["engine"].isObject()) {
        result.error = "Missing or invalid 'engine' section in preset";
        return result;
    }

    result.engine = EngineDeserializer::deserialize(root["engine"], "engine");

    // Vehicle (optional)
    if (root.has("vehicle") && root["vehicle"].isObject()) {
        try {
            result.vehicle = VehicleDeserializer::deserialize(root["vehicle"], "vehicle");
        } catch (const std::exception& e) {
            result.error = std::string("Vehicle deserialization failed: ") + e.what();
            return result;
        }
    }

    // Transmission (optional)
    if (root.has("transmission") && root["transmission"].isObject()) {
        try {
            const json::JsonValue& transJson = root["transmission"];
            result.transmission = TransmissionDeserializer::deserialize(
                transJson, "transmission");

            // Read runtime state that must be applied after physics wiring
            // (changeGear requires m_vehicle, which is null at this point).
            // clutchPressure can be set immediately in the deserializer, but we
            // also carry it here so SimulatorFactory can re-apply if needed.
            if (transJson.has("currentGear")) {
                result.initialGear = transJson["currentGear"].asInt();
            }
            if (transJson.has("clutchPressure")) {
                result.initialClutchPressure = transJson["clutchPressure"].asNumber();
            }
        } catch (const std::exception& e) {
            result.error = std::string("Transmission deserialization failed: ") + e.what();
            return result;
        }
    }

    return result;
}
