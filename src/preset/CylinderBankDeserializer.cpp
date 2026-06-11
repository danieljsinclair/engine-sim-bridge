#include "preset/CylinderBankDeserializer.h"

#include "cylinder_bank.h"
#include "piston.h"
#include "connecting_rod.h"
#include "preset/ConnectingRodDeserializer.h"
#include "common/PresetExceptions.h"

using json::JsonValue;

void CylinderBankDeserializer::deserialize(const JsonValue& json, CylinderBank* bank,
                                            Crankshaft* crankshaft, int bankIndex,
                                            const std::string& context) {
    const std::string ctx = context.empty() ? "cylinderBank" : context;

    if (!json.has("cylinderCount")) {
        throw PresetDeserializationException("Missing required field 'cylinderCount' in " + ctx);
    }
    int cylCount = json["cylinderCount"].asInt();

    CylinderBank::Parameters params;
    params.crankshaft = crankshaft;

    if (!json.has("angle")) {
        throw PresetDeserializationException("Missing required field 'angle' in " + ctx);
    }
    params.angle = json["angle"].asNumber();

    if (!json.has("bore")) {
        throw PresetDeserializationException("Missing required field 'bore' in " + ctx);
    }
    params.bore = json["bore"].asNumber();

    if (!json.has("deckHeight")) {
        throw PresetDeserializationException("Missing required field 'deckHeight' in " + ctx);
    }
    params.deckHeight = json["deckHeight"].asNumber();

    if (!json.has("displayDepth")) {
        throw PresetDeserializationException("Missing required field 'displayDepth' in " + ctx);
    }
    params.displayDepth = json["displayDepth"].asNumber();

    if (!json.has("index")) {
        throw PresetDeserializationException("Missing required field 'index' in " + ctx);
    }
    params.index = json["index"].asInt();

    params.cylinderCount = cylCount;

    if (!json.has("positionX")) {
        throw PresetDeserializationException("Missing required field 'positionX' in " + ctx);
    }
    params.positionX = json["positionX"].asNumber();

    if (!json.has("positionY")) {
        throw PresetDeserializationException("Missing required field 'positionY' in " + ctx);
    }
    params.positionY = json["positionY"].asNumber();

    bank->initialize(params);
}
