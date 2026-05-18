#include "preset/CylinderBankDeserializer.h"

#include "cylinder_bank.h"
#include "piston.h"
#include "connecting_rod.h"
#include "preset/ConnectingRodDeserializer.h"

#include <stdexcept>

using json::JsonValue;

void CylinderBankDeserializer::deserialize(const JsonValue& json, CylinderBank* bank,
                                            Crankshaft* crankshaft, int bankIndex,
                                            const std::string& context) {
    const std::string ctx = context.empty() ? "cylinderBank" : context;

    int cylCount = json["cylinderCount"].intOr(0);

    CylinderBank::Parameters params;
    params.crankshaft = crankshaft;
    params.angle = json["angle"].numberOr(0);
    params.bore = json["bore"].numberOr(0);
    params.deckHeight = json["deckHeight"].numberOr(0);
    params.displayDepth = json["displayDepth"].numberOr(0);
    params.index = json["index"].intOr(bankIndex);
    params.cylinderCount = cylCount;
    params.positionX = json["positionX"].numberOr(0);
    params.positionY = json["positionY"].numberOr(0);

    bank->initialize(params);
}
