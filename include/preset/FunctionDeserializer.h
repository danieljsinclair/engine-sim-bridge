#ifndef FUNCTION_DESERIALIZER_H
#define FUNCTION_DESERIALIZER_H

#include "common/JsonParser.h"
#include "function.h"

#include <string>

class FunctionDeserializer {
public:
    static Function* deserialize(const json::JsonValue& fnJson, const std::string& context = "");

private:
    static std::string fieldError(const std::string& field, const std::string& context);
};

#endif // FUNCTION_DESERIALIZER_H
