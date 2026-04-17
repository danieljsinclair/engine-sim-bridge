// JsonParser.h - Minimal JSON parser for preset files
// Recursive descent parser -- no external dependencies
// Supports: objects, arrays, strings, numbers (int/double), booleans, null
//
// Limitations (acceptable for preset files):
// - No Unicode escape handling beyond \uXXXX basic support
// - Numbers use strtod (sufficient for our SI-unit doubles)
// - No streaming -- loads entire file into memory

#ifndef JSON_PARSER_H
#define JSON_PARSER_H

#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <cstring>
#include <cstdlib>

namespace json {

class JsonValue;

// JSON value types
enum class JsonType {
    Null,
    Bool,
    Number,
    String,
    Array,
    Object
};

// JsonValue - Represents a JSON value (variant type)
class JsonValue {
public:
    using Object = std::map<std::string, JsonValue>;
    using Array = std::vector<JsonValue>;

    JsonValue() : type_(JsonType::Null), boolVal_(false), numVal_(0) {}
    ~JsonValue() = default;

    // Type queries
    JsonType type() const { return type_; }
    bool isNull() const { return type_ == JsonType::Null; }
    bool isBool() const { return type_ == JsonType::Bool; }
    bool isNumber() const { return type_ == JsonType::Number; }
    bool isString() const { return type_ == JsonType::String; }
    bool isArray() const { return type_ == JsonType::Array; }
    bool isObject() const { return type_ == JsonType::Object; }

    // Value accessors (undefined behavior if wrong type -- caller must check)
    bool asBool() const { return boolVal_; }
    double asNumber() const { return numVal_; }
    int asInt() const { return static_cast<int>(numVal_); }
    const std::string& asString() const { return strVal_; }
    const Array& asArray() const { return arrVal_; }
    const Object& asObject() const { return objVal_; }

    // Convenience: get value with default
    double numberOr(double def) const { return isNumber() ? numVal_ : def; }
    int intOr(int def) const { return isNumber() ? static_cast<int>(numVal_) : def; }
    const std::string& stringOr(const std::string& def) const { return isString() ? strVal_ : def; }
    bool boolOr(bool def) const { return isBool() ? boolVal_ : def; }

    // Object key access (returns reference to static null value if not found)
    const JsonValue& operator[](const std::string& key) const;
    const JsonValue& operator[](const char* key) const;
    bool has(const std::string& key) const;
    bool has(const char* key) const;

    // Array index access
    const JsonValue& operator[](size_t index) const;
    size_t size() const;

    // Factory methods for building values
    static JsonValue makeNull();
    static JsonValue makeBool(bool v);
    static JsonValue makeNumber(double v);
    static JsonValue makeString(const std::string& v);
    static JsonValue makeArray();
    static JsonValue makeObject();

    // Mutable access for building
    Array& arrayRef() { return arrVal_; }
    Object& objectRef() { return objVal_; }
    void pushBack(JsonValue v);

private:
    JsonType type_;
    bool boolVal_;
    double numVal_;
    std::string strVal_;
    Array arrVal_;
    Object objVal_;

    static JsonValue nullValue_;
};

// Parse JSON string into JsonValue tree
// Throws std::runtime_error on parse errors
JsonValue parse(const std::string& json);

// Parse JSON file
JsonValue parseFile(const std::string& path);

} // namespace json

#endif // JSON_PARSER_H
