// JsonParser.cpp - Minimal JSON parser implementation
// Recursive descent parser for preset JSON files

#include "common/JsonParser.h"

#include <fstream>
#include <sstream>
#include <cmath>

namespace json {

// Static null value for safe out-of-bounds access
JsonValue JsonValue::nullValue_;

// ============================================================================
// JsonValue implementation
// ============================================================================

const JsonValue& JsonValue::operator[](const std::string& key) const {
    if (type_ == JsonType::Object) {
        auto it = objVal_.find(key);
        if (it != objVal_.end()) return it->second;
    }
    return nullValue_;
}

const JsonValue& JsonValue::operator[](const char* key) const {
    return (*this)[std::string(key)];
}

bool JsonValue::has(const std::string& key) const {
    if (type_ == JsonType::Object) {
        return objVal_.find(key) != objVal_.end();
    }
    return false;
}

bool JsonValue::has(const char* key) const {
    return has(std::string(key));
}

const JsonValue& JsonValue::operator[](size_t index) const {
    if (type_ == JsonType::Array && index < arrVal_.size()) {
        return arrVal_[index];
    }
    return nullValue_;
}

size_t JsonValue::size() const {
    if (type_ == JsonType::Array) return arrVal_.size();
    if (type_ == JsonType::Object) return objVal_.size();
    return 0;
}

JsonValue JsonValue::makeNull() { JsonValue v; v.type_ = JsonType::Null; return v; }
JsonValue JsonValue::makeBool(bool v) { JsonValue r; r.type_ = JsonType::Bool; r.boolVal_ = v; return r; }
JsonValue JsonValue::makeNumber(double v) { JsonValue r; r.type_ = JsonType::Number; r.numVal_ = v; return r; }
JsonValue JsonValue::makeString(const std::string& v) { JsonValue r; r.type_ = JsonType::String; r.strVal_ = v; return r; }
JsonValue JsonValue::makeArray() { JsonValue r; r.type_ = JsonType::Array; return r; }
JsonValue JsonValue::makeObject() { JsonValue r; r.type_ = JsonType::Object; return r; }

void JsonValue::pushBack(JsonValue v) {
    if (type_ == JsonType::Array) {
        arrVal_.push_back(std::move(v));
    }
}

// ============================================================================
// Recursive Descent Parser
// ============================================================================

class Parser {
public:
    Parser(const std::string& input) : input_(input), pos_(0) {}

    JsonValue parse() {
        skipWhitespace();
        JsonValue result = parseValue();
        skipWhitespace();
        return result;
    }

private:
    const std::string& input_;
    size_t pos_;

    char peek() {
        return pos_ < input_.size() ? input_[pos_] : '\0';
    }

    char advance() {
        return pos_ < input_.size() ? input_[pos_++] : '\0';
    }

    void expect(char c) {
        skipWhitespace();
        if (peek() != c) {
            throw std::runtime_error("Expected '" + std::string(1, c) +
                "' at position " + std::to_string(pos_) + ", got '" +
                std::string(1, peek()) + "'");
        }
        advance();
    }

    void skipWhitespace() {
        while (pos_ < input_.size() &&
               (input_[pos_] == ' ' || input_[pos_] == '\t' ||
                input_[pos_] == '\n' || input_[pos_] == '\r')) {
            pos_++;
        }
    }

    JsonValue parseValue() {
        skipWhitespace();
        char c = peek();
        if (c == '{') return parseObject();
        if (c == '[') return parseArray();
        if (c == '"') return parseString();
        if (c == 't' || c == 'f') return parseBool();
        if (c == 'n') return parseNull();
        if (c == '-' || (c >= '0' && c <= '9')) return parseNumber();
        throw std::runtime_error("Unexpected character '" + std::string(1, c) +
            "' at position " + std::to_string(pos_));
    }

    JsonValue parseObject() {
        JsonValue result = JsonValue::makeObject();
        expect('{');
        skipWhitespace();

        if (peek() != '}') {
            // Parse first key-value pair
            parseKeyValuePair(result);
            skipWhitespace();

            while (peek() == ',') {
                advance();
                skipWhitespace();
                parseKeyValuePair(result);
                skipWhitespace();
            }
        }

        expect('}');
        return result;
    }

    void parseKeyValuePair(JsonValue& obj) {
        skipWhitespace();
        JsonValue keyVal = parseString();
        skipWhitespace();
        expect(':');
        skipWhitespace();
        JsonValue value = parseValue();
        obj.objectRef()[keyVal.asString()] = std::move(value);
    }

    JsonValue parseArray() {
        JsonValue result = JsonValue::makeArray();
        expect('[');
        skipWhitespace();

        if (peek() != ']') {
            result.pushBack(parseValue());
            skipWhitespace();

            while (peek() == ',') {
                advance();
                skipWhitespace();
                result.pushBack(parseValue());
                skipWhitespace();
            }
        }

        expect(']');
        return result;
    }

    JsonValue parseString() {
        expect('"');
        std::string result;
        while (pos_ < input_.size() && input_[pos_] != '"') {
            if (input_[pos_] == '\\') {
                pos_++;
                if (pos_ >= input_.size()) throw std::runtime_error("Unterminated string escape");
                switch (input_[pos_]) {
                    case '"':  result += '"'; break;
                    case '\\': result += '\\'; break;
                    case '/':  result += '/'; break;
                    case 'b':  result += '\b'; break;
                    case 'f':  result += '\f'; break;
                    case 'n':  result += '\n'; break;
                    case 'r':  result += '\r'; break;
                    case 't':  result += '\t'; break;
                    case 'u': {
                        // Skip 4 hex digits (basic Unicode escape)
                        pos_++;
                        // Just skip \uXXXX for now -- preset strings are ASCII
                        for (int i = 0; i < 4 && pos_ < input_.size(); i++) pos_++;
                        result += '?'; // Placeholder
                        break;
                    }
                    default:
                        result += input_[pos_];
                        break;
                }
            } else {
                result += input_[pos_];
            }
            pos_++;
        }
        expect('"');
        return JsonValue::makeString(result);
    }

    JsonValue parseNumber() {
        size_t start = pos_;
        if (peek() == '-') pos_++;
        while (pos_ < input_.size() && input_[pos_] >= '0' && input_[pos_] <= '9') pos_++;
        if (pos_ < input_.size() && input_[pos_] == '.') {
            pos_++;
            while (pos_ < input_.size() && input_[pos_] >= '0' && input_[pos_] <= '9') pos_++;
        }
        if (pos_ < input_.size() && (input_[pos_] == 'e' || input_[pos_] == 'E')) {
            pos_++;
            if (pos_ < input_.size() && (input_[pos_] == '+' || input_[pos_] == '-')) pos_++;
            while (pos_ < input_.size() && input_[pos_] >= '0' && input_[pos_] <= '9') pos_++;
        }

        std::string numStr = input_.substr(start, pos_ - start);
        double value = std::strtod(numStr.c_str(), nullptr);
        return JsonValue::makeNumber(value);
    }

    JsonValue parseBool() {
        if (input_.compare(pos_, 4, "true") == 0) {
            pos_ += 4;
            return JsonValue::makeBool(true);
        }
        if (input_.compare(pos_, 5, "false") == 0) {
            pos_ += 5;
            return JsonValue::makeBool(false);
        }
        throw std::runtime_error("Invalid boolean at position " + std::to_string(pos_));
    }

    JsonValue parseNull() {
        if (input_.compare(pos_, 4, "null") == 0) {
            pos_ += 4;
            return JsonValue::makeNull();
        }
        throw std::runtime_error("Invalid null at position " + std::to_string(pos_));
    }
};

// ============================================================================
// Public API
// ============================================================================

JsonValue parse(const std::string& json) {
    Parser p(json);
    return p.parse();
}

JsonValue parseFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open JSON file: " + path);
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return parse(ss.str());
}

} // namespace json
