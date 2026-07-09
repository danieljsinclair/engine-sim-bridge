#ifndef PRESET_EXCEPTIONS_H
#define PRESET_EXCEPTIONS_H

#include <stdexcept>
#include <string>

// Base exception for all preset-related errors
class PresetException : public std::runtime_error {
public:
    explicit PresetException(const std::string& message)
        : std::runtime_error(message) {}
};

// Thrown when JSON deserialization fails (missing fields, wrong types, etc.)
class PresetDeserializationException : public PresetException {
public:
    explicit PresetDeserializationException(const std::string& message)
        : PresetException(message) {}
};

// Thrown when preset data fails validation (invalid values, inconsistent state, etc.)
class PresetValidationException : public PresetException {
public:
    explicit PresetValidationException(const std::string& message)
        : PresetException(message) {}
};

// Thrown for simulator-level errors (initialization, runtime, configuration)
class SimulatorException : public std::runtime_error {
public:
    explicit SimulatorException(const std::string& message)
        : std::runtime_error(message) {}
};

#endif // PRESET_EXCEPTIONS_H
