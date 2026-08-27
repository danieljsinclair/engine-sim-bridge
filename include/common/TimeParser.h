#pragma once
#include <string>
#include <vector>
#include <sstream>

namespace engine_sim_bridge {

// Parse a time string (plain seconds "30.5", mm:ss "1:30.5", or hh:mm:ss "0:01:30.5")
// into seconds (as double). Returns -1.0 on invalid input.
inline double parseTimecodeToSeconds(const std::string& s) {
    if (s.empty()) return -1.0;
    if (s.front() == ':' || s.back() == ':') return -1.0;

    std::vector<std::string> parts;
    std::stringstream ss(s);
    std::string part;
    while (std::getline(ss, part, ':')) {
        parts.push_back(part);
    }

    try {
        if (parts.size() == 1) {
            return std::stod(parts[0]);
        }
        if (parts.size() == 2) {
            return std::stod(parts[0]) * 60.0 + std::stod(parts[1]);
        }
        if (parts.size() == 3) {
            return std::stod(parts[0]) * 3600.0
                 + std::stod(parts[1]) * 60.0
                 + std::stod(parts[2]);
        }
    } catch (const std::invalid_argument&) {
        return -1.0;
    } catch (const std::out_of_range&) {
        return -1.0;
    }
    return -1.0;
}

} // namespace engine_sim_bridge
