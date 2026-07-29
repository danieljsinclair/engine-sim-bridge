// PathNormalizer.h - Portable relative path normalization for impulse responses
//
// Piranha stores CWD-relative paths like "../../es/sound-library/X.wav".
// The portable form is "sound-library/X.wav" — no ".." components, no "es/" prefix.
// This normalization runs at both deserialization (entry) and serialization (output)
// to ensure both paths produce identical filenames.

#ifndef ENGINE_SIM_BRIDGE_PATH_NORMALIZER_H
#define ENGINE_SIM_BRIDGE_PATH_NORMALIZER_H

#include <filesystem>
#include <string>

namespace PathNormalizer {

inline std::string normalizeImpulseResponsePath(const std::string& filename) {
    if (filename.empty()) return filename;

    std::filesystem::path p(filename);

    if (p.is_absolute()) {
        bool foundAnchor = false;
        std::filesystem::path result;
        for (const auto& component : p) {
            if (component == "sound-library") {
                foundAnchor = true;
            }
            if (foundAnchor) {
                result /= component;
            }
        }
        if (foundAnchor && !result.empty()) {
            return result.string();
        }
        return filename;
    }

    std::filesystem::path result;
    for (const auto& component : p) {
        if (component == "..") {
            result = result.parent_path();
        } else if (component != ".") {
            result /= component;
        }
    }

    std::string s = result.string();
    if (s.size() > 3 && s.substr(0, 3) == "es/") {
        s = s.substr(3);
    }

    return s;
}

}  // namespace PathNormalizer

#endif  // ENGINE_SIM_BRIDGE_PATH_NORMALIZER_H
