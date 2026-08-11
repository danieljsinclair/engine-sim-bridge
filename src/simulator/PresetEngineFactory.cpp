// PresetEngineFactory.cpp - Delegates to PresetDeserializer for JSON loading

#include "simulator/PresetEngineFactory.h"
#include "preset/PresetDeserializer.h"
#include "common/JsonParser.h"
#include "common/PresetExceptions.h"

#include "engine.h"
#include "exhaust_system.h"
#include "impulse_response.h"

#include <fstream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <vector>
#include <string>

using json::JsonValue;

// Minimal fnmatch: '*' matches any run of characters, '?' matches exactly one,
// all other characters match literally. Used to select afterfire pop WAVs from a
// glob whose leaf is a pattern such as "pop_*.wav".
static bool matchGlob(const std::string& pattern, const std::string& name) {
    const size_t p = pattern.size();
    const size_t n = name.size();
    // dp[j] = can the first i chars of pattern match the first j chars of name.
    std::vector<bool> dpPrev(p + 1, false);
    dpPrev[0] = true;
    for (size_t i = 1; i <= p; ++i) {
        if (pattern[i - 1] == '*' && dpPrev[i - 1]) dpPrev[i] = true;
        else break;
    }
    for (size_t j = 1; j <= n; ++j) {
        std::vector<bool> dpCurr(p + 1, false);
        for (size_t i = 1; i <= p; ++i) {
            const char c = pattern[i - 1];
            if (c == '*') {
                dpCurr[i] = dpPrev[i] || dpCurr[i - 1];
            }
            else if (c == '?' || c == name[j - 1]) {
                dpCurr[i] = dpPrev[i - 1];
            }
        }
        dpPrev.swap(dpCurr);
    }
    return dpPrev[p];
}

// Expand an afterfire WAV path into the list of candidate files. A literal path
// yields itself (if it exists); a path containing '*','?' or '[' is treated as a
// glob and every matching file in its directory is returned. Returns an empty
// list when nothing resolves, which tells the chamber to fall back to the
// engine's default exhaust impulse response.
// Static helper owned by PresetEngineFactory (it already centralises the
// filesystem + WAV-loading concerns for the bridge).
std::vector<std::string> resolveAfterfireWavPaths(const std::filesystem::path& rawPath) {
    std::vector<std::string> result;
    if (rawPath.empty()) return result;

    const std::string rawText = rawPath.string();
    const bool isGlob = rawText.find('*') != std::string::npos
                     || rawText.find('?') != std::string::npos
                     || rawText.find('[') != std::string::npos;

    const std::filesystem::path dir =
        rawPath.has_parent_path() ? rawPath.parent_path() : std::filesystem::path(".");
    const std::string leaf = rawPath.filename().string();

    if (!isGlob) {
        if (std::filesystem::exists(rawPath)) {
            result.push_back(std::filesystem::weakly_canonical(rawPath).string());
        }
        return result;
    }

    if (!std::filesystem::exists(dir) || !std::filesystem::is_directory(dir)) {
        return result;
    }
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;
        if (matchGlob(leaf, entry.path().filename().string())) {
            result.push_back(entry.path().string());
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

// Resolve relative impulse response filenames to absolute paths using the
// engine-sim root as the base directory. Paths like "../../es/sound-library/..."
// are CWD-relative from the engine-sim root (the CWD when the preset compiler
// ran). weakly_canonical resolves symlinks and normalizes the path.
static void resolveImpulseResponsePaths(const Engine* engine, const std::filesystem::path& assetBase) {
    if (!engine || assetBase.empty()) return;

    for (int i = 0; i < engine->getExhaustSystemCount(); i++) {
        const ExhaustSystem* es = engine->getExhaustSystem(i);
        if (!es) continue;

        ImpulseResponse* ir = es->getImpulseResponse();
        if (!ir) continue;

        const std::string& filename = ir->getFilename();
        if (filename.empty()) continue;

        std::filesystem::path p(filename);
        if (!p.is_absolute()) {
            std::filesystem::path resolved =
                std::filesystem::weakly_canonical(assetBase / p);
            ir->initialize(resolved.string(), ir->getVolume());
        }
    }
}

PresetLoadResult PresetEngineFactory::loadFromFile(const std::string& jsonPath,
                                                    const std::string& assetBasePath) {
    std::filesystem::path resolvedPath(jsonPath);
    if (!resolvedPath.is_absolute()) {
        resolvedPath = std::filesystem::absolute(resolvedPath);
    }
    std::ifstream file(resolvedPath);
    if (!file.is_open()) {
        PresetLoadResult result;
        result.error = "Cannot open preset file: " + resolvedPath.string();
        return result;
    }

    std::ostringstream ss;
    ss << file.rdbuf();
    return loadFromString(ss.str(), jsonPath, assetBasePath);
}

PresetLoadResult PresetEngineFactory::loadFromString(const std::string& jsonContent,
                                                     const std::string& sourceName,
                                                     const std::string& assetBasePath) {
    PresetLoadResult result;

    try {
        JsonValue root = json::parse(jsonContent);
        result = PresetDeserializer::deserialize(root, sourceName);
    } catch (const PresetDeserializationException& e) {
        result.error = std::string("JSON parse error: ") + e.what();
        return result;
    } catch (const PresetException& e) {
        result.error = std::string("Preset error: ") + e.what();
        return result;
    }

    // Resolve relative impulse response paths to absolute using the asset base.
    if (result.success() && !assetBasePath.empty()) {
        resolveImpulseResponsePaths(result.engine, std::filesystem::path(assetBasePath));
    }

    return result;
}

PresetLoadResult PresetEngineFactory::loadFromJson(const char* jsonContent, size_t jsonSize) {
    if (!jsonContent || jsonSize == 0) {
        PresetLoadResult result;
        result.error = "Null or empty JSON content";
        return result;
    }
    std::string content(jsonContent, jsonSize);
    return loadFromString(content, "<json>");
}
