// ScriptLoadHelpers.cpp - Implementation of shared script loading helpers
// DRY: Shared helpers for engine simulation setup

#define DR_WAV_IMPLEMENTATION
#include "simulator/ScriptLoadHelpers.h"
#include "common/AssetResolver.h"
#include "common/PathNormalizer.h"
#include "common/wav_loader.h"

#include <algorithm>

namespace ScriptLoadHelpers {

namespace {

bool isAbsolutePath(const std::string& path) {
    return !path.empty() && (path[0] == '/' || (path.length() > 1 && path[1] == ':'));
}

std::string joinPath(const std::string& dir, const std::string& rel) {
    if (dir.empty()) return rel;
    if (dir.back() == '/') return dir + rel;
    return dir + "/" + rel;
}

// Reduce a runtime audio reference to the portable "sound-library/..." form
// when it references the sound library. Piranha stores the reference relative
// to the importing unit's directory, so refs arrive as
// "<family>/sound-library/<sub>/<file>.wav" (e.g. "es/sound-library/smooth/
// smooth_39.wav") regardless of which family directory the entry script
// itself lives in. References without a sound-library component pass through
// unchanged.
std::string soundLibraryAnchored(const std::string& filename) {
    std::filesystem::path anchored;
    bool found = false;
    for (const auto& component : std::filesystem::path(filename)) {
        if (component == "sound-library") found = true;
        if (found) anchored /= component;
    }
    return found ? anchored.generic_string() : filename;
}

} // namespace

std::vector<std::string> audioFileCandidates(
    const std::string& assetBasePath,
    const std::string& filename)
{
    std::vector<std::string> candidates;
    auto add = [&candidates](const std::string& path) {
        if (path.empty()) return;
        if (std::find(candidates.begin(), candidates.end(), path) == candidates.end()) {
            candidates.push_back(path);
        }
    };

    if (isAbsolutePath(filename)) {
        add(filename);
        return candidates;
    }

    const std::string anchored = soundLibraryAnchored(filename);

    // 1. The asset base derived from the script's own location: the script
    //    family's library first (portable anchored form), then the reference
    //    as written. For bare library-relative refs (no sound-library
    //    component) also probe base/sound-library/<ref>.
    if (!assetBasePath.empty()) {
        add(joinPath(assetBasePath, anchored));
        add(joinPath(assetBasePath, filename));
        if (anchored == filename) {
            add(joinPath(joinPath(assetBasePath, "sound-library"), filename));
        }
    }

    // 2. Exe-aware search roots (exe dir -> repo root -> cwd). References are
    //    engine-sim-root relative as written ("es/sound-library/..."), so try
    //    the reference as written first, then the anchored form.
    for (const auto& root : asset_resolver::searchRoots()) {
        add((root / filename).generic_string());
        add((root / anchored).generic_string());
    }

    return candidates;
}

bool loadImpulseResponses(
    Simulator* simulator,
    const Engine* engine,
    const std::string& assetBasePath,
    ILogging* logger)
{
    if (!engine) {
        return false;
    }

    const int exhaustCount = engine->getExhaustSystemCount();
    for (int i = 0; i < exhaustCount; ++i) {
        const ExhaustSystem* exhaust = engine->getExhaustSystem(i);
        if (!exhaust) continue;

        const ImpulseResponse* impulse = exhaust->getImpulseResponse();
        if (!impulse) continue;

        std::string rawFn = impulse->getFilename();
        bool isAbsolute = (!rawFn.empty() && (rawFn[0] == '/' || (rawFn.length() > 1 && rawFn[1] == ':')));
        std::string filename = isAbsolute ? PathNormalizer::normalizeImpulseResponsePath(rawFn) : rawFn;
        if (filename.empty()) {
            continue;
        }

        // Construct the full path by probing candidates in priority order.
        // For ABSOLUTE baked temp paths (e.g. /var/folders/.../T/sound-library/...),
        // normalizeImpulseResponsePath anchors on "sound-library/" and strips the temp
        // prefix, yielding the portable "sound-library/..." form. RELATIVE paths
        // (e.g. "../../es/sound-library/X.wav" or "es/sound-library/X.wav") are
        // resolved by audioFileCandidates: anchored against the script family's
        // asset base first, then the exe-aware root chain (exe dir -> repo root
        // -> cwd) — never CWD-only.
        // normalize is NOT applied unconditionally because it would collapse the leading
        // "../.." and strip "es/", regressing legitimate relative IR paths.
        const std::vector<std::string> candidates = audioFileCandidates(assetBasePath, filename);

        WavLoader::Result wavResult;
        std::string loadedPath;
        for (const std::string& candidate : candidates) {
            wavResult = WavLoader::load(candidate);
            if (wavResult.valid) {
                loadedPath = candidate;
                break;
            }
        }

        if (!wavResult.valid) {
            if (logger) {
                const std::string& primary = candidates.empty() ? filename : candidates.front();
                logger->error(LogMask::ASSET, __ilog_format("Failed to load required audio file: %s", primary.c_str()));
                logger->error(LogMask::ASSET, __ilog_format("(asset base: %s, from script: %s)", assetBasePath.c_str(), filename.c_str()));
                std::string tried;
                for (const auto& candidate : candidates) {
                    if (!tried.empty()) tried += " | ";
                    tried += candidate;
                }
                logger->error(LogMask::ASSET, __ilog_format("Candidates tried: %s", tried.c_str()));
            }
            return false;
        }

        if (logger) {
            logger->info(LogMask::ASSET, __ilog_format("Loaded impulse response: %s (%zu samples)", loadedPath.c_str(), wavResult.getSampleCount()));
        }

        simulator->synthesizer().initializeImpulseResponse(
            wavResult.getData(),
            static_cast<unsigned int>(wavResult.getSampleCount()),
            static_cast<float>(impulse->getVolume()),
            i
        );
    }

    return true;
}

} // namespace ScriptLoadHelpers
