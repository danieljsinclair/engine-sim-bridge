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

// Resolve an impulse response's runtime filename to the form probed by
// audioFileCandidates. For ABSOLUTE baked temp paths (e.g.
// /var/folders/.../T/sound-library/...), normalizeImpulseResponsePath anchors
// on "sound-library/" and strips the temp prefix, yielding the portable
// "sound-library/..." form. RELATIVE paths (e.g.
// "../../es/sound-library/X.wav" or "es/sound-library/X.wav") pass through
// as written. normalize is NOT applied to relative paths because it would
// collapse the leading "../.." and strip "es/", regressing legitimate
// relative IR paths. Returns "" when the reference carries no filename.
std::string impulseFilename(const ImpulseResponse* impulse) {
    const std::string rawFilename = impulse->getFilename();
    if (rawFilename.empty()) {
        return rawFilename;
    }
    return isAbsolutePath(rawFilename)
        ? PathNormalizer::normalizeImpulseResponsePath(rawFilename)
        : rawFilename;
}

// Probe candidates in priority order and return the first that loads as a
// WAV ("" when none does). outResult always holds the last load attempt, so
// a miss leaves it invalid — mirroring the plain scan this replaces.
std::string firstLoadableCandidate(
    const std::vector<std::string>& candidates,
    WavLoader::Result& outResult)
{
    for (const std::string& candidate : candidates) {
        outResult = WavLoader::load(candidate);
        if (outResult.valid) {
            return candidate;
        }
    }
    return "";
}

// " | "-joined candidate list for the diagnostics line.
std::string joinCandidates(const std::vector<std::string>& candidates) {
    std::string joined;
    for (const std::string& candidate : candidates) {
        if (!joined.empty()) joined += " | ";
        joined += candidate;
    }
    return joined;
}

// Fail-fast diagnostics for an unloadable impulse response: the primary
// candidate, the asset base with the reference as written, and every
// candidate tried.
void logImpulseLoadFailure(
    ILogging* logger,
    const std::vector<std::string>& candidates,
    const std::string& assetBasePath,
    const std::string& filename)
{
    if (logger == nullptr) {
        return;
    }

    const std::string& primary = candidates.empty() ? filename : candidates.front();
    logger->error(LogMask::ASSET, __ilog_format("Failed to load required audio file: %s", primary.c_str()));
    logger->error(LogMask::ASSET, __ilog_format("(asset base: %s, from script: %s)", assetBasePath.c_str(), filename.c_str()));
    logger->error(LogMask::ASSET, __ilog_format("Candidates tried: %s", joinCandidates(candidates).c_str()));
}

// Load one exhaust system's impulse response into its synthesizer channel.
// Exhaust systems without an impulse response (or without a filename) are
// skips, not failures.
bool loadExhaustImpulseResponse(
    Simulator* simulator,
    const ExhaustSystem* exhaust,
    int channel,
    const std::string& assetBasePath,
    ILogging* logger)
{
    const ImpulseResponse* impulse = exhaust->getImpulseResponse();
    if (impulse == nullptr) {
        return true;
    }

    const std::string filename = impulseFilename(impulse);
    if (filename.empty()) {
        return true;
    }

    // Full-path resolution probes candidates in priority order: anchored
    // against the script family's asset base first, then the exe-aware root
    // chain (exe dir -> repo root -> cwd) — never CWD-only.
    const std::vector<std::string> candidates = audioFileCandidates(assetBasePath, filename);

    WavLoader::Result wavResult;
    const std::string loadedPath = firstLoadableCandidate(candidates, wavResult);

    if (!wavResult.valid) {
        logImpulseLoadFailure(logger, candidates, assetBasePath, filename);
        return false;
    }

    if (logger != nullptr) {
        logger->info(LogMask::ASSET, __ilog_format("Loaded impulse response: %s (%zu samples)", loadedPath.c_str(), wavResult.getSampleCount()));
    }

    simulator->synthesizer().initializeImpulseResponse(
        wavResult.getData(),
        static_cast<unsigned int>(wavResult.getSampleCount()),
        static_cast<float>(impulse->getVolume()),
        channel
    );
    return true;
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
    if (engine == nullptr) {
        return false;
    }

    const int exhaustCount = engine->getExhaustSystemCount();
    for (int i = 0; i < exhaustCount; ++i) {
        const ExhaustSystem* exhaust = engine->getExhaustSystem(i);
        if (exhaust == nullptr) {
            continue;
        }
        if (!loadExhaustImpulseResponse(simulator, exhaust, i, assetBasePath, logger)) {
            return false;
        }
    }

    return true;
}

} // namespace ScriptLoadHelpers
