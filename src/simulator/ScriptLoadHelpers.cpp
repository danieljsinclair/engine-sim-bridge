// ScriptLoadHelpers.cpp - Implementation of shared script loading helpers
// DRY: Shared helpers for engine simulation setup

// NOTE: DR_WAV_IMPLEMENTATION is intentionally NOT defined here. The single
// definition of WavLoader and the dr_wav C functions lives in
// engine-sim/src/wav_loader.cpp. The bridge links engine-sim, so WavLoader::load
// in this file resolves to that definition at link time. Defining
// DR_WAV_IMPLEMENTATION here too would duplicate the dr_wav symbols.
//
// This includes engine-sim's wav_loader.h DIRECTLY (it is on the bridge target's
// PUBLIC include path). A bridge-local "declaration-only mirror" of the same
// class used to live at common/wav_loader.h; it was deleted because it declared
// WavLoader::Result WITHOUT the `channels` field while the linked definition had
// it. Two layouts (32 vs 40 bytes) for one class is an ODR violation: this TU
// read `valid` at the wrong offset and destroyed the returned std::string
// through a shifted `this`, corrupting the heap. One header = one layout.
#include "simulator/ScriptLoadHelpers.h"
#include "common/PathNormalizer.h"
#include "wav_loader.h"
#include "common/PresetExceptions.h"  // SimulatorException (fail-fast on missing asset)

namespace ScriptLoadHelpers {

/**
 * Build the full path for a relative filename against an asset base path.
 * Handles absolute paths, and detects when the filename already starts with
 * the last path component of the asset base (e.g. "Presets/sound-library/...")
 * to avoid double-prefixing.
 */
static std::string buildFullPath(const std::string& assetBasePath, const std::string& filename) {
    if (filename[0] == '/' || (filename.length() > 1 && filename[1] == ':')) {
        return filename;
    }
    if (size_t firstSlash = filename.find('/'); firstSlash != std::string::npos) {
        size_t lastSlash = assetBasePath.find_last_of('/');
        std::string lastComponent = assetBasePath.substr(lastSlash + 1);
        if (filename.find(lastComponent + "/") == 0) {
            return filename;
        }
    }
    return assetBasePath + "/" + filename;
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

        // Construct full path deterministically.
        // For ABSOLUTE baked temp paths (e.g. /var/folders/.../T/sound-library/...),
        // normalizeImpulseResponsePath anchors on "sound-library/" and strips the temp
        // prefix, yielding the portable "sound-library/..." form. RELATIVE paths
        // (e.g. "../../es/sound-library/X.wav") are passed through raw — buildFullPath
        // resolves them correctly against assetBase (engine-sim/assets/../../es/...).
        // normalize is NOT applied unconditionally because it would collapse the leading
        // "../.." and strip "es/", regressing legitimate relative IR paths.
        std::string fullPath = buildFullPath(assetBasePath, filename);

        WavLoader::Result wavResult = WavLoader::load(fullPath);

        if (!wavResult.valid) {
            // Fail-fast: a missing impulse response means the run would produce
            // no exhaust audio at all, so we must NOT silently continue. Throw
            // (not return false) so the top-level SimulatorException handler names
            // the missing path + asset base and the process exits non-zero.
            // Distinguish "the file is absent" from "the file is present but
            // unreadable/corrupt": both are fatal, but they need different fixes
            // (wrong asset base vs. a bad WAV), and saying which saves a guess.
            std::error_code ec;
            const bool present = std::filesystem::exists(fullPath, ec) && !ec;
            if (logger) {
                logger->error(LogMask::ASSET, __ilog_format(
                    present ? "Required audio file is present but could not be decoded: %s"
                            : "Required audio file is MISSING: %s",
                    fullPath.c_str()));
                logger->error(LogMask::ASSET, __ilog_format(
                    "(asset base: %s, from script: %s, exhaust system: %d)",
                    assetBasePath.c_str(), filename.c_str(), i));
                logger->error(LogMask::ASSET,
                    "Asset base must be the directory that directly contains 'sound-library/'"
                    " (e.g. <repo>/es). Run the engine from a tree with the WAVs present.");
            }
            throw SimulatorException(
                std::string(present ? "Required audio file present but unreadable: "
                                    : "Required audio file MISSING: ")
                + fullPath
                + " (asset base: " + assetBasePath
                + ", referenced from script as: " + filename + ")");
        }

        if (logger) {
            logger->info(LogMask::ASSET, __ilog_format("Loaded impulse response: %s (%zu samples)", fullPath.c_str(), wavResult.getSampleCount()));
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
