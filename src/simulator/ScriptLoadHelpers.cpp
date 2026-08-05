// ScriptLoadHelpers.cpp - Implementation of shared script loading helpers
// DRY: Shared helpers for engine simulation setup

#define DR_WAV_IMPLEMENTATION
#include "simulator/ScriptLoadHelpers.h"
#include "common/wav_loader.h"
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

        std::string filename = impulse->getFilename();
        if (filename.empty()) {
            continue;
        }

        // Construct full path deterministically.
        // The deserializer normalizes the filename to "sound-library/..." (no "es/" prefix).
        // resolveAssetBasePath returns the directory containing "sound-library/" on both
        // macOS (<root>/es/) and iOS (<bundle>/). No fallbacks needed.
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
