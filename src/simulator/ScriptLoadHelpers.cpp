// ScriptLoadHelpers.cpp - Implementation of shared script loading helpers
// DRY: Used by both engine_sim_bridge.cpp (C API) and BridgeSimulator (C++ API)

#include "simulator/ScriptLoadHelpers.h"
#include "common/wav_loader.h"  // Declarations only; DR_WAV_IMPLEMENTATION is in engine_sim_bridge.cpp

namespace ScriptLoadHelpers {

bool loadImpulseResponses(
    Simulator* simulator,
    Engine* engine,
    const std::string& assetBasePath,
    ILogging* logger)
{
    if (!engine) {
        return false;
    }

    const int exhaustCount = engine->getExhaustSystemCount();
    for (int i = 0; i < exhaustCount; ++i) {
        ExhaustSystem* exhaust = engine->getExhaustSystem(i);
        if (!exhaust) continue;

        ImpulseResponse* impulse = exhaust->getImpulseResponse();
        if (!impulse) continue;

        std::string filename = impulse->getFilename();
        if (filename.empty()) {
            continue;
        }

        // Construct full path
        std::string fullPath;
        if (filename[0] == '/' || (filename.length() > 1 && filename[1] == ':')) {
            fullPath = filename;
        } else {
            size_t firstSlash = filename.find('/');
            if (firstSlash != std::string::npos) {
                size_t lastSlash = assetBasePath.find_last_of('/');
                std::string lastComponent = assetBasePath.substr(lastSlash + 1);
                if (filename.find(lastComponent + "/") == 0) {
                    fullPath = filename;
                } else {
                    fullPath = assetBasePath + "/" + filename;
                }
            } else {
                fullPath = assetBasePath + "/" + filename;
            }
        }

        WavLoader::Result wavResult = WavLoader::load(fullPath);
        if (!wavResult.valid) {
            if (logger) {
                logger->error(LogMask::ASSET, "Failed to load required audio file: %s", fullPath.c_str());
                logger->error(LogMask::ASSET, "(asset base: %s, from script: %s)", assetBasePath.c_str(), filename.c_str());
            }
            return false;
        }

        if (logger) {
            logger->info(LogMask::ASSET, "Loaded impulse response: %s (%zu samples)", fullPath.c_str(), wavResult.getSampleCount());
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
