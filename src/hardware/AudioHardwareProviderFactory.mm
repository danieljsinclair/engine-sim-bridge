// AudioHardwareProviderFactory.mm - iOS factory for audio hardware providers
// Compiled as ObjC++ so it can include AVAudioEngine headers
// OCP: New platforms can be added by extending this factory

#include "hardware/IAudioHardwareProvider.h"
#include "hardware/AVAudioEngineHardwareProvider.h"
#include "common/ILogging.h"

#include <memory>

std::unique_ptr<IAudioHardwareProvider> AudioHardwareProviderFactory::createProvider(ILogging* logger) {
    return std::make_unique<AVAudioEngineHardwareProvider>(logger);
}
