// ESP32I2SHardwareProvider.h - ESP32 I2S implementation of IAudioHardwareProvider
// Outputs audio via I2S to external DAC (MAX98357A) on ESP32-S3
// OCP: ESP32-specific implementation behind IAudioHardwareProvider interface

#ifndef ESP32_I2S_HARDWARE_PROVIDER_H
#define ESP32_I2S_HARDWARE_PROVIDER_H

#ifdef ESP_PLATFORM

#include "hardware/IAudioHardwareProvider.h"
#include "common/ILogging.h"

#include <memory>
#include <driver/i2s_std.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class ESP32I2SHardwareProvider : public IAudioHardwareProvider {
public:
    explicit ESP32I2SHardwareProvider(ILogging* logger = nullptr);
    ~ESP32I2SHardwareProvider() override;

    bool initialize(const AudioStreamFormat& format) override;
    void cleanup() override;

    bool startPlayback() override;
    void stopPlayback() override;

    void setVolume(double volume) override;
    double getVolume() const override;

    bool registerAudioCallback(const AudioCallback& callback) override;

    AudioHardwareState getHardwareState() const override;
    void resetDiagnostics() override;

private:
    i2s_chan_handle_t txHandle;
    bool initialized_;
    bool playing_;
    double volume_;
    int underrunCount_;
    int overrunCount_;

    AudioCallback callback_;
    AudioStreamFormat currentFormat_;

    ILogging* logger_;

    // I2S writer task handle
    TaskHandle_t writerTask_;
    static constexpr int WRITER_STACK_SIZE = 4096;
    static constexpr int WRITER_PRIORITY = configMAX_PRIORITIES - 1;

    static void writerTaskFunc(void* arg);
};

#endif // ESP_PLATFORM
#endif // ESP32_I2S_HARDWARE_PROVIDER_H
