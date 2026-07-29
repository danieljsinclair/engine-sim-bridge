// ESP32I2SHardwareProvider.cpp - ESP32 I2S audio output via MAX98357A
// Uses ESP-IDF I2S standard mode driver for 16-bit stereo output

#ifdef ESP_PLATFORM

#include "hardware/ESP32I2SHardwareProvider.h"
#include "hardware/AudioTypes.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <cstring>

static const char* TAG = "I2SProvider";

// GPIO pin assignment (MAX98357A breakout)
static constexpr gpio_num_t I2S_BCLK = GPIO_NUM_4;
static constexpr gpio_num_t I2S_LRCLK = GPIO_NUM_5;
static constexpr gpio_num_t I2S_DIN = GPIO_NUM_6;

// I2S write buffer: 48kHz stereo 16-bit, ~10ms worth of samples
static constexpr int DMA_BUF_COUNT = 6;
static constexpr int DMA_BUF_SAMPLES = 240;  // frames per DMA buffer

ESP32I2SHardwareProvider::ESP32I2SHardwareProvider(ILogging* logger)
    : txHandle(nullptr)
    , initialized_(false)
    , playing_(false)
    , volume_(1.0)
    , underrunCount_(0)
    , overrunCount_(0)
    , logger_(logger)
    , writerTask_(nullptr)
{}

ESP32I2SHardwareProvider::~ESP32I2SHardwareProvider() {
    cleanup();
}

bool ESP32I2SHardwareProvider::initialize(const AudioStreamFormat& format) {
    if (initialized_) return true;

    currentFormat_ = format;

    i2s_chan_handle_t handle = nullptr;

    i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chanCfg.dma_desc_num = DMA_BUF_COUNT;
    chanCfg.dma_frame_num = DMA_BUF_SAMPLES;
    chanCfg.auto_clear = true;

    esp_err_t err = i2s_new_channel(&chanCfg, &handle, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(err));
        return false;
    }

    i2s_std_config_t stdCfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(static_cast<uint32_t>(format.sampleRate)),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK,
            .ws = I2S_LRCLK,
            .dout = I2S_DIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(handle, &stdCfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S std mode: %s", esp_err_to_name(err));
        i2s_del_channel(handle);
        return false;
    }

    txHandle = handle;
    initialized_ = true;
    ESP_LOGI(TAG, "I2S initialized: %dHz stereo 16-bit", format.sampleRate);
    return true;
}

void ESP32I2SHardwareProvider::cleanup() {
    stopPlayback();
    if (txHandle) {
        i2s_del_channel(txHandle);
        txHandle = nullptr;
    }
    initialized_ = false;
}

bool ESP32I2SHardwareProvider::startPlayback() {
    if (!initialized_ || playing_) return playing_;

    esp_err_t err = i2s_channel_enable(txHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S: %s", esp_err_to_name(err));
        return false;
    }

    // Spawn writer task on Core 1 (I2S on Core 0 is fine, writer on Core 1 keeps it responsive)
    xTaskCreatePinnedToCore(
        writerTaskFunc,
        "i2s_writer",
        WRITER_STACK_SIZE,
        this,
        WRITER_PRIORITY,
        &writerTask_,
        1
    );

    playing_ = true;
    ESP_LOGI(TAG, "Playback started");
    return true;
}

void ESP32I2SHardwareProvider::stopPlayback() {
    if (!playing_) return;
    playing_ = false;

    if (writerTask_) {
        // Task checks playing_ flag and exits
        vTaskDelay(pdMS_TO_TICKS(50));
        writerTask_ = nullptr;
    }

    if (txHandle) {
        i2s_channel_disable(txHandle);
    }
    ESP_LOGI(TAG, "Playback stopped");
}

void ESP32I2SHardwareProvider::setVolume(double volume) {
    volume_ = (volume < 0.0) ? 0.0 : (volume > 1.0) ? 1.0 : volume;
}

double ESP32I2SHardwareProvider::getVolume() const {
    return volume_;
}

bool ESP32I2SHardwareProvider::registerAudioCallback(const AudioCallback& callback) {
    callback_ = callback;
    return static_cast<bool>(callback_);
}

AudioHardwareState ESP32I2SHardwareProvider::getHardwareState() const {
    AudioHardwareState state;
    state.isInitialized = initialized_;
    state.isPlaying = playing_;
    state.isCallbackActive = playing_;
    state.currentVolume = volume_;
    state.underrunCount = underrunCount_;
    state.overrunCount = overrunCount_;
    return state;
}

void ESP32I2SHardwareProvider::resetDiagnostics() {
    underrunCount_ = 0;
    overrunCount_ = 0;
}

void ESP32I2SHardwareProvider::writerTaskFunc(void* arg) {
    auto* self = static_cast<ESP32I2SHardwareProvider*>(arg);

    // Allocate a persistent render buffer: stereo 16-bit
    const int framesPerWrite = DMA_BUF_SAMPLES;
    const size_t samplesPerWrite = framesPerWrite * 2;  // stereo
    auto* pcmBuf = static_cast<int16_t*>(heap_caps_malloc(samplesPerWrite * sizeof(int16_t), MALLOC_CAP_DMA));
    if (!pcmBuf) {
        ESP_LOGE(TAG, "Failed to alloc DMA buffer");
        vTaskDelete(nullptr);
        return;
    }

    // Float render buffer for the callback
    auto* floatBuf = static_cast<float*>(malloc(samplesPerWrite * sizeof(float)));
    if (!floatBuf) {
        heap_caps_free(pcmBuf);
        ESP_LOGE(TAG, "Failed to alloc float buffer");
        vTaskDelete(nullptr);
        return;
    }

    while (self->playing_) {
        if (!self->callback_) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Ask the sim to render stereo float samples
        AudioBufferView view(floatBuf, framesPerWrite, 2);
        (void)self->callback_(view);

        // Convert float to int16 with volume scaling
        const float scale = self->volume_ * 32767.0f;
        for (int i = 0; i < static_cast<int>(samplesPerWrite); ++i) {
            float s = floatBuf[i];
            if (s > 1.0f) s = 1.0f;
            if (s < -1.0f) s = -1.0f;
            pcmBuf[i] = static_cast<int16_t>(s * scale);
        }

        // Write to I2S
        size_t bytesWritten = 0;
        esp_err_t err = i2s_channel_write(self->txHandle, pcmBuf,
            samplesPerWrite * sizeof(int16_t), &bytesWritten, pdMS_TO_TICKS(100));

        if (err == ESP_ERR_TIMEOUT) {
            self->overrunCount_++;
        } else if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S write error: %s", esp_err_to_name(err));
        }
    }

    free(floatBuf);
    heap_caps_free(pcmBuf);
    vTaskDelete(nullptr);
}

#endif // ESP_PLATFORM
