#include "m5cardputer_legacy_speaker.h"
#include "esphome/core/log.h"

#include <cmath>
#include <driver/i2s.h>

namespace esphome {
namespace m5cardputer_legacy_speaker {

static const char *const TAG = "m5cardputer_legacy_speaker";

static constexpr i2s_port_t I2S_PORT = I2S_NUM_1;
static constexpr int PIN_BCLK = 41;
static constexpr int PIN_LRCLK = 43;
static constexpr int PIN_DOUT = 42;
static constexpr uint32_t SAMPLE_RATE = 48000;
static constexpr float TONE_HZ = 880.0f;
static constexpr size_t FRAMES_PER_CHUNK = 256;
static constexpr size_t CHUNKS_PER_TONE = 96;  // 512 ms at 48 kHz

void M5CardputerLegacySpeaker::dump_config() {
  ESP_LOGCONFIG(TAG, "M5Cardputer Legacy Speaker Test:");
  ESP_LOGCONFIG(TAG, "  I2S: port=1 bclk=GPIO41 lrclk=GPIO43 dout=GPIO42");
}

void M5CardputerLegacySpeaker::play_test_tone() {
  ESP_LOGI(TAG, "Playing legacy I2S test tone: mono right");
  this->play_tone_(false);
  delay(120);
  ESP_LOGI(TAG, "Playing legacy I2S test tone: stereo both");
  this->play_tone_(true);
}

void M5CardputerLegacySpeaker::play_tone_(bool stereo) {
  i2s_driver_uninstall(I2S_PORT);

  i2s_config_t config = {};
  config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
  config.sample_rate = SAMPLE_RATE;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = stereo ? I2S_CHANNEL_FMT_RIGHT_LEFT : I2S_CHANNEL_FMT_ONLY_RIGHT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_desc_num = 8;
  config.dma_frame_num = FRAMES_PER_CHUNK;
  config.use_apll = false;
  config.tx_desc_auto_clear = true;
  config.fixed_mclk = 0;
  config.mclk_multiple = I2S_MCLK_MULTIPLE_256;

  esp_err_t err = i2s_driver_install(I2S_PORT, &config, 0, nullptr);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2s_driver_install failed: %s", esp_err_to_name(err));
    return;
  }

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = PIN_BCLK;
  pins.ws_io_num = PIN_LRCLK;
  pins.data_out_num = PIN_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  err = i2s_set_pin(I2S_PORT, &pins);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2s_set_pin failed: %s", esp_err_to_name(err));
    i2s_driver_uninstall(I2S_PORT);
    return;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  i2s_start(I2S_PORT);

  int16_t mono[FRAMES_PER_CHUNK];
  int16_t stereo_samples[FRAMES_PER_CHUNK * 2];
  uint32_t sample_index = 0;

  for (size_t chunk = 0; chunk < CHUNKS_PER_TONE; chunk++) {
    for (size_t i = 0; i < FRAMES_PER_CHUNK; i++, sample_index++) {
      const float phase = (2.0f * static_cast<float>(M_PI) * TONE_HZ * sample_index) / SAMPLE_RATE;
      const int16_t sample = static_cast<int16_t>(std::sin(phase) * 14000.0f);
      if (stereo) {
        stereo_samples[i * 2] = sample;
        stereo_samples[i * 2 + 1] = sample;
      } else {
        mono[i] = sample;
      }
    }

    size_t written = 0;
    const void *data = stereo ? static_cast<const void *>(stereo_samples) : static_cast<const void *>(mono);
    const size_t bytes = stereo ? sizeof(stereo_samples) : sizeof(mono);
    err = i2s_write(I2S_PORT, data, bytes, &written, portMAX_DELAY);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "i2s_write failed: %s", esp_err_to_name(err));
      break;
    }
  }

  i2s_zero_dma_buffer(I2S_PORT);
  delay(20);
  i2s_stop(I2S_PORT);
  i2s_driver_uninstall(I2S_PORT);
}

}  // namespace m5cardputer_legacy_speaker
}  // namespace esphome
