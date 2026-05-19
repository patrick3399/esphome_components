#include "m5cardputer_i2s_speaker.h"
#include "esphome/core/log.h"

#include <cmath>
#include <driver/i2s_std.h>

namespace esphome {
namespace m5cardputer_i2s_speaker {

static const char *const TAG = "m5cardputer_i2s_speaker";

static constexpr i2s_port_t I2S_PORT = I2S_NUM_1;
static constexpr gpio_num_t PIN_BCLK = GPIO_NUM_41;
static constexpr gpio_num_t PIN_LRCLK = GPIO_NUM_43;
static constexpr gpio_num_t PIN_DOUT = GPIO_NUM_42;
static constexpr uint32_t SAMPLE_RATE = 48000;
static constexpr float TONE_HZ = 880.0f;
static constexpr size_t FRAMES_PER_CHUNK = 256;
static constexpr size_t CHUNKS_PER_TONE = 96;

void M5CardputerI2SSpeaker::dump_config() {
  ESP_LOGCONFIG(TAG, "M5Cardputer ESP-IDF New I2S Speaker Test:");
  ESP_LOGCONFIG(TAG, "  I2S: port=1 bclk=GPIO41 lrclk=GPIO43 dout=GPIO42");
}

void M5CardputerI2SSpeaker::play_test_tone() {
  this->play_tone_("stereo both Philips", true, false, false);
  delay(180);
  this->play_tone_("mono right Philips", false, true, false);
  delay(180);
  this->play_tone_("mono left Philips", false, false, false);
  delay(180);
  this->play_tone_("stereo both inverted WS", true, false, true);
}

void M5CardputerI2SSpeaker::play_tone_(const char *label, bool stereo, bool right_slot, bool invert_ws) {
  ESP_LOGI(TAG, "Playing new I2S direct test tone: %s", label);

  i2s_chan_handle_t tx_handle = nullptr;
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = FRAMES_PER_CHUNK;
  chan_cfg.auto_clear = true;

  esp_err_t err = i2s_new_channel(&chan_cfg, &tx_handle, nullptr);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(err));
    return;
  }

  i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

  i2s_std_slot_config_t slot_cfg =
      I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, stereo ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO);
  slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;
  slot_cfg.slot_mask = stereo ? I2S_STD_SLOT_BOTH : (right_slot ? I2S_STD_SLOT_RIGHT : I2S_STD_SLOT_LEFT);
  slot_cfg.ws_pol = invert_ws;

  i2s_std_gpio_config_t gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = PIN_BCLK,
      .ws = PIN_LRCLK,
      .dout = PIN_DOUT,
      .din = I2S_GPIO_UNUSED,
      .invert_flags =
          {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false,
          },
  };

  i2s_std_config_t std_cfg = {
      .clk_cfg = clk_cfg,
      .slot_cfg = slot_cfg,
      .gpio_cfg = gpio_cfg,
  };

  err = i2s_channel_init_std_mode(tx_handle, &std_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %s", esp_err_to_name(err));
    i2s_del_channel(tx_handle);
    return;
  }

  err = i2s_channel_enable(tx_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2s_channel_enable failed: %s", esp_err_to_name(err));
    i2s_del_channel(tx_handle);
    return;
  }

  int16_t mono[FRAMES_PER_CHUNK];
  int16_t stereo_samples[FRAMES_PER_CHUNK * 2];
  uint32_t sample_index = 0;

  for (size_t chunk = 0; chunk < CHUNKS_PER_TONE; chunk++) {
    for (size_t i = 0; i < FRAMES_PER_CHUNK; i++, sample_index++) {
      const float phase = (2.0f * static_cast<float>(M_PI) * TONE_HZ * sample_index) / SAMPLE_RATE;
      const int16_t sample = static_cast<int16_t>(std::sin(phase) * 18000.0f);
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
    err = i2s_channel_write(tx_handle, data, bytes, &written, pdMS_TO_TICKS(500));
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "i2s_channel_write failed: %s", esp_err_to_name(err));
      break;
    }
  }

  i2s_channel_disable(tx_handle);
  i2s_del_channel(tx_handle);
}

}  // namespace m5cardputer_i2s_speaker
}  // namespace esphome
