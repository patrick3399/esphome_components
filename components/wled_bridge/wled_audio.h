#pragma once
#ifdef WLED_BRIDGE_AUDIO

#include "esphome/core/defines.h"
#include "esphome/components/microphone/microphone_source.h"
#include "esphome/components/ring_buffer/ring_buffer.h"
#include "wled_types.h"

#include <memory>

namespace esphome {

namespace wled_bridge {

static constexpr uint8_t AUDIO_GEQ_CHANNELS = 16;
static constexpr uint16_t AUDIO_SAMPLE_RATE = 22050;
static constexpr uint16_t AUDIO_FFT_SAMPLES = 512;

// Smoothed audio data available to effects every frame.
// Modelled after WLED's um_data fields so effects can port 1:1.
struct AudioData {
  // Layer 1: Volume (always present)
  float volume_smth{0};  // 0–255, IIR-smoothed amplitude (≈ WLED volumeSmth)
  float volume_raw{0};  // 0–255, unsmoothed (≈ WLED volumeRaw)
  bool sample_peak{false};  // beat detection flag (≈ WLED samplePeak)
  uint8_t max_vol{10};  // AGC target ceiling

  // Layer 2: FFT (zeroed when WLED_BRIDGE_FFT not compiled)
  uint8_t fft_result[AUDIO_GEQ_CHANNELS]{};
  float fft_major_peak{0};  // dominant frequency (Hz)
  float fft_magnitude{0};  // magnitude at peak

  bool has_audio() const {
    return volume_smth > 0.5f;
  }
};

enum SoundSimMode : uint8_t {
  SIM_BEAT_SIN = 0,
  SIM_ROCK_YOU = 1,
  SIM_PERLIN_A = 2,
  SIM_PERLIN_B = 3,
};

// AGC presets matching WLED's three modes.
struct AgcPreset {
  float target_top;
  float target_bottom;
  float attack;
  float decay;
  float min_gain;
  float max_gain;
};

static constexpr AgcPreset AGC_NORMAL{0.80f, 0.60f, 0.0005f, 0.0001f, 0.5f, 4.0f};
static constexpr AgcPreset AGC_VIVID{0.80f, 0.50f, 0.0010f, 0.0002f, 0.5f, 6.0f};
static constexpr AgcPreset AGC_LAZY{0.85f, 0.65f, 0.0002f, 0.0001f, 0.5f, 3.0f};

// Consumes audio samples from an ESPHome MicrophoneSource and produces
// per-frame AudioData for the effect engine.  Designed to coexist with
// voice_assistant via the MicrophoneSource passive mode.
class AudioAnalyzer {
 public:
  void setup(microphone::MicrophoneSource *source, bool enable_fft, bool enable_agc);
  void loop();

  const AudioData &data() const {
    return this->data_;
  }
  bool is_active() const;

  // Fill AudioData with synthetic patterns when no real audio is available.
  void simulate(uint8_t mode, uint32_t now);

  void set_agc_enabled(bool v) {
    this->agc_enabled_ = v;
  }
  bool get_agc_enabled() const {
    return this->agc_enabled_;
  }

 protected:
  void on_audio_data_(const std::vector<uint8_t> &data);
  void process_volume_(const int16_t *samples, size_t count);
  void process_beat_();

  microphone::MicrophoneSource *source_{nullptr};
  std::weak_ptr<ring_buffer::RingBuffer> ring_buffer_;
  std::shared_ptr<ring_buffer::RingBuffer> ring_buffer_storage_;
  AudioData data_{};

  // Volume state
  float sample_avg_{0};
  float sample_real_{0};  // peak of current batch
  int32_t squared_peak_{0};

  // AGC state
  bool agc_enabled_{true};
  AgcPreset agc_preset_{AGC_NORMAL};
  float multi_agc_{1.0f};
  float sample_agc_{0};

  // Beat detection
  float beat_avg_{0};
  uint32_t last_beat_ms_{0};

  // Ring buffer read
  int16_t *read_buf_{nullptr};
  size_t read_buf_len_{0};

  // FFT
  bool fft_enabled_{false};

  uint32_t last_process_ms_{0};
  bool started_{false};
};

#ifdef WLED_BRIDGE_FFT
// Implemented in wled_audio_fft.cpp — called by AudioAnalyzer::loop().
void audio_fft_process(const int16_t *samples, size_t count, AudioData *out);
bool audio_fft_setup();
#endif

}  // namespace wled_bridge
}  // namespace esphome

#endif  // WLED_BRIDGE_AUDIO
