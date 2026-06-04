#include "esphome/core/defines.h"
#ifdef WLED_BRIDGE_AUDIO

#include "wled_audio.h"
#include "wled_fx_math.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/ring_buffer/ring_buffer.h"

#include <cmath>
#include <cstring>

namespace esphome {
namespace wled_bridge {

static const char *const TAG = "wled_bridge.audio";

static constexpr uint32_t RING_BUFFER_MS = 120;
static constexpr size_t BYTES_PER_SAMPLE = 2;  // int16
static constexpr size_t RING_BUFFER_SIZE = (AUDIO_SAMPLE_RATE * RING_BUFFER_MS / 1000) * BYTES_PER_SAMPLE;
static constexpr uint32_t PROCESS_INTERVAL_MS = 20;  // ~50 Hz analysis rate
static constexpr size_t READ_BUF_SAMPLES = (AUDIO_SAMPLE_RATE * PROCESS_INTERVAL_MS / 1000);

// IIR smoothing factor for volume envelope (lower = smoother).
static constexpr float VOLUME_SMOOTH = 0.08f;

// Beat detection: peak-to-average ratio threshold.
static constexpr float BEAT_THRESHOLD = 1.8f;
static constexpr uint32_t BEAT_MIN_INTERVAL_MS = 120;

void AudioAnalyzer::setup(microphone::MicrophoneSource *source, bool enable_fft, bool enable_agc) {
  this->source_ = source;
  this->fft_enabled_ = enable_fft;
  this->agc_enabled_ = enable_agc;

  this->source_->add_data_callback([this](const std::vector<uint8_t> &data) { this->on_audio_data_(data); });

  this->read_buf_len_ = READ_BUF_SAMPLES;
  this->read_buf_ = static_cast<int16_t *>(malloc(this->read_buf_len_ * BYTES_PER_SAMPLE));
  if (this->read_buf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate audio read buffer");
    return;
  }

#ifdef WLED_BRIDGE_FFT
  if (this->fft_enabled_) {
    audio_fft_setup();
  }
#endif

  ESP_LOGCONFIG(TAG, "Audio analyzer ready (FFT: %s, AGC: %s)", this->fft_enabled_ ? "yes" : "no",
                this->agc_enabled_ ? "yes" : "no");
}

void AudioAnalyzer::on_audio_data_(const std::vector<uint8_t> &data) {
  std::shared_ptr<ring_buffer::RingBuffer> rb = this->ring_buffer_.lock();
  if (rb != nullptr) {
    rb->write(const_cast<void *>(static_cast<const void *>(data.data())), data.size());
  }
}

bool AudioAnalyzer::is_active() const {
  if (this->source_ == nullptr)
    return false;
  return this->source_->is_running() && this->started_;
}

void AudioAnalyzer::loop() {
  if (this->source_ == nullptr || this->read_buf_ == nullptr)
    return;

  // Lazy ring buffer allocation (follows sound_level pattern).
  if (!this->started_) {
    if (!this->source_->is_running())
      return;
    auto rb_unique = ring_buffer::RingBuffer::create(RING_BUFFER_SIZE);
    if (rb_unique == nullptr) {
      ESP_LOGE(TAG, "Failed to allocate ring buffer (%u bytes)", RING_BUFFER_SIZE);
      return;
    }
    this->ring_buffer_storage_ = std::shared_ptr<ring_buffer::RingBuffer>(rb_unique.release());
    this->ring_buffer_ = this->ring_buffer_storage_;
    this->started_ = true;
    ESP_LOGD(TAG, "Ring buffer allocated (%u bytes), audio active", RING_BUFFER_SIZE);
  }

  uint32_t now = millis();
  if (now - this->last_process_ms_ < PROCESS_INTERVAL_MS)
    return;
  this->last_process_ms_ = now;

  std::shared_ptr<ring_buffer::RingBuffer> rb = this->ring_buffer_.lock();
  if (rb == nullptr)
    return;

  size_t bytes_available = rb->available();
  size_t bytes_to_read = this->read_buf_len_ * BYTES_PER_SAMPLE;
  if (bytes_available < bytes_to_read)
    return;

  // Drain to latest: skip old data if ring buffer has more than one batch.
  while (bytes_available > bytes_to_read * 2) {
    rb->read(this->read_buf_, bytes_to_read, 0);
    bytes_available -= bytes_to_read;
  }
  rb->read(this->read_buf_, bytes_to_read, 0);

  this->process_volume_(this->read_buf_, this->read_buf_len_);
  this->process_beat_();

#ifdef WLED_BRIDGE_FFT
  if (this->fft_enabled_) {
    audio_fft_process(this->read_buf_, this->read_buf_len_, &this->data_);
  }
#endif
}

void AudioAnalyzer::process_volume_(const int16_t *samples, size_t count) {
  if (count == 0)
    return;

  int64_t sum_sq = 0;
  int32_t peak = 0;

  for (size_t i = 0; i < count; i++) {
    int32_t s = static_cast<int32_t>(samples[i]);
    int32_t sq = s * s;
    sum_sq += sq;
    if (sq > peak)
      peak = sq;
  }

  float rms = sqrtf(static_cast<float>(sum_sq) / static_cast<float>(count));
  float peak_val = sqrtf(static_cast<float>(peak));

  // Scale int16 range to 0–255 (WLED convention).
  float raw = (rms / 32768.0f) * 255.0f;
  this->sample_real_ = (peak_val / 32768.0f) * 255.0f;

  // AGC: adjust gain to keep signal in target range.
  float processed = raw;
  if (this->agc_enabled_ && raw > 0.25f) {
    processed = raw * this->multi_agc_;
    if (processed > 255.0f)
      processed = 255.0f;

    const AgcPreset &p = this->agc_preset_;
    float target = p.target_top * 255.0f;
    float bottom = p.target_bottom * 255.0f;

    if (processed > target) {
      this->multi_agc_ -= p.attack * (processed - target);
    } else if (processed < bottom) {
      this->multi_agc_ += p.decay * (bottom - processed);
    }

    if (this->multi_agc_ < p.min_gain)
      this->multi_agc_ = p.min_gain;
    if (this->multi_agc_ > p.max_gain)
      this->multi_agc_ = p.max_gain;
  }

  // IIR smoothing.
  this->sample_avg_ += VOLUME_SMOOTH * (processed - this->sample_avg_);

  this->data_.volume_raw = processed;
  this->data_.volume_smth = this->sample_avg_;
}

void AudioAnalyzer::process_beat_() {
  float vol = this->data_.volume_smth;

  // Slow-moving average for beat detection baseline.
  this->beat_avg_ += 0.02f * (vol - this->beat_avg_);

  uint32_t now = millis();
  bool is_beat = false;
  if (this->beat_avg_ > 1.0f && vol > BEAT_THRESHOLD * this->beat_avg_) {
    if (now - this->last_beat_ms_ > BEAT_MIN_INTERVAL_MS) {
      is_beat = true;
      this->last_beat_ms_ = now;
    }
  }
  this->data_.sample_peak = is_beat;
}

// ---------- simulateSound — 4 modes matching WLED ----------

void AudioAnalyzer::simulate(uint8_t mode, uint32_t now) {
  uint32_t ms = now;
  AudioData &d = this->data_;

  switch (mode) {
    case SIM_BEAT_SIN:
      for (int i = 0; i < 16; i++)
        d.fft_result[i] = beatsin8(120 / (i + 1), 0, 255, ms, 0);
      d.volume_smth = d.fft_result[8];
      break;

    case SIM_ROCK_YOU: {
      uint32_t cycle = (ms / 200) % 5;
      memset(d.fft_result, 0, 16);
      if (cycle == 0) {
        for (int i = 0; i < 5; i++)
          d.fft_result[i] = hw_random8(120, 255);
        d.volume_smth = hw_random8(180, 255);
      } else if (cycle == 2) {
        for (int i = 5; i < 11; i++)
          d.fft_result[i] = hw_random8(120, 255);
        d.volume_smth = hw_random8(180, 255);
      } else if (cycle == 4) {
        for (int i = 11; i < 16; i++)
          d.fft_result[i] = hw_random8(120, 255);
        d.volume_smth = hw_random8(180, 255);
      } else {
        d.volume_smth = 0;
      }
      break;
    }

    case SIM_PERLIN_A:
      for (int i = 0; i < 16; i++)
        d.fft_result[i] = inoise8(beatsin8(90 / (i + 1), 0, 200, ms) * 15 + (ms >> 10), ms >> 3);
      d.volume_smth = d.fft_result[8];
      break;

    case SIM_PERLIN_B:
      for (int i = 0; i < 16; i++)
        d.fft_result[i] = inoise8(beatsin8(120 / (i + 1), 10, 30, ms) * 10 + (ms >> 14), ms >> 3);
      d.volume_smth = d.fft_result[8];
      break;
  }

  d.sample_peak = hw_random8() > 250;
  d.fft_major_peak = 21.0f + (d.volume_smth * d.volume_smth) / 8.0f;
  d.volume_raw = d.volume_smth;
  d.fft_magnitude = (d.volume_smth > 1.0f) ? 10000.0f / 8.0f : 0.001f;
  d.max_vol = 31;
}

}  // namespace wled_bridge
}  // namespace esphome

#endif  // WLED_BRIDGE_AUDIO
