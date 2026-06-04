#include "esphome/core/defines.h"
#if defined(WLED_BRIDGE_AUDIO) && defined(WLED_BRIDGE_FFT)

#include "wled_audio.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstring>

namespace esphome {
namespace wled_bridge {

static const char *const TAG = "wled_bridge.audio";

// Lightweight in-place radix-2 real FFT.
// N must be a power of 2.  Input is int16 samples, output is float magnitudes.
// No external library dependency — self-contained for ESPHome.

static constexpr size_t FFT_N = AUDIO_FFT_SAMPLES;  // 512
static constexpr size_t FFT_HALF = FFT_N / 2;  // 256

// Pre-computed twiddle factors (sin/cos lookup for N=512).
static float *fft_sin_table_{nullptr};
static float *fft_cos_table_{nullptr};

// Bit-reversal permutation table.
static uint16_t *fft_bit_rev_{nullptr};

// Working buffers.
static float *fft_real_{nullptr};
static float *fft_imag_{nullptr};

static void fft_free_buffers_() {
  free(fft_sin_table_);
  free(fft_cos_table_);
  free(fft_bit_rev_);
  free(fft_real_);
  free(fft_imag_);
  fft_sin_table_ = nullptr;
  fft_cos_table_ = nullptr;
  fft_bit_rev_ = nullptr;
  fft_real_ = nullptr;
  fft_imag_ = nullptr;
}

static void fft_compute_tables_() {
  for (size_t i = 0; i < FFT_HALF; i++) {
    float angle = -2.0f * M_PI * static_cast<float>(i) / static_cast<float>(FFT_N);
    fft_sin_table_[i] = sinf(angle);
    fft_cos_table_[i] = cosf(angle);
  }

  for (size_t i = 0; i < FFT_N; i++) {
    uint16_t j = 0;
    uint16_t x = static_cast<uint16_t>(i);
    for (int b = 0; b < 9; b++) {  // log2(512) = 9
      j = (j << 1) | (x & 1);
      x >>= 1;
    }
    fft_bit_rev_[i] = j;
  }
}

static void fft_execute_(const int16_t *input, size_t input_count) {
  // Fill working buffer (zero-pad if input is short).
  for (size_t i = 0; i < FFT_N; i++) {
    fft_real_[i] = (i < input_count) ? static_cast<float>(input[i]) : 0.0f;
    fft_imag_[i] = 0.0f;
  }

  // Apply Hann window.
  for (size_t i = 0; i < FFT_N; i++) {
    float w = 0.5f * (1.0f - cosf(2.0f * M_PI * static_cast<float>(i) / static_cast<float>(FFT_N)));
    fft_real_[i] *= w;
  }

  // Bit-reversal permutation.
  for (size_t i = 0; i < FFT_N; i++) {
    if (fft_bit_rev_[i] > i) {
      float tmp = fft_real_[i];
      fft_real_[i] = fft_real_[fft_bit_rev_[i]];
      fft_real_[fft_bit_rev_[i]] = tmp;
    }
  }

  // Cooley-Tukey radix-2 DIT.
  for (size_t stage = 1; stage < FFT_N; stage <<= 1) {
    size_t step = stage << 1;
    size_t twiddle_step = FFT_HALF / stage;
    for (size_t k = 0; k < FFT_N; k += step) {
      for (size_t j = 0; j < stage; j++) {
        size_t ti = j * twiddle_step;
        float cos_v = fft_cos_table_[ti];
        float sin_v = fft_sin_table_[ti];
        size_t even = k + j;
        size_t odd = even + stage;
        float tr = cos_v * fft_real_[odd] - sin_v * fft_imag_[odd];
        float ti2 = sin_v * fft_real_[odd] + cos_v * fft_imag_[odd];
        fft_real_[odd] = fft_real_[even] - tr;
        fft_imag_[odd] = fft_imag_[even] - ti2;
        fft_real_[even] += tr;
        fft_imag_[even] += ti2;
      }
    }
  }
}

// WLED-style non-linear bin mapping for 16 GEQ channels.
// Maps 256 FFT bins (0–11025 Hz at 22050 Hz sample rate) into 16 bands.
// Lower bands are narrower (better bass resolution), upper bands are wider.
static constexpr uint16_t GEQ_BIN_STARTS[AUDIO_GEQ_CHANNELS + 1] = {1,  2,  3,  5,   7,   10,  14,  20, 28,
                                                                    40, 56, 79, 111, 156, 180, 210, 256};

bool audio_fft_setup() {
  fft_sin_table_ = static_cast<float *>(malloc(FFT_HALF * sizeof(float)));
  fft_cos_table_ = static_cast<float *>(malloc(FFT_HALF * sizeof(float)));
  fft_bit_rev_ = static_cast<uint16_t *>(malloc(FFT_N * sizeof(uint16_t)));
  fft_real_ = static_cast<float *>(malloc(FFT_N * sizeof(float)));
  fft_imag_ = static_cast<float *>(malloc(FFT_N * sizeof(float)));

  if (fft_sin_table_ == nullptr || fft_cos_table_ == nullptr || fft_bit_rev_ == nullptr || fft_real_ == nullptr ||
      fft_imag_ == nullptr) {
    fft_free_buffers_();
    ESP_LOGE(TAG, "Failed to allocate FFT buffers");
    return false;
  }

  fft_compute_tables_();
  ESP_LOGCONFIG(TAG, "FFT initialized (N=%u, %u GEQ channels)", FFT_N, AUDIO_GEQ_CHANNELS);
  return true;
}

void audio_fft_process(const int16_t *samples, size_t count, AudioData *out) {
  if (samples == nullptr || out == nullptr || fft_sin_table_ == nullptr || fft_cos_table_ == nullptr ||
      fft_bit_rev_ == nullptr || fft_real_ == nullptr || fft_imag_ == nullptr)
    return;

  fft_execute_(samples, count);

  // Compute magnitudes for lower half of spectrum.
  float magnitudes[FFT_HALF];
  float max_mag = 0;
  size_t max_bin = 0;
  for (size_t i = 0; i < FFT_HALF; i++) {
    float mag = sqrtf(fft_real_[i] * fft_real_[i] + fft_imag_[i] * fft_imag_[i]);
    magnitudes[i] = mag;
    if (i > 0 && mag > max_mag) {
      max_mag = mag;
      max_bin = i;
    }
  }

  // Aggregate into 16 GEQ channels.
  for (uint8_t ch = 0; ch < AUDIO_GEQ_CHANNELS; ch++) {
    uint16_t lo = GEQ_BIN_STARTS[ch];
    uint16_t hi = GEQ_BIN_STARTS[ch + 1];
    float sum = 0;
    for (uint16_t b = lo; b < hi && b < FFT_HALF; b++)
      sum += magnitudes[b];
    float avg = sum / static_cast<float>(hi - lo);

    // Normalize to 0–254 (WLED convention: 254 max, not 255).
    float scaled = avg / 256.0f;
    if (scaled > 254.0f)
      scaled = 254.0f;
    out->fft_result[ch] = static_cast<uint8_t>(scaled);
  }

  // Major peak frequency.
  float bin_hz = static_cast<float>(AUDIO_SAMPLE_RATE) / static_cast<float>(FFT_N);
  out->fft_major_peak = static_cast<float>(max_bin) * bin_hz;
  out->fft_magnitude = max_mag;
}

}  // namespace wled_bridge
}  // namespace esphome

#endif  // WLED_BRIDGE_AUDIO && WLED_BRIDGE_FFT
