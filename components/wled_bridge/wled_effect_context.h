#pragma once
#include "esphome/core/defines.h"
#include "wled_types.h"
#include "wled_color.h"
#include "wled_palette.h"
#ifdef WLED_BRIDGE_AUDIO
#include "wled_audio.h"
#endif

namespace esphome {
namespace wled_bridge {

// Per-frame context passed to every effect function.
// Effects write into frame_buf[] using virtual absolute indices —
// the bus mapping (which physical strip owns which indices) is resolved
// later by WLEDBridgeComponent::flush_frame_to_buses_().
struct EffectContext {
  uint32_t *frame_buf;  // bridge's full_frame_[] — virtual absolute indices
  uint32_t frame_len;  // total virtual LED count (sum of all buses)
  EffectParams *params;
  SegmentState *env;
  uint32_t now;  // millis() snapshot for this frame
  int32_t start;  // inclusive segment start (virtual absolute index)
  int32_t stop;  // exclusive segment stop (physical)
  int32_t len;  // virtual segment length (after grouping/spacing collapse)
  bool reverse;
  bool mirror;
  uint16_t grouping{1};  // physical pixels painted per virtual pixel
  uint16_t spacing{0};  // physical pixels skipped after each group

  // virtual segment index → first physical absolute index of its group.
  // With grouping=1, spacing=0 this is identical to the legacy 1:1 mapping.
  int32_t map_pixel(int32_t i) const {
    if (i < 0 || i >= len)
      return -1;
    int32_t mapped = reverse ? (len - 1 - i) : i;
    if (mirror) {
      int32_t half = (len + 1) / 2;
      if (mapped >= half)
        mapped = len - 1 - mapped;
    }
    int32_t group = grouping < 1 ? 1 : grouping;
    int32_t step = group + spacing;
    return start + mapped * step;
  }

  // ---- pixel write ----
  void set_pixel(int32_t i, uint32_t color) {
    int32_t base = map_pixel(i);
    if (base < 0)
      return;
    int32_t group = grouping < 1 ? 1 : grouping;
    for (int32_t g = 0; g < group; g++) {
      int32_t abs = base + g;
      if (abs >= start && abs < stop && abs < static_cast<int32_t>(frame_len))
        frame_buf[abs] = color;
    }
  }

  void set_pixel_gamma(int32_t i, uint32_t color) {
    set_pixel(i, RGBW32(gamma8(R(color)), gamma8(G(color)), gamma8(B(color)), W(color)));
  }

  // ---- bulk helpers ----
  void fill(uint32_t color) {
    for (int32_t i = 0; i < len; i++)
      set_pixel(i, color);
  }

  void fill_black() {
    fill(0);
  }

  // Read-modify-write: fade each physical segment pixel toward black.
  // Reads from frame_buf (unscaled previous frame) — correct for trailing-fade effects.
  void fade_to_black(uint8_t amount) {
    for (int32_t abs = start; abs < stop && abs < static_cast<int32_t>(frame_len); abs++) {
      uint32_t &c = frame_buf[abs];
      c = RGBW32(scale8(R(c), amount), scale8(G(c), amount), scale8(B(c), amount), scale8(W(c), amount));
    }
  }

  // ---- color accessors ----
  uint32_t color(uint8_t slot) const {
    return params->colors[slot < 3 ? slot : 0];
  }

  uint32_t pal_color(uint8_t index, uint8_t brightness = 255) const {
    uint8_t pal_id = params->palette_id;
    if (pal_id == PAL_DEFAULT)
      return color(0);

    // PAL_RANDOM (1): slow-cycling random hue palette, morphs every ~5 s.
    // Uses deterministic hash of time so all pixels see a consistent palette
    // within the same frame, and the palette transitions smoothly across periods.
    if (pal_id == PAL_RANDOM) {
      uint32_t period = 5000u;
      uint32_t phase = now / period;
      uint32_t frac = ((now % period) * 255u) / period;  // 0..255 blend into next phase
      // Hash function: simple LCG per phase to get 4 base hues
      auto hash_hue = [](uint32_t ph, uint8_t slot) -> uint8_t {
        uint32_t h = ph * 2654435761u + static_cast<uint32_t>(slot) * 2246822519u;
        return static_cast<uint8_t>(h >> 16);
      };
      uint8_t slot = index >> 6;  // 0..3 quadrant
      uint8_t h_cur = hash_hue(phase, slot);
      uint8_t h_nxt = hash_hue(phase + 1, slot);
      uint8_t h =
          h_cur + static_cast<uint8_t>(((static_cast<int16_t>(h_nxt) - h_cur) * static_cast<int16_t>(frac)) / 255);
      uint32_t c = color_wheel(static_cast<uint8_t>(h + (index & 0x3F)));
      if (brightness != 255)
        c = RGBW32(scale8(R(c), brightness), scale8(G(c), brightness), scale8(B(c), brightness), 0);
      return c;
    }

    // PAL_PRIMARY (2): gradient black → color0 → white, centered on the segment's
    // primary colour.  Gives any effect an instant "themed" look from the UI colour picker.
    if (pal_id == PAL_PRIMARY) {
      uint32_t c0 = color(0);
      uint32_t out;
      if (index < 128) {
        // black → color0
        uint8_t f = index * 2;
        out = RGBW32(scale8(R(c0), f), scale8(G(c0), f), scale8(B(c0), f), scale8(W(c0), f));
      } else {
        // color0 → white
        uint8_t f = (index - 128) * 2;
        out = RGBW32(R(c0) + scale8(255 - R(c0), f), G(c0) + scale8(255 - G(c0), f), B(c0) + scale8(255 - B(c0), f),
                     W(c0) + scale8(255 - W(c0), f));
      }
      if (brightness != 255)
        out = RGBW32(scale8(R(out), brightness), scale8(G(out), brightness), scale8(B(out), brightness),
                     scale8(W(out), brightness));
      return out;
    }

    if (pal_id >= 3 && pal_id <= 5) {
      uint32_t stops[4];
      size_t stop_count = 0;
      if (pal_id == 3) {
        stops[0] = color(0);
        stops[1] = color(0);
        stops[2] = color(1);
        stops[3] = color(1);
        stop_count = 4;
      } else if (pal_id == 4) {
        stops[0] = color(2);
        stops[1] = color(1);
        stops[2] = color(0);
        stop_count = 3;
      } else {
        stops[0] = color(0);
        stops[1] = color(1);
        stops[2] = color(2);
        stops[3] = color(0);
        stop_count = 4;
      }

      uint8_t scaled =
          stop_count > 1 ? static_cast<uint8_t>((static_cast<uint16_t>(index) * (stop_count - 1)) / 255) : 0;
      if (scaled >= stop_count - 1)
        scaled = stop_count - 2;
      uint8_t local = static_cast<uint8_t>((static_cast<uint16_t>(index) * (stop_count - 1)) -
                                           (static_cast<uint16_t>(scaled) * 255));
      uint32_t a = stops[scaled];
      uint32_t b = stops[scaled + 1];
      uint32_t out = RGBW32(lerp8by8(R(a), R(b), local), lerp8by8(G(a), G(b), local), lerp8by8(B(a), B(b), local),
                            lerp8by8(W(a), W(b), local));
      if (brightness != 255)
        out = RGBW32(scale8(R(out), brightness), scale8(G(out), brightness), scale8(B(out), brightness),
                     scale8(W(out), brightness));
      return out;
    }

    if (pal_id == PAL_RAINBOW)
      return color_wheel(static_cast<uint8_t>(index + (now >> 3)));
    if (pal_id < WLED_PALETTE_COUNT && WLED_PALETTES[pal_id].data != nullptr)
      return palette_color(*WLED_PALETTES[pal_id].data, index, brightness);
    return color(0);
  }

  uint32_t pal_color_at(int32_t pixel_idx, uint8_t brightness = 255) const {
    if (len <= 0)
      return 0;
    uint8_t idx = static_cast<uint8_t>((static_cast<uint32_t>(pixel_idx) * 255) / static_cast<uint32_t>(len));
    return pal_color(idx, brightness);
  }

  uint32_t wheel(uint8_t pos) const {
    return color_wheel(pos);
  }

  // ---- timing helpers ----
  uint32_t cycle_time() const {
    return 50u + (255u - params->speed) * 5u;
  }

  bool should_run(uint32_t interval_ms) {
    if (now - env->step >= interval_ms) {
      env->step = now;
      return true;
    }
    return false;
  }

  // ---- 2D matrix support ----
  // These are zero when no matrix is configured; effects must guard with is_2d().
  uint16_t matrix_w{0};
  uint16_t matrix_h{0};
  bool serpentine{false};

  bool is_2d() const {
    return matrix_w > 0 && matrix_h > 0;
  }

  // Map (x,y) to segment-local 1D virtual index. Returns -1 if out of bounds.
  int32_t xy(int16_t x, int16_t y) const {
    if (x < 0 || y < 0 || x >= static_cast<int16_t>(matrix_w) || y >= static_cast<int16_t>(matrix_h))
      return -1;
    int32_t row = static_cast<int32_t>(y);
    int32_t col = static_cast<int32_t>(x);
    if (serpentine && (row & 1))
      col = static_cast<int32_t>(matrix_w) - 1 - col;
    return row * static_cast<int32_t>(matrix_w) + col;
  }

  void set_pixel_2d(int16_t x, int16_t y, uint32_t color) {
    int32_t i = xy(x, y);
    if (i >= 0)
      set_pixel(i, color);
  }

  uint32_t get_pixel_2d(int16_t x, int16_t y) const {
    int32_t i = xy(x, y);
    if (i < 0)
      return 0;
    int32_t base = map_pixel(i);
    if (base < start || base >= stop || base >= static_cast<int32_t>(frame_len))
      return 0;
    return frame_buf[base];
  }

  // Fade all 2D pixels (same as fade_to_black but named for clarity in 2D effects).
  void fade_2d(uint8_t amount) {
    fade_to_black(amount);
  }

#ifdef WLED_BRIDGE_AUDIO
  // ---- audio data (never nullptr when WLED_BRIDGE_AUDIO is defined) ----
  const AudioData *audio{nullptr};

  float volume() const {
    return audio != nullptr ? audio->volume_smth : 0;
  }
  bool has_audio() const {
    return audio != nullptr && audio->volume_smth > 0.5f;
  }
  bool beat() const {
    return audio != nullptr && audio->sample_peak;
  }
  uint8_t fft_bin(uint8_t i) const {
#ifdef WLED_BRIDGE_FFT
    return (audio != nullptr && i < 16) ? audio->fft_result[i] : 0;
#else
    return 0;
#endif
  }
  float major_peak() const {
#ifdef WLED_BRIDGE_FFT
    return audio != nullptr ? audio->fft_major_peak : 0;
#else
    return 0;
#endif
  }
#endif  // WLED_BRIDGE_AUDIO
};

}  // namespace wled_bridge
}  // namespace esphome
