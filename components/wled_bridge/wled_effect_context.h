#pragma once
#include "wled_types.h"
#include "wled_color.h"
#include "wled_palette.h"
#include "esphome/components/light/addressable_light.h"

namespace esphome {
namespace wled_bridge {

// Per-frame context passed to every effect function.
// Replaces WLED's global SEGMENT / SEGENV / strip macros.
struct EffectContext {
  light::AddressableLight *strip;
  EffectParams *params;
  SegmentState *env;
  uint32_t now;  // millis() snapshot for this frame
  int32_t len;  // strip->size()

  // ---- pixel write ----
  void set_pixel(int32_t i, uint32_t color) {
    if (i < 0 || i >= len)
      return;
    write_pixel((*strip)[i], color);
  }

  void set_pixel_gamma(int32_t i, uint32_t color) {
    // Apply WLED-style gamma before writing
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

  // Fade each pixel toward black by amount (WLED's fade_out / scale down)
  void fade_to_black(uint8_t amount) {
    for (int32_t i = 0; i < len; i++) {
      auto pv = (*strip)[i];
      pv.set_rgb(scale8(pv.get_red(), amount), scale8(pv.get_green(), amount), scale8(pv.get_blue(), amount));
    }
  }

  // ---- color accessors ----
  uint32_t color(uint8_t slot) const {
    return params->colors[slot < 3 ? slot : 0];
  }

  // Color from active palette at position 0..255
  uint32_t pal_color(uint8_t index, uint8_t brightness = 255) const {
    uint8_t pal_id = params->palette_id;
    if (pal_id == PAL_DEFAULT) {
      // Solid segment color
      return color(0);
    }
    if (pal_id == PAL_RAINBOW) {
      return color_wheel(static_cast<uint8_t>(index + (now >> 3)));
    }
    if (pal_id < WLED_PALETTE_COUNT && WLED_PALETTES[pal_id].data != nullptr) {
      return palette_color(*WLED_PALETTES[pal_id].data, index, brightness);
    }
    // Fallback: solid color
    return color(0);
  }

  // ---- per-pixel from palette (position-mapped) ----
  uint32_t pal_color_at(int32_t pixel_idx, uint8_t brightness = 255) const {
    if (len <= 0)
      return 0;
    uint8_t idx = static_cast<uint8_t>((static_cast<uint32_t>(pixel_idx) * 255) / len);
    return pal_color(idx, brightness);
  }

  // color_wheel: hue wheel
  uint32_t wheel(uint8_t pos) const {
    return color_wheel(pos);
  }

  // ---- timing helpers ----
  uint32_t cycle_time() const {
    // Returns a cycleTime in ms proportional to inverse speed (like WLED: 50+(255-speed)*x)
    return 50u + (255u - params->speed) * 5u;
  }

  bool should_run(uint32_t interval_ms) {
    // Returns true if enough time has passed since last step
    if (now - env->step >= interval_ms) {
      env->step = now;
      return true;
    }
    return false;
  }
};

}  // namespace wled_bridge
}  // namespace esphome
