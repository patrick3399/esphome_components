#pragma once
#include "wled_types.h"
#include "wled_color.h"
#include "wled_palette.h"

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
};

}  // namespace wled_bridge
}  // namespace esphome
