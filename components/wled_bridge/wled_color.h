#pragma once
#include <stdint.h>
#include "esphome/components/light/addressable_light.h"

namespace esphome {
namespace wled_bridge {

// ---------- 32-bit color pack/unpack (WLED uses 0xWWRRGGBB) ----------
inline uint32_t RGBW32(uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0) {
  return (static_cast<uint32_t>(w) << 24) | (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) |
         static_cast<uint32_t>(b);
}
inline uint8_t R(uint32_t c) {
  return (c >> 16) & 0xFF;
}
inline uint8_t G(uint32_t c) {
  return (c >> 8) & 0xFF;
}
inline uint8_t B(uint32_t c) {
  return c & 0xFF;
}
inline uint8_t W(uint32_t c) {
  return (c >> 24) & 0xFF;
}

// ---------- saturating arithmetic ----------
inline uint8_t qadd8(uint8_t a, uint8_t b) {
  uint16_t t = static_cast<uint16_t>(a) + b;
  return t > 255 ? 255 : static_cast<uint8_t>(t);
}
inline uint8_t qsub8(uint8_t a, uint8_t b) {
  return a > b ? a - b : 0;
}

// scale8: (v * s + 1) / 256  (FastLED semantics, never rounds to 0 when v>0 && s>0)
inline uint8_t scale8(uint8_t v, uint8_t s) {
  return static_cast<uint8_t>((static_cast<uint16_t>(v) * s) >> 8);
}
inline uint8_t scale8_video(uint8_t v, uint8_t s) {
  return (v == 0 || s == 0) ? 0 : static_cast<uint8_t>((static_cast<uint16_t>(v) * s) >> 8) + 1;
}

// ---------- sin/cos lookups ----------
uint8_t sin8(uint8_t v);  // defined in wled_color.cpp
uint8_t cos8(uint8_t v);
int16_t sin16(uint16_t theta);

// ---------- gamma ----------
uint8_t gamma8(uint8_t v);  // apply gamma 2.8 forward
uint8_t gamma8_inv(uint8_t v);  // inverse gamma

// ---------- linear interpolation ----------
inline uint8_t lerp8by8(uint8_t a, uint8_t b, uint8_t frac) {
  int16_t d = static_cast<int16_t>(b) - a;
  return static_cast<uint8_t>(a + ((d * frac) >> 8));
}

// ---------- WLED color utilities ----------
uint32_t color_blend(uint32_t c1, uint32_t c2, uint8_t blend);
uint32_t color_fade(uint32_t c, uint8_t amount, bool video = false);
uint32_t color_add(uint32_t c1, uint32_t c2);

// color_wheel: 8-bit hue → RGB packed as uint32 (WLED's implementation)
uint32_t color_wheel(uint8_t pos);

// ---------- ESPHome pixel write bridge ----------
// Writes a WLED uint32 color (0xWWRRGGBB) into an ESPColorView pixel.
// Gamma is already applied by caller; ESPHome correction should be set to pass-through.
inline void write_pixel(light::ESPColorView pv, uint32_t color) {
  pv.set_rgbw(R(color), G(color), B(color), W(color));
}

}  // namespace wled_bridge
}  // namespace esphome
