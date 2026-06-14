#pragma once
#include <stdint.h>
#include "wled_types.h"
#include "wled_color.h"

namespace esphome {
namespace wled_bridge {

// ---------- fixed-point beat functions (ported from FastLED) ----------

inline uint16_t beat88(uint16_t beats_per_minute_88, uint32_t time_ms) {
  return (time_ms * beats_per_minute_88 * 280) >> 16;
}

inline uint16_t beat16(uint16_t bpm, uint32_t time_ms) {
  return beat88(bpm << 8, time_ms);
}

inline uint8_t beat8(uint8_t bpm, uint32_t time_ms) {
  return static_cast<uint8_t>(beat16(bpm, time_ms) >> 8);
}

inline uint8_t beatsin8(uint8_t bpm, uint8_t lo, uint8_t hi, uint32_t time_ms, uint8_t phase = 0) {
  uint8_t beat = beat8(bpm, time_ms) + phase;
  uint8_t s = sin8(beat);
  return static_cast<uint8_t>(scale8(s, hi - lo) + lo);
}

inline uint16_t beatsin16(uint16_t bpm, uint16_t lo, uint16_t hi, uint32_t time_ms, uint16_t phase = 0) {
  uint16_t beat = beat16(bpm, time_ms) + phase;
  uint16_t s = static_cast<uint16_t>(sin16(beat) + 32768);  // 0..65535
  return static_cast<uint16_t>(((s * static_cast<uint32_t>(hi - lo)) >> 16) + lo);
}

// ---------- triangle wave ----------
inline uint16_t triwave16(uint16_t v) {
  return v < 0x8000 ? v * 2 : (0xFFFFu - v) * 2;
}
inline uint8_t triwave8(uint8_t v) {
  return v < 128 ? v * 2 : 254 - v * 2;
}

// ---------- cubic wave ----------
// FastLED: cubicwave8(in) = ease8InOutCubic(triwave8(in)).
inline uint8_t cubicwave8(uint8_t v) {
  uint8_t i = triwave8(v);
  uint8_t ii = scale8(i, i);
  uint8_t iii = scale8(ii, i);
  uint16_t r1 = (3u * static_cast<uint16_t>(ii)) - (2u * static_cast<uint16_t>(iii));
  return (r1 & 0x100) ? 255 : static_cast<uint8_t>(r1);
}

// ---------- scale16 ----------
inline uint16_t scale16(uint16_t v, uint16_t s) {
  return static_cast<uint16_t>((static_cast<uint32_t>(v) * s) >> 16);
}

// ---------- Perlin noise (faithful FastLED 3.6.0 gradient-noise port) ----------
// Overloads match FastLED's 1-D / 2-D / 3-D entry points (defined in wled_fx_math.cpp).
// Output: inoise8 → 0..255, inoise16 → 0..65535.
uint8_t inoise8(uint16_t x);
uint8_t inoise8(uint16_t x, uint16_t y);
uint8_t inoise8(uint16_t x, uint16_t y, uint16_t z);
uint16_t inoise16(uint32_t x);
uint16_t inoise16(uint32_t x, uint32_t y);
uint16_t inoise16(uint32_t x, uint32_t y, uint32_t z);

// ---------- map ----------
inline int32_t map_range(int32_t x, int32_t in_lo, int32_t in_hi, int32_t out_lo, int32_t out_hi) {
  return out_lo + (x - in_lo) * (out_hi - out_lo) / (in_hi - in_lo);
}

// ---------- random helpers (complement wled_types.h PRNG) ----------
inline uint8_t get_random_wheel_index(uint8_t base) {
  // pick a wheel index at least 42 away from base
  uint8_t r = 0;
  const uint8_t min_dist = 42;
  do {
    r = hw_random8();
    uint8_t delta = r > base ? r - base : base - r;
    if (delta > 128)
      delta = 256 - delta;
    if (delta >= min_dist)
      break;
  } while (true);
  return r;
}

}  // namespace wled_bridge
}  // namespace esphome
