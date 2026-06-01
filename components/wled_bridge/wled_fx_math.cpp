#include "wled_fx_math.h"

namespace esphome {
namespace wled_bridge {

// ---------- Perlin noise — 8-bit output variant ----------
// Based on Ken Perlin's gradient noise, adapted from FastLED's noise8 implementation.
// Uses a 256-entry permutation table.

static const uint8_t P[256] = {
    151, 160, 137, 91,  90,  15,  131, 13,  201, 95,  96,  53,  194, 233, 7,   225, 140, 36,  103, 30,  69,  142,
    8,   99,  37,  240, 21,  10,  23,  190, 6,   148, 247, 120, 234, 75,  0,   26,  197, 62,  94,  252, 219, 203,
    117, 35,  11,  32,  57,  177, 33,  88,  237, 149, 56,  87,  174, 20,  125, 136, 171, 168, 68,  175, 74,  165,
    71,  134, 139, 48,  27,  166, 77,  146, 158, 231, 83,  111, 229, 122, 60,  211, 133, 230, 220, 105, 92,  41,
    55,  46,  245, 40,  244, 102, 143, 54,  65,  25,  63,  161, 1,   216, 80,  73,  209, 76,  132, 187, 208, 89,
    18,  169, 200, 196, 135, 130, 116, 188, 159, 86,  164, 100, 109, 198, 173, 186, 3,   64,  52,  217, 226, 250,
    124, 123, 5,   202, 38,  147, 118, 126, 255, 82,  85,  212, 207, 206, 59,  227, 47,  16,  58,  17,  182, 189,
    28,  42,  223, 183, 170, 213, 119, 248, 152, 2,   44,  154, 163, 70,  221, 153, 101, 155, 167, 43,  172, 9,
    129, 22,  39,  253, 19,  98,  108, 110, 79,  113, 224, 232, 178, 185, 112, 104, 218, 246, 97,  228, 251, 34,
    242, 193, 238, 210, 144, 12,  191, 179, 162, 241, 81,  51,  145, 235, 249, 14,  239, 107, 49,  192, 214, 31,
    181, 199, 106, 157, 184, 84,  204, 176, 115, 121, 50,  45,  127, 4,   150, 254, 138, 236, 205, 93,  222, 114,
    67,  29,  24,  72,  243, 141, 128, 195, 78,  66,  215, 61,  156, 180,
};

static inline uint8_t p8(uint8_t i) {
  return P[i];
}

static int8_t grad8(uint8_t hash, int8_t x) {
  // 4-gradient set: 1, -1, ~0.7, ~-0.7
  uint8_t h = hash & 0x0F;
  int8_t grad = 1 + (h & 7);
  if (h & 8)
    grad = -grad;
  return (grad * x) >> 3;
}

uint8_t inoise8(uint16_t x, uint16_t /*y*/, uint16_t /*z*/) {
  // 1D Perlin noise using 8.8 fixed-point
  uint8_t xi = x >> 8;  // integer part
  uint8_t xf = x & 0xFF;  // fractional part

  uint8_t a = p8(xi);
  uint8_t b = p8(static_cast<uint8_t>(xi + 1));

  // fade curve: t * t * (3 - 2*t), computed in 8-bit fixed-point
  uint8_t u = xf;
  uint8_t fade_u = scale8(u, scale8(u, 255 - u * 2 / 255 * 255));  // approximate
  // simpler: cubic fade via integer multiply
  uint16_t uu = u;
  uint8_t fade = static_cast<uint8_t>((uu * uu * (3 * 256 - 2 * uu)) >> 16);

  int8_t ga = grad8(a, static_cast<int8_t>(xf));
  int8_t gb = grad8(b, static_cast<int8_t>(xf - 256));

  // lerp ga..gb by fade
  int16_t lerped = static_cast<int16_t>(ga) + ((static_cast<int16_t>(gb - ga) * fade) >> 8);
  // map -128..127 → 0..255
  return static_cast<uint8_t>(lerped + 128);
}

uint16_t inoise16(uint32_t x, uint32_t /*y*/, uint32_t /*z*/) {
  // 16-bit version: sample at two octaves and combine
  uint8_t lo = inoise8(static_cast<uint16_t>(x >> 8));
  uint8_t hi = inoise8(static_cast<uint16_t>(x >> 4));
  return (static_cast<uint16_t>(hi) << 8) | lo;
}

}  // namespace wled_bridge
}  // namespace esphome
