#include "wled_fx_math.h"
#include "wled_color.h"

namespace esphome {
namespace wled_bridge {

// ============================================================================
// Perlin noise — faithful port of FastLED 3.6.0 `noise.cpp` (MIT).
//
// The previous implementation was a custom 1-D approximation that ignored the
// y/z arguments, so every WLED noise-based effect (Pacifica, Lake, Fill Noise,
// Noise 1-4, Noisepal, 2-D noise fields, …) rendered differently from WLED.
// This is a 1:1 transcription of FastLED's gradient noise so those effects match.
//
// FastLED's defaults are FASTLED_SCALE8_FIXED == 1 and FASTLED_NOISE_FIXED == 1,
// so the fade/scale helpers below carry FastLED's "+1" fixed-point rounding term.
// NOTE: the component-global scale8()/scale16() intentionally omit that term, so
// the noise code keeps its own FastLED-exact helpers here rather than reuse them.
// ============================================================================

namespace {

// FastLED permutation table `p[]` — 257 entries; the final byte duplicates p[0]
// so that P(x+1) with x==255 reads a valid slot (FastLED relies on this).
const uint8_t P_TABLE[257] = {
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
    67,  29,  24,  72,  243, 141, 128, 195, 78,  66,  215, 61,  156, 180, 151,
};

inline uint8_t pn(uint16_t i) {
  return P_TABLE[i];
}

// FastLED-exact fixed-point helpers (carry the "+1" term).
inline uint8_t n_scale8(uint8_t i, uint8_t s) {
  return static_cast<uint8_t>((static_cast<uint16_t>(i) * (1u + static_cast<uint16_t>(s))) >> 8);
}
inline uint16_t n_scale16(uint16_t i, uint16_t s) {
  return static_cast<uint16_t>((static_cast<uint32_t>(i) * (1u + static_cast<uint32_t>(s))) >> 16);
}

// ease8InOutQuad / ease16InOutQuad — FastLED's EASE when FASTLED_NOISE_FIXED == 1.
inline uint8_t ease8(uint8_t i) {
  uint8_t j = i;
  if (j & 0x80)
    j = 255 - j;
  uint8_t jj = n_scale8(j, j);
  uint8_t jj2 = jj << 1;
  if (i & 0x80)
    jj2 = 255 - jj2;
  return jj2;
}
inline uint16_t ease16(uint16_t i) {
  uint16_t j = i;
  if (j & 0x8000)
    j = 65535 - j;
  uint16_t jj = n_scale16(j, j);
  uint16_t jj2 = jj << 1;
  if (i & 0x8000)
    jj2 = 65535 - jj2;
  return jj2;
}

// Signed integer averages (FastLED avg7 / avg15).
inline int8_t avg7(int8_t i, int8_t j) {
  return static_cast<int8_t>((i >> 1) + (j >> 1) + (i & 0x1));
}
inline int16_t avg15(int16_t i, int16_t j) {
  return static_cast<int16_t>((i >> 1) + (j >> 1) + (i & 0x1));
}

// FastLED lerp7by8 / lerp15by16.
inline int8_t lerp7by8(int8_t a, int8_t b, uint8_t frac) {
  int8_t result;
  if (b > a) {
    uint8_t delta = static_cast<uint8_t>(b - a);
    uint8_t scaled = n_scale8(delta, frac);
    result = static_cast<int8_t>(a + scaled);
  } else {
    uint8_t delta = static_cast<uint8_t>(a - b);
    uint8_t scaled = n_scale8(delta, frac);
    result = static_cast<int8_t>(a - scaled);
  }
  return result;
}
inline int16_t lerp15by16(int16_t a, int16_t b, uint16_t frac) {
  int16_t result;
  if (b > a) {
    uint16_t delta = static_cast<uint16_t>(b - a);
    uint16_t scaled = n_scale16(delta, frac);
    result = static_cast<int16_t>(a + scaled);
  } else {
    uint16_t delta = static_cast<uint16_t>(a - b);
    uint16_t scaled = n_scale16(delta, frac);
    result = static_cast<int16_t>(a - scaled);
  }
  return result;
}

// ---- gradient functions (FastLED grad16 / grad8, active branches) ----
inline int16_t grad16(uint8_t hash, int16_t x, int16_t y, int16_t z) {
  hash = hash & 15;
  int16_t u = hash < 8 ? x : y;
  int16_t v = hash < 4 ? y : (hash == 12 || hash == 14 ? x : z);
  if (hash & 1)
    u = -u;
  if (hash & 2)
    v = -v;
  return avg15(u, v);
}
inline int16_t grad16(uint8_t hash, int16_t x, int16_t y) {
  hash = hash & 7;
  int16_t u, v;
  if (hash < 4) {
    u = x;
    v = y;
  } else {
    u = y;
    v = x;
  }
  if (hash & 1)
    u = -u;
  if (hash & 2)
    v = -v;
  return avg15(u, v);
}
inline int16_t grad16(uint8_t hash, int16_t x) {
  hash = hash & 15;
  int16_t u, v;
  if (hash > 8) {
    u = x;
    v = x;
  } else if (hash < 4) {
    u = x;
    v = 1;
  } else {
    u = 1;
    v = x;
  }
  if (hash & 1)
    u = -u;
  if (hash & 2)
    v = -v;
  return avg15(u, v);
}

inline int8_t grad8(uint8_t hash, int8_t x, int8_t y, int8_t z) {
  hash &= 0xF;
  int8_t u = (hash & 8) ? y : x;
  int8_t v = hash < 4 ? y : (hash == 12 || hash == 14 ? x : z);
  if (hash & 1)
    u = -u;
  if (hash & 2)
    v = -v;
  return avg7(u, v);
}
inline int8_t grad8(uint8_t hash, int8_t x, int8_t y) {
  int8_t u, v;
  if (hash & 4) {
    u = y;
    v = x;
  } else {
    u = x;
    v = y;
  }
  if (hash & 1)
    u = -u;
  if (hash & 2)
    v = -v;
  return avg7(u, v);
}
inline int8_t grad8(uint8_t hash, int8_t x) {
  int8_t u, v;
  if (hash & 8) {
    u = x;
    v = x;
  } else if (hash & 4) {
    u = 1;
    v = x;
  } else {
    u = x;
    v = 1;
  }
  if (hash & 1)
    u = -u;
  if (hash & 2)
    v = -v;
  return avg7(u, v);
}

// ---- 16-bit raw noise ----
int16_t inoise16_raw(uint32_t x, uint32_t y, uint32_t z) {
  uint8_t X = (x >> 16) & 0xFF;
  uint8_t Y = (y >> 16) & 0xFF;
  uint8_t Z = (z >> 16) & 0xFF;

  uint8_t A = pn(X) + Y;
  uint8_t AA = pn(A) + Z;
  uint8_t AB = pn(A + 1) + Z;
  uint8_t B = pn(X + 1) + Y;
  uint8_t BA = pn(B) + Z;
  uint8_t BB = pn(B + 1) + Z;

  uint16_t u = x & 0xFFFF;
  uint16_t v = y & 0xFFFF;
  uint16_t w = z & 0xFFFF;

  int16_t xx = (u >> 1) & 0x7FFF;
  int16_t yy = (v >> 1) & 0x7FFF;
  int16_t zz = (w >> 1) & 0x7FFF;
  uint16_t N = 0x8000L;

  u = ease16(u);
  v = ease16(v);
  w = ease16(w);

  int16_t X1 = lerp15by16(grad16(pn(AA), xx, yy, zz), grad16(pn(BA), xx - N, yy, zz), u);
  int16_t X2 = lerp15by16(grad16(pn(AB), xx, yy - N, zz), grad16(pn(BB), xx - N, yy - N, zz), u);
  int16_t X3 = lerp15by16(grad16(pn(AA + 1), xx, yy, zz - N), grad16(pn(BA + 1), xx - N, yy, zz - N), u);
  int16_t X4 = lerp15by16(grad16(pn(AB + 1), xx, yy - N, zz - N), grad16(pn(BB + 1), xx - N, yy - N, zz - N), u);

  int16_t Y1 = lerp15by16(X1, X2, v);
  int16_t Y2 = lerp15by16(X3, X4, v);

  return lerp15by16(Y1, Y2, w);
}

int16_t inoise16_raw(uint32_t x, uint32_t y) {
  uint8_t X = x >> 16;
  uint8_t Y = y >> 16;

  uint8_t A = pn(X) + Y;
  uint8_t AA = pn(A);
  uint8_t AB = pn(A + 1);
  uint8_t B = pn(X + 1) + Y;
  uint8_t BA = pn(B);
  uint8_t BB = pn(B + 1);

  uint16_t u = x & 0xFFFF;
  uint16_t v = y & 0xFFFF;

  int16_t xx = (u >> 1) & 0x7FFF;
  int16_t yy = (v >> 1) & 0x7FFF;
  uint16_t N = 0x8000L;

  u = ease16(u);
  v = ease16(v);

  int16_t X1 = lerp15by16(grad16(pn(AA), xx, yy), grad16(pn(BA), xx - N, yy), u);
  int16_t X2 = lerp15by16(grad16(pn(AB), xx, yy - N), grad16(pn(BB), xx - N, yy - N), u);

  return lerp15by16(X1, X2, v);
}

int16_t inoise16_raw(uint32_t x) {
  uint8_t X = x >> 16;

  uint8_t A = pn(X);
  uint8_t AA = pn(A);
  uint8_t B = pn(X + 1);
  uint8_t BA = pn(B);

  uint16_t u = x & 0xFFFF;
  int16_t xx = (u >> 1) & 0x7FFF;
  uint16_t N = 0x8000L;

  u = ease16(u);

  return lerp15by16(grad16(pn(AA), xx), grad16(pn(BA), xx - N), u);
}

// ---- 8-bit raw noise (output range -64 .. +64) ----
int8_t inoise8_raw(uint16_t x, uint16_t y, uint16_t z) {
  uint8_t X = x >> 8;
  uint8_t Y = y >> 8;
  uint8_t Z = z >> 8;

  uint8_t A = pn(X) + Y;
  uint8_t AA = pn(A) + Z;
  uint8_t AB = pn(A + 1) + Z;
  uint8_t B = pn(X + 1) + Y;
  uint8_t BA = pn(B) + Z;
  uint8_t BB = pn(B + 1) + Z;

  uint8_t u = x;
  uint8_t v = y;
  uint8_t w = z;

  int8_t xx = (static_cast<uint8_t>(x) >> 1) & 0x7F;
  int8_t yy = (static_cast<uint8_t>(y) >> 1) & 0x7F;
  int8_t zz = (static_cast<uint8_t>(z) >> 1) & 0x7F;
  uint8_t N = 0x80;

  u = ease8(u);
  v = ease8(v);
  w = ease8(w);

  int8_t X1 = lerp7by8(grad8(pn(AA), xx, yy, zz), grad8(pn(BA), xx - N, yy, zz), u);
  int8_t X2 = lerp7by8(grad8(pn(AB), xx, yy - N, zz), grad8(pn(BB), xx - N, yy - N, zz), u);
  int8_t X3 = lerp7by8(grad8(pn(AA + 1), xx, yy, zz - N), grad8(pn(BA + 1), xx - N, yy, zz - N), u);
  int8_t X4 = lerp7by8(grad8(pn(AB + 1), xx, yy - N, zz - N), grad8(pn(BB + 1), xx - N, yy - N, zz - N), u);

  int8_t Y1 = lerp7by8(X1, X2, v);
  int8_t Y2 = lerp7by8(X3, X4, v);

  return lerp7by8(Y1, Y2, w);
}

int8_t inoise8_raw(uint16_t x, uint16_t y) {
  uint8_t X = x >> 8;
  uint8_t Y = y >> 8;

  uint8_t A = pn(X) + Y;
  uint8_t AA = pn(A);
  uint8_t AB = pn(A + 1);
  uint8_t B = pn(X + 1) + Y;
  uint8_t BA = pn(B);
  uint8_t BB = pn(B + 1);

  uint8_t u = x;
  uint8_t v = y;

  int8_t xx = (static_cast<uint8_t>(x) >> 1) & 0x7F;
  int8_t yy = (static_cast<uint8_t>(y) >> 1) & 0x7F;
  uint8_t N = 0x80;

  u = ease8(u);
  v = ease8(v);

  int8_t X1 = lerp7by8(grad8(pn(AA), xx, yy), grad8(pn(BA), xx - N, yy), u);
  int8_t X2 = lerp7by8(grad8(pn(AB), xx, yy - N), grad8(pn(BB), xx - N, yy - N), u);

  return lerp7by8(X1, X2, v);
}

int8_t inoise8_raw(uint16_t x) {
  uint8_t X = x >> 8;

  uint8_t A = pn(X);
  uint8_t AA = pn(A);
  uint8_t B = pn(X + 1);
  uint8_t BA = pn(B);

  uint8_t u = x;
  int8_t xx = (static_cast<uint8_t>(x) >> 1) & 0x7F;
  uint8_t N = 0x80;

  u = ease8(u);

  return lerp7by8(grad8(pn(AA), xx), grad8(pn(BA), xx - N), u);
}

}  // namespace

// ---- public API: scaled to 0..255 / 0..65535 like FastLED ----
uint8_t inoise8(uint16_t x, uint16_t y, uint16_t z) {
  int8_t n = inoise8_raw(x, y, z);  // -64..+64
  n += 64;  //   0..128
  return qadd8(static_cast<uint8_t>(n), static_cast<uint8_t>(n));  // 0..255
}
uint8_t inoise8(uint16_t x, uint16_t y) {
  int8_t n = inoise8_raw(x, y);
  n += 64;
  return qadd8(static_cast<uint8_t>(n), static_cast<uint8_t>(n));
}
uint8_t inoise8(uint16_t x) {
  int8_t n = inoise8_raw(x);
  n += 64;
  return qadd8(static_cast<uint8_t>(n), static_cast<uint8_t>(n));
}

uint16_t inoise16(uint32_t x, uint32_t y, uint32_t z) {
  int32_t ans = inoise16_raw(x, y, z);
  ans = ans + 19052L;
  uint32_t pan = ans;
  pan *= 440L;
  return static_cast<uint16_t>(pan >> 8);
}
uint16_t inoise16(uint32_t x, uint32_t y) {
  int32_t ans = inoise16_raw(x, y);
  ans = ans + 17308L;
  uint32_t pan = ans;
  pan *= 484L;
  return static_cast<uint16_t>(pan >> 8);
}
uint16_t inoise16(uint32_t x) {
  return static_cast<uint16_t>((static_cast<uint32_t>(static_cast<int32_t>(inoise16_raw(x)) + 17308L)) << 1);
}

}  // namespace wled_bridge
}  // namespace esphome
