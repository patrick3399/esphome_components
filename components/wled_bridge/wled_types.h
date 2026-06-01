#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <esp_random.h>
#include <esp_heap_caps.h>

namespace esphome {
namespace wled_bridge {

// ---------- timing constants ----------
static constexpr uint32_t WLED_FPS = 42;
static constexpr uint32_t FRAMETIME_MS = 1000u / WLED_FPS;

// ---------- PRNG — bit-exact copy of WLED's prng.h ----------
class PRNG {
 public:
  explicit PRNG(uint16_t s = 0x1234) : seed_(s) {}
  void set_seed(uint16_t s) {
    seed_ = s;
  }
  uint16_t get_seed() const {
    return seed_;
  }

  uint16_t random16() {
    seed_ = seed_ * 3001u + 31683u;
    seed_ ^= (seed_ >> 7);
    return seed_;
  }
  uint16_t random16(uint16_t lim) {
    return static_cast<uint16_t>((static_cast<uint32_t>(random16()) * lim) >> 16);
  }
  uint16_t random16(uint16_t lo, uint16_t hi) {
    uint16_t d = hi - lo;
    return random16(d) + lo;
  }
  uint8_t random8() {
    return static_cast<uint8_t>(random16());
  }
  uint8_t random8(uint8_t lim) {
    return static_cast<uint8_t>((static_cast<uint16_t>(random8()) * lim) >> 8);
  }
  uint8_t random8(uint8_t lo, uint8_t hi) {
    uint8_t d = hi - lo;
    return random8(d) + lo;
  }

 private:
  uint16_t seed_;
};

// File-static PRNG seeded with hardware entropy — defined once in wled_effects.cpp
extern PRNG g_prng;

// Convenience wrappers matching WLED's hw_random8 / hw_random16
inline uint8_t hw_random8() {
  return g_prng.random8();
}
inline uint8_t hw_random8(uint8_t lim) {
  return g_prng.random8(lim);
}
inline uint8_t hw_random8(uint8_t lo, uint8_t hi) {
  return g_prng.random8(lo, hi);
}
inline uint16_t hw_random16() {
  return g_prng.random16();
}
inline uint16_t hw_random16(uint16_t lim) {
  return g_prng.random16(lim);
}

// ---------- per-segment runtime state ----------
struct SegmentState {
  uint32_t step{0};  // generic step/iteration counter
  uint32_t call{0};  // increments each rendered frame
  uint16_t aux0{0};  // effect-specific aux counter 0
  uint16_t aux1{0};  // effect-specific aux counter 1
  uint8_t *data{nullptr};
  size_t data_len{0};

  // Allocate effect-private data (PSRAM first, internal heap fallback).
  // Returns true on success; if already allocated with same size, noop true.
  bool allocate_data(size_t len) {
    if (data != nullptr && data_len == len)
      return true;
    free_data();
    data = static_cast<uint8_t *>(heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (data == nullptr)
      data = static_cast<uint8_t *>(malloc(len));
    if (data == nullptr)
      return false;
    memset(data, 0, len);
    data_len = len;
    return true;
  }

  void free_data() {
    if (data != nullptr) {
      free(data);
      data = nullptr;
      data_len = 0;
    }
  }

  ~SegmentState() {
    free_data();
  }
};

// ---------- per-frame immutable effect parameters ----------
struct EffectParams {
  uint8_t speed{128};
  uint8_t intensity{128};
  uint8_t custom1{128};
  uint8_t custom2{128};
  uint8_t custom3{16};
  bool check1{false};
  bool check2{false};
  bool check3{false};
  uint32_t colors[3]{0xFFAA00, 0x000000, 0x000000};
  uint8_t palette_id{0};
};

}  // namespace wled_bridge
}  // namespace esphome
