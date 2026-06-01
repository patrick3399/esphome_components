#pragma once
#include <stdint.h>
#include <stddef.h>
#include <array>
#include "wled_color.h"

namespace esphome {
namespace wled_bridge {

// ---------- palette types ----------
// Each palette is 16 gradient stops (index 0..255 interpolated between stops).
struct PaletteEntry {
  uint8_t pos;  // 0..255 position along the gradient
  uint8_t r, g, b;
};

using WLEDPalette16 = std::array<PaletteEntry, 16>;

// ---------- palette lookup ----------
// Interpolates between stops in a WLEDPalette16.
// index: 0..255 position along the gradient
// brightness: 0..255 scale applied after interpolation
uint32_t palette_color(const WLEDPalette16 &pal, uint8_t index, uint8_t brightness = 255);

// ---------- built-in palette table ----------
struct PaletteInfo {
  const char *name;
  const WLEDPalette16 *data;  // nullptr for dynamic palettes (Default/Solid, Random)
};

// Palette IDs matching WLED's numbering
enum PaletteID : uint8_t {
  PAL_DEFAULT = 0,  // solid segment color (dynamic, no table)
  PAL_RANDOM = 1,  // random cycling (dynamic, no table)
  PAL_PRIMARY = 2,  // based on seg color 0 (dynamic, no table)
  PAL_RAINBOW = 6,
  PAL_RAINBOW_BANDS = 7,
  PAL_PARTY = 8,
  PAL_OCEAN = 9,
  PAL_LAVA = 10,
  PAL_FOREST = 11,
  PAL_SUNSET = 12,
  PAL_HEAT = 13,
  PAL_TIAMAT = 14,
  PAL_APRIL_NIGHT = 15,
  PAL_ORANGERY = 16,
  PAL_SAKURA = 17,
  PAL_AURORA = 18,
};

static constexpr size_t WLED_PALETTE_COUNT = 19;
extern const PaletteInfo WLED_PALETTES[WLED_PALETTE_COUNT];

// Palette data tables (defined in wled_palette.cpp)
extern const WLEDPalette16 PAL_DATA_RAINBOW;
extern const WLEDPalette16 PAL_DATA_RAINBOW_BANDS;
extern const WLEDPalette16 PAL_DATA_PARTY;
extern const WLEDPalette16 PAL_DATA_OCEAN;
extern const WLEDPalette16 PAL_DATA_LAVA;
extern const WLEDPalette16 PAL_DATA_FOREST;
extern const WLEDPalette16 PAL_DATA_SUNSET;
extern const WLEDPalette16 PAL_DATA_HEAT;
extern const WLEDPalette16 PAL_DATA_TIAMAT;
extern const WLEDPalette16 PAL_DATA_APRIL_NIGHT;
extern const WLEDPalette16 PAL_DATA_ORANGERY;
extern const WLEDPalette16 PAL_DATA_SAKURA;
extern const WLEDPalette16 PAL_DATA_AURORA;

}  // namespace wled_bridge
}  // namespace esphome
