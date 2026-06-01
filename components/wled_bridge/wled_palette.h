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
  PAL_C9 = 19,
  PAL_ATLANTICA = 20,
  PAL_C9_2 = 21,
  PAL_C9_NEW = 22,
  PAL_TEMPERATURE = 23,
  PAL_CANDY = 24,
  PAL_TOXY_REAF = 25,
  PAL_FAIRY_REAF = 26,
  PAL_SEMI_BLUE = 27,
  PAL_PINK_CANDY = 28,
  PAL_RED_REAF = 29,
  PAL_YELBLU_HOT = 30,
  PAL_SUNSET_REAL = 31,
  PAL_RIVENDELL = 32,
  PAL_RED_BLUE = 33,
  PAL_TERTIARY = 34,
  PAL_FIRE = 35,
  PAL_ICEFIRE = 36,
  PAL_CYANE = 37,
  PAL_LIGHT_PINK = 38,
  PAL_AURORA2 = 39,
  PAL_RETRO_CLOWN = 40,
  PAL_AUTUMN = 41,
  PAL_MAGENTA = 42,
  PAL_MAGRED = 43,
  PAL_YELMAG = 44,
  PAL_YELBLU = 45,
  PAL_ORANGE_TEAL = 46,
  PAL_AQUA_FLASH = 47,
  PAL_LITE_LIGHT = 48,
  PAL_RED_FLASH = 49,
  PAL_BLINK_RED = 50,
  PAL_RED_SHIFT = 51,
  PAL_RED_TIDE = 52,
  PAL_CANDY2 = 53,
};

static constexpr size_t WLED_PALETTE_COUNT = 54;
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
extern const WLEDPalette16 PAL_DATA_C9;
extern const WLEDPalette16 PAL_DATA_ATLANTICA;
extern const WLEDPalette16 PAL_DATA_C9_2;
extern const WLEDPalette16 PAL_DATA_C9_NEW;
extern const WLEDPalette16 PAL_DATA_TEMPERATURE;
extern const WLEDPalette16 PAL_DATA_CANDY;
extern const WLEDPalette16 PAL_DATA_TOXY_REAF;
extern const WLEDPalette16 PAL_DATA_FAIRY_REAF;
extern const WLEDPalette16 PAL_DATA_SEMI_BLUE;
extern const WLEDPalette16 PAL_DATA_PINK_CANDY;
extern const WLEDPalette16 PAL_DATA_RED_REAF;
extern const WLEDPalette16 PAL_DATA_YELBLU_HOT;
extern const WLEDPalette16 PAL_DATA_SUNSET_REAL;
extern const WLEDPalette16 PAL_DATA_RIVENDELL;
extern const WLEDPalette16 PAL_DATA_RED_BLUE;
extern const WLEDPalette16 PAL_DATA_TERTIARY;
extern const WLEDPalette16 PAL_DATA_FIRE;
extern const WLEDPalette16 PAL_DATA_ICEFIRE;
extern const WLEDPalette16 PAL_DATA_CYANE;
extern const WLEDPalette16 PAL_DATA_LIGHT_PINK;
extern const WLEDPalette16 PAL_DATA_AURORA2;
extern const WLEDPalette16 PAL_DATA_RETRO_CLOWN;
extern const WLEDPalette16 PAL_DATA_AUTUMN;
extern const WLEDPalette16 PAL_DATA_MAGENTA;
extern const WLEDPalette16 PAL_DATA_MAGRED;
extern const WLEDPalette16 PAL_DATA_YELMAG;
extern const WLEDPalette16 PAL_DATA_YELBLU;
extern const WLEDPalette16 PAL_DATA_ORANGE_TEAL;
extern const WLEDPalette16 PAL_DATA_AQUA_FLASH;
extern const WLEDPalette16 PAL_DATA_LITE_LIGHT;
extern const WLEDPalette16 PAL_DATA_RED_FLASH;
extern const WLEDPalette16 PAL_DATA_BLINK_RED;
extern const WLEDPalette16 PAL_DATA_RED_SHIFT;
extern const WLEDPalette16 PAL_DATA_RED_TIDE;
extern const WLEDPalette16 PAL_DATA_CANDY2;

}  // namespace wled_bridge
}  // namespace esphome
