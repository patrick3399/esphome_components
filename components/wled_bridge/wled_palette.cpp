#include "wled_palette.h"

namespace esphome {
namespace wled_bridge {

// ---------- palette interpolation ----------
uint32_t palette_color(const WLEDPalette16 &pal, uint8_t index, uint8_t brightness) {
  // Find the two stops that bracket `index`
  const PaletteEntry *lo = &pal[0];
  const PaletteEntry *hi = &pal[15];

  for (int i = 0; i < 15; i++) {
    if (pal[i].pos <= index && pal[i + 1].pos > index) {
      lo = &pal[i];
      hi = &pal[i + 1];
      break;
    }
  }

  uint8_t blend;
  if (hi->pos == lo->pos) {
    blend = 0;
  } else {
    blend = static_cast<uint8_t>(255u * (static_cast<uint16_t>(index - lo->pos)) /
                                 (static_cast<uint16_t>(hi->pos - lo->pos)));
  }

  uint8_t r = lerp8by8(lo->r, hi->r, blend);
  uint8_t g = lerp8by8(lo->g, hi->g, blend);
  uint8_t b = lerp8by8(lo->b, hi->b, blend);

  if (brightness != 255) {
    r = scale8(r, brightness);
    g = scale8(g, brightness);
    b = scale8(b, brightness);
  }

  return RGBW32(r, g, b, 0);
}

// ---------- palette data (WLED-sourced gradient stops) ----------

// Rainbow — full HSV rainbow
const WLEDPalette16 PAL_DATA_RAINBOW = {{
    {0, 255, 0, 0},  // red
    {16, 255, 128, 0},  // orange
    {32, 255, 255, 0},  // yellow
    {64, 0, 255, 0},  // green
    {96, 0, 255, 128},  // spring
    {112, 0, 255, 255},  // cyan
    {128, 0, 128, 255},  // azure
    {144, 0, 0, 255},  // blue
    {160, 64, 0, 255},  // violet
    {176, 128, 0, 255},  // purple
    {192, 255, 0, 255},  // magenta
    {208, 255, 0, 128},  // rose
    {224, 255, 0, 64},  // crimson
    {240, 255, 0, 0},  // red again
    {248, 255, 0, 0},
    {255, 255, 0, 0},
}};

// Rainbow Bands — distinct rainbow segments
const WLEDPalette16 PAL_DATA_RAINBOW_BANDS = {{
    {0, 255, 0, 0},
    {14, 255, 0, 0},
    {15, 0, 255, 0},
    {29, 0, 255, 0},
    {30, 0, 0, 255},
    {44, 0, 0, 255},
    {45, 255, 255, 0},
    {59, 255, 255, 0},
    {60, 0, 255, 255},
    {74, 0, 255, 255},
    {75, 255, 0, 255},
    {89, 255, 0, 255},
    {90, 255, 128, 0},
    {127, 255, 128, 0},
    {128, 255, 0, 0},
    {255, 255, 0, 0},
}};

// Party — warm reds/pinks/purples
const WLEDPalette16 PAL_DATA_PARTY = {{
    {0, 85, 0, 255},
    {16, 132, 0, 255},
    {32, 192, 0, 255},
    {48, 255, 0, 233},
    {64, 255, 0, 111},
    {80, 255, 0, 0},
    {96, 255, 55, 0},
    {112, 255, 200, 0},
    {128, 255, 255, 0},
    {144, 255, 115, 0},
    {160, 255, 0, 0},
    {176, 255, 0, 80},
    {192, 255, 0, 200},
    {208, 255, 55, 255},
    {240, 170, 0, 255},
    {255, 85, 0, 255},
}};

// Ocean — blues and teals
const WLEDPalette16 PAL_DATA_OCEAN = {{
    {0, 0, 0, 67},
    {32, 0, 0, 84},
    {64, 0, 0, 112},
    {96, 0, 5, 165},
    {128, 0, 79, 213},
    {160, 0, 146, 185},
    {192, 0, 182, 181},
    {224, 0, 218, 213},
    {240, 0, 255, 255},
    {245, 50, 255, 255},
    {250, 100, 255, 255},
    {252, 150, 255, 255},
    {253, 200, 255, 255},
    {254, 230, 255, 255},
    {255, 255, 255, 255},
    {255, 255, 255, 255},
}};

// Lava — reds and oranges
const WLEDPalette16 PAL_DATA_LAVA = {{
    {0, 0, 0, 0},
    {16, 18, 0, 0},
    {32, 113, 0, 0},
    {64, 142, 3, 1},
    {96, 175, 17, 1},
    {128, 213, 44, 2},
    {160, 255, 82, 4},
    {192, 255, 115, 4},
    {208, 255, 156, 4},
    {224, 255, 203, 24},
    {240, 255, 255, 100},
    {248, 255, 255, 200},
    {253, 255, 255, 240},
    {254, 255, 255, 255},
    {255, 255, 255, 255},
    {255, 255, 255, 255},
}};

// Forest — greens
const WLEDPalette16 PAL_DATA_FOREST = {{
    {0, 0, 14, 0},
    {16, 0, 25, 0},
    {32, 0, 36, 0},
    {64, 0, 56, 0},
    {96, 0, 85, 0},
    {128, 0, 128, 0},
    {144, 34, 128, 0},
    {160, 68, 128, 0},
    {176, 102, 150, 0},
    {192, 136, 182, 0},
    {208, 170, 202, 0},
    {224, 190, 220, 0},
    {240, 210, 240, 0},
    {248, 230, 250, 0},
    {253, 250, 255, 0},
    {255, 255, 255, 0},
}};

// Sunset — warm gradient
const WLEDPalette16 PAL_DATA_SUNSET = {{
    {0, 120, 0, 0},
    {22, 179, 22, 0},
    {51, 255, 104, 0},
    {85, 167, 22, 18},
    {135, 100, 0, 103},
    {198, 16, 0, 130},
    {255, 0, 0, 160},
    {255, 0, 0, 160},
    {255, 0, 0, 160},
    {255, 0, 0, 160},
    {255, 0, 0, 160},
    {255, 0, 0, 160},
    {255, 0, 0, 160},
    {255, 0, 0, 160},
    {255, 0, 0, 160},
    {255, 0, 0, 160},
}};

// Heat — fire palette
const WLEDPalette16 PAL_DATA_HEAT = {{
    {0, 0, 0, 0},
    {32, 128, 0, 0},
    {64, 255, 0, 0},
    {96, 255, 45, 0},
    {128, 255, 128, 0},
    {160, 255, 200, 0},
    {192, 255, 255, 0},
    {224, 255, 255, 128},
    {240, 255, 255, 200},
    {252, 255, 255, 255},
    {253, 255, 255, 255},
    {254, 255, 255, 255},
    {255, 255, 255, 255},
    {255, 255, 255, 255},
    {255, 255, 255, 255},
    {255, 255, 255, 255},
}};

// Tiamat — neon blues and purples
const WLEDPalette16 PAL_DATA_TIAMAT = {{
    {0, 1, 2, 14},
    {16, 2, 5, 35},
    {32, 13, 135, 92},
    {48, 43, 255, 193},
    {64, 52, 235, 231},
    {80, 101, 165, 227},
    {96, 142, 120, 239},
    {112, 171, 84, 232},
    {128, 224, 20, 229},
    {144, 255, 0, 220},
    {160, 255, 60, 186},
    {176, 249, 136, 117},
    {192, 255, 196, 66},
    {208, 255, 255, 0},
    {224, 255, 255, 100},
    {255, 255, 255, 255},
}};

// April Night
const WLEDPalette16 PAL_DATA_APRIL_NIGHT = {{
    {0, 1, 5, 45},
    {10, 1, 5, 45},
    {11, 5, 5, 45},
    {40, 5, 5, 45},
    {41, 5, 5, 45},
    {61, 5, 5, 45},
    {62, 255, 55, 0},
    {74, 255, 55, 0},
    {75, 5, 5, 45},
    {93, 5, 5, 45},
    {94, 255, 215, 0},
    {99, 255, 215, 0},
    {100, 5, 5, 45},
    {200, 5, 5, 45},
    {201, 255, 55, 0},
    {255, 255, 55, 0},
}};

// Orangery
const WLEDPalette16 PAL_DATA_ORANGERY = {{
    {0, 255, 95, 0},
    {32, 255, 60, 0},
    {64, 255, 30, 0},
    {96, 200, 10, 0},
    {128, 165, 5, 0},
    {160, 135, 3, 0},
    {192, 100, 2, 0},
    {224, 75, 1, 0},
    {240, 50, 0, 0},
    {248, 30, 0, 0},
    {250, 20, 0, 0},
    {252, 10, 0, 0},
    {253, 5, 0, 0},
    {254, 2, 0, 0},
    {255, 1, 0, 0},
    {255, 0, 0, 0},
}};

// Sakura — pink blossoms
const WLEDPalette16 PAL_DATA_SAKURA = {{
    {0, 196, 19, 10},
    {32, 255, 69, 45},
    {64, 255, 90, 60},
    {96, 255, 120, 95},
    {128, 255, 160, 130},
    {160, 255, 195, 170},
    {192, 255, 220, 200},
    {224, 255, 240, 230},
    {240, 255, 250, 245},
    {245, 255, 255, 250},
    {250, 255, 255, 255},
    {252, 255, 255, 255},
    {253, 255, 255, 255},
    {254, 255, 255, 255},
    {255, 255, 255, 255},
    {255, 255, 255, 255},
}};

// Aurora — northern lights
const WLEDPalette16 PAL_DATA_AURORA = {{
    {0, 17, 177, 13},
    {32, 30, 200, 70},
    {64, 50, 220, 120},
    {96, 25, 200, 160},
    {128, 10, 170, 200},
    {160, 10, 130, 220},
    {192, 10, 100, 230},
    {224, 30, 70, 255},
    {240, 50, 40, 255},
    {248, 30, 20, 220},
    {252, 10, 5, 190},
    {253, 5, 2, 160},
    {254, 2, 0, 130},
    {255, 0, 0, 100},
    {255, 0, 0, 80},
    {255, 0, 0, 60},
}};

// ---------- palette info table ----------
const PaletteInfo WLED_PALETTES[WLED_PALETTE_COUNT] = {
    {/* 0  */ "Default", nullptr},
    {/* 1  */ "Random Cycle", nullptr},
    {/* 2  */ "Primary Color", nullptr},
    {/* 3  */ "(Reserved)", nullptr},
    {/* 4  */ "(Reserved)", nullptr},
    {/* 5  */ "(Reserved)", nullptr},
    {/* 6  */ "Rainbow", &PAL_DATA_RAINBOW},
    {/* 7  */ "Rainbow Bands", &PAL_DATA_RAINBOW_BANDS},
    {/* 8  */ "Party", &PAL_DATA_PARTY},
    {/* 9  */ "Ocean", &PAL_DATA_OCEAN},
    {/* 10 */ "Lava", &PAL_DATA_LAVA},
    {/* 11 */ "Forest", &PAL_DATA_FOREST},
    {/* 12 */ "Sunset", &PAL_DATA_SUNSET},
    {/* 13 */ "Heat", &PAL_DATA_HEAT},
    {/* 14 */ "Tiamat", &PAL_DATA_TIAMAT},
    {/* 15 */ "April Night", &PAL_DATA_APRIL_NIGHT},
    {/* 16 */ "Orangery", &PAL_DATA_ORANGERY},
    {/* 17 */ "Sakura", &PAL_DATA_SAKURA},
    {/* 18 */ "Aurora", &PAL_DATA_AURORA},
};

}  // namespace wled_bridge
}  // namespace esphome
