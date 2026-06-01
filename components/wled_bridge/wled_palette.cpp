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

// C9 — classic C9 Christmas bulb colors (WLED: C9_gp, palette #48/35)
// Source: 4 stops padded to 16; red/amber/green/blue segments
const WLEDPalette16 PAL_DATA_C9 = {{
    {0, 184, 4, 0},  // red
    {15, 184, 4, 0},
    {30, 184, 4, 0},
    {60, 184, 4, 0},
    {65, 144, 44, 2},  // amber
    {80, 144, 44, 2},
    {100, 144, 44, 2},
    {125, 144, 44, 2},
    {130, 4, 96, 2},  // green
    {150, 4, 96, 2},
    {170, 4, 96, 2},
    {190, 4, 96, 2},
    {195, 7, 7, 88},  // blue
    {215, 7, 7, 88},
    {235, 7, 7, 88},
    {255, 7, 7, 88},
}};

// Atlantica — deep ocean blues and greens (WLED: Atlantica_gp, palette #51/38)
// Source: 6 stops resampled to 16
const WLEDPalette16 PAL_DATA_ATLANTICA = {{
    {0, 0, 28, 112},  // deep navy
    {17, 0, 28, 112},
    {34, 16, 62, 183},
    {50, 32, 96, 255},  // bright blue
    {67, 16, 169, 150},
    {83, 0, 243, 45},
    {100, 0, 243, 45},  // aqua green
    {117, 6, 169, 63},
    {133, 12, 95, 82},  // teal
    {150, 12, 95, 82},
    {167, 18, 142, 88},
    {183, 25, 190, 95},  // bright green
    {200, 25, 190, 95},
    {218, 32, 180, 87},
    {236, 40, 170, 80},
    {255, 40, 170, 80},
}};

// C9 2 — C9 variant with green/blue/red/amber/yellow (WLED: C9_2_gp, palette #52/39)
// Source: 10 stops resampled to 16
const WLEDPalette16 PAL_DATA_C9_2 = {{
    {0, 6, 126, 2},  // green
    {22, 6, 126, 2},
    {45, 6, 126, 2},
    {46, 4, 30, 114},  // blue
    {67, 4, 30, 114},
    {90, 4, 30, 114},
    {91, 255, 5, 0},  // red
    {112, 255, 5, 0},
    {135, 255, 5, 0},
    {136, 196, 57, 2},  // amber
    {157, 196, 57, 2},
    {180, 196, 57, 2},
    {181, 137, 85, 2},  // yellow
    {205, 137, 85, 2},
    {230, 137, 85, 2},
    {255, 137, 85, 2},
}};

// C9 New — brighter C9 with less purple blue (WLED: C9_new_gp, palette #53/40)
// Source: 8 stops resampled to 16
const WLEDPalette16 PAL_DATA_C9_NEW = {{
    {0, 255, 5, 0},  // red
    {17, 255, 5, 0},
    {34, 255, 5, 0},
    {60, 255, 5, 0},
    {61, 196, 57, 2},  // amber
    {90, 196, 57, 2},
    {120, 196, 57, 2},
    {121, 6, 126, 2},  // green
    {150, 6, 126, 2},
    {165, 6, 126, 2},
    {180, 6, 126, 2},
    {181, 4, 30, 114},  // blue
    {205, 4, 30, 114},
    {225, 4, 30, 114},
    {245, 4, 30, 114},
    {255, 4, 30, 114},
}};

// Temperature — blue-to-yellow-to-red temperature scale (WLED: temperature_gp, palette #54/41)
// Source: 18 stops resampled to 16 (uniform spacing)
const WLEDPalette16 PAL_DATA_TEMPERATURE = {{
    {0, 20, 92, 171},  // cool blue
    {17, 6, 142, 211},
    {34, 16, 181, 239},
    {51, 86, 204, 200},
    {68, 182, 229, 125},
    {85, 196, 230, 63},
    {102, 241, 240, 22},
    {119, 254, 222, 30},
    {136, 251, 199, 4},
    {153, 247, 157, 9},
    {170, 243, 114, 15},
    {187, 230, 72, 22},
    {204, 213, 30, 29},
    {221, 181, 34, 32},
    {238, 151, 38, 35},
    {255, 151, 38, 35},
}};

// Candy — sweet yellow-orange-purple gradient (WLED: candy_gp, palette #57/44)
// Source: 5 stops resampled to 16
const WLEDPalette16 PAL_DATA_CANDY = {{
    {0, 243, 242, 23},  // yellow
    {15, 242, 168, 38},  // orange
    {34, 218, 121, 60},
    {51, 195, 74, 82},
    {68, 173, 48, 104},
    {85, 152, 37, 117},
    {102, 142, 21, 151},  // purple start
    {119, 133, 21, 151},
    {136, 121, 21, 151},
    {153, 111, 21, 151},
    {170, 100, 22, 150},
    {187, 88, 22, 145},
    {198, 74, 22, 150},
    {220, 40, 12, 135},
    {238, 18, 3, 123},
    {255, 0, 0, 117},
}};

// Toxy Reaf — cyan-to-magenta two-stop gradient (WLED: toxy_reaf_gp, palette #58/45)
// Source: 2 stops expanded to 16
const WLEDPalette16 PAL_DATA_TOXY_REAF = {{
    {0, 2, 239, 126},
    {17, 11, 224, 133},
    {34, 20, 209, 140},
    {51, 29, 194, 147},
    {68, 38, 179, 154},
    {85, 47, 164, 161},
    {102, 56, 149, 168},
    {119, 65, 134, 175},
    {136, 74, 119, 182},
    {153, 83, 104, 188},
    {170, 92, 89, 195},
    {187, 101, 74, 202},
    {204, 110, 59, 209},
    {221, 119, 50, 213},
    {238, 132, 42, 215},
    {255, 145, 35, 217},
}};

// Fairy Reaf — pink-to-cyan gradient (WLED: fairy_reaf_gp, palette #59/46)
// Source: 4 stops resampled to 16
const WLEDPalette16 PAL_DATA_FAIRY_REAF = {{
    {0, 220, 19, 187},  // hot pink
    {20, 205, 24, 190},
    {40, 186, 31, 196},
    {60, 160, 42, 202},
    {80, 130, 58, 208},
    {100, 100, 80, 213},
    {120, 70, 108, 216},
    {140, 46, 143, 219},
    {160, 12, 225, 219},  // cyan
    {175, 44, 231, 220},
    {190, 77, 235, 221},
    {205, 109, 238, 222},
    {219, 147, 241, 222},  // lighter
    {232, 185, 246, 228},
    {245, 222, 250, 240},
    {255, 255, 255, 255},
}};

// Semi Blue — dark blue-purple (WLED: semi_blue_gp, palette #60/47)
// Source: 9 stops resampled to 16
const WLEDPalette16 PAL_DATA_SEMI_BLUE = {{
    {0, 0, 0, 0},
    {12, 24, 4, 38},
    {24, 40, 6, 62},
    {36, 48, 7, 73},
    {53, 55, 8, 84},
    {67, 49, 28, 121},
    {80, 43, 48, 159},
    {100, 37, 69, 198},
    {119, 31, 89, 237},
    {133, 43, 74, 202},
    {145, 50, 59, 166},
    {166, 61, 45, 132},
    {186, 71, 30, 98},
    {210, 51, 22, 71},
    {233, 31, 15, 45},
    {255, 0, 0, 0},
}};

// Pink Candy — pink/white/blue confection (WLED: pink_candy_gp, palette #61/48)
// Source: 7 stops resampled to 16
const WLEDPalette16 PAL_DATA_PINK_CANDY = {{
    {0, 255, 255, 255},  // white
    {18, 200, 200, 255},
    {34, 152, 140, 255},
    {45, 50, 64, 255},  // deep blue
    {56, 100, 40, 220},
    {68, 150, 24, 202},
    {80, 191, 18, 193},
    {96, 242, 16, 186},  // magenta
    {112, 242, 16, 186},
    {126, 248, 135, 220},
    {140, 255, 255, 255},  // white
    {148, 248, 135, 220},
    {155, 242, 16, 186},
    {176, 179, 14, 176},
    {196, 116, 13, 166},
    {255, 255, 255, 255},
}};

// Red Reaf — navy-to-aqua-to-red (WLED: red_reaf_gp / bhw1_w00t, palette #62/49)
// Source: 4 stops resampled to 16
const WLEDPalette16 PAL_DATA_RED_REAF = {{
    {0, 36, 68, 114},  // navy
    {26, 55, 93, 143},
    {52, 75, 119, 171},
    {78, 97, 145, 198},
    {104, 149, 195, 248},  // aqua
    {120, 185, 160, 197},
    {136, 212, 120, 140},
    {152, 230, 78, 85},
    {168, 240, 36, 38},
    {180, 252, 10, 14},
    {188, 255, 0, 0},  // pure red
    {209, 215, 4, 3},
    {222, 188, 6, 5},
    {235, 161, 9, 7},
    {245, 127, 11, 8},
    {255, 94, 14, 9},
}};

// Yelblu Hot — dark blue to yellow hot gradient (WLED: yelblu_hot_gp, palette #64/51)
// Source: 7 stops resampled to 16
const WLEDPalette16 PAL_DATA_YELBLU_HOT = {{
    {0, 43, 30, 57},  // dark purple
    {19, 52, 20, 76},
    {38, 60, 12, 92},
    {58, 73, 0, 119},  // blue-purple
    {75, 80, 0, 97},
    {90, 85, 0, 86},
    {107, 87, 0, 80},
    {122, 87, 0, 74},  // dark blue
    {139, 120, 22, 57},
    {158, 154, 40, 44},
    {176, 180, 68, 34},
    {183, 197, 57, 22},
    {202, 208, 87, 27},
    {219, 218, 117, 27},
    {237, 232, 147, 27},
    {255, 246, 247, 27},  // warm yellow
}};

// Sunset Real — warm red/orange/purple sunset (WLED: Sunset_Real_gp, palette #13/00)
// Source: 7 stops resampled to 16
const WLEDPalette16 PAL_DATA_SUNSET_REAL = {{
    {0, 181, 0, 0},  // deep red
    {11, 197, 38, 0},
    {22, 218, 85, 0},  // orange
    {36, 240, 128, 0},
    {51, 255, 170, 0},  // yellow-orange
    {68, 233, 128, 38},
    {85, 211, 85, 77},  // salmon
    {102, 195, 57, 100},
    {119, 178, 36, 122},
    {135, 167, 0, 169},  // purple
    {152, 143, 0, 177},
    {168, 120, 0, 184},
    {184, 97, 0, 186},
    {198, 73, 0, 188},  // dark purple
    {227, 36, 0, 198},
    {255, 0, 0, 207},
}};

// Rivendell — elven green-grey mist (WLED: es_rivendell_15_gp, palette #14/01)
// Source: 5 stops resampled to 16
const WLEDPalette16 PAL_DATA_RIVENDELL = {{
    {0, 24, 69, 44},  // mossy green
    {20, 34, 77, 50},
    {40, 45, 85, 56},
    {60, 56, 93, 62},
    {80, 67, 99, 67},
    {101, 73, 105, 70},  // silver-grey green
    {120, 90, 113, 79},
    {140, 107, 122, 87},
    {160, 123, 131, 92},
    {165, 129, 140, 97},  // warm grey
    {185, 149, 158, 114},
    {205, 168, 176, 131},
    {225, 188, 193, 149},
    {242, 200, 204, 166},  // warm white-grey
    {248, 200, 204, 166},
    {255, 200, 204, 166},
}};

// Red & Blue — purple/magenta/crimson (WLED: rgi_15_gp, palette #16/03)
// Source: 9 stops resampled to 16
const WLEDPalette16 PAL_DATA_RED_BLUE = {{
    {0, 41, 14, 99},  // deep purple
    {18, 84, 19, 87},
    {31, 128, 24, 74},  // medium purple
    {47, 177, 29, 62},
    {63, 227, 34, 50},  // crimson
    {79, 179, 32, 63},
    {95, 132, 31, 76},  // dark purple
    {111, 89, 30, 89},
    {127, 47, 29, 102},  // deep blue-purple
    {143, 78, 38, 101},
    {159, 109, 47, 101},  // slate purple
    {175, 142, 56, 100},
    {191, 176, 66, 100},  // violet
    {207, 152, 61, 102},
    {223, 129, 57, 104},  // medium purple
    {255, 84, 48, 108},
}};

// Tertiary — blue/green/yellow (WLED: Tertiary_01_gp, palette #34/21)
// Source: 5 stops resampled to 16
const WLEDPalette16 PAL_DATA_TERTIARY = {{
    {0, 0, 25, 255},  // deep blue
    {21, 13, 55, 216},
    {42, 26, 90, 176},
    {63, 38, 140, 117},  // teal-green
    {84, 62, 197, 58},
    {106, 86, 255, 0},  // bright green
    {127, 86, 255, 0},
    {149, 126, 197, 9},
    {163, 167, 140, 19},  // olive
    {170, 167, 140, 19},
    {191, 211, 90, 30},
    {204, 255, 65, 37},
    {212, 255, 45, 39},
    {220, 255, 25, 41},  // red-orange
    {238, 255, 25, 41},
    {255, 255, 25, 41},
}};

// Fire — lava flow red/orange/yellow (WLED: lava_gp, palette #35/22)
// Source: 13 stops resampled to 16
const WLEDPalette16 PAL_DATA_FIRE = {{
    {0, 0, 0, 0},  // black
    {23, 38, 0, 0},
    {46, 77, 0, 0},  // dark red
    {72, 136, 19, 4},
    {96, 177, 0, 0},  // red
    {108, 196, 38, 9},
    {119, 215, 76, 19},  // orange
    {133, 225, 95, 24},
    {146, 235, 115, 29},
    {160, 245, 133, 33},
    {174, 255, 153, 41},  // yellow-orange
    {188, 255, 178, 41},
    {202, 255, 204, 41},  // yellow
    {218, 255, 230, 41},
    {234, 255, 255, 41},  // bright yellow
    {255, 255, 255, 255},  // white
}};

// Icefire — black to blue to white (WLED: fierce_ice_gp, palette #36/23)
// Source: 7 stops resampled to 16
const WLEDPalette16 PAL_DATA_ICEFIRE = {{
    {0, 0, 0, 0},  // black
    {20, 0, 17, 39},
    {40, 0, 34, 78},
    {59, 0, 51, 117},  // dark blue
    {79, 0, 76, 186},
    {99, 0, 102, 255},  // medium blue
    {119, 0, 102, 255},
    {134, 19, 127, 255},
    {149, 38, 153, 255},  // bright blue
    {165, 62, 178, 255},
    {180, 86, 204, 255},  // light blue
    {199, 126, 217, 255},
    {217, 167, 230, 255},  // pale blue
    {231, 211, 242, 255},
    {244, 255, 255, 255},
    {255, 255, 255, 255},  // white
}};

// Cyane — colourful green/yellow/blue (WLED: Colorfull_gp, palette #37/24)
// Source: 11 stops resampled to 16
const WLEDPalette16 PAL_DATA_CYANE = {{
    {0, 61, 155, 44},  // sea green
    {12, 73, 163, 57},
    {25, 95, 174, 77},  // medium green
    {42, 113, 183, 95},
    {60, 132, 193, 113},
    {77, 143, 179, 119},
    {93, 154, 166, 125},  // grey-green
    {107, 165, 152, 131},
    {120, 175, 138, 136},  // grey
    {124, 255, 255, 192},  // bright white-yellow
    {146, 211, 236, 197},
    {168, 167, 218, 203},  // periwinkle
    {195, 132, 207, 208},
    {220, 110, 196, 211},
    {240, 95, 189, 213},
    {255, 84, 182, 215},  // sky blue
}};

// Light Pink — pink-purple pastel (WLED: Pink_Purple_gp, palette #38/25)
// Source: 11 stops resampled to 16
const WLEDPalette16 PAL_DATA_LIGHT_PINK = {{
    {0, 79, 32, 109},  // deep purple
    {12, 84, 36, 113},
    {25, 90, 40, 117},  // medium purple
    {38, 96, 44, 120},
    {51, 102, 48, 124},  // purple
    {64, 121, 91, 154},
    {76, 141, 135, 185},  // periwinkle
    {89, 160, 178, 216},
    {102, 180, 222, 248},  // pale blue
    {106, 194, 229, 250},
    {110, 208, 236, 252},  // ice blue
    {112, 222, 243, 253},
    {120, 237, 250, 255},  // near-white blue
    {136, 221, 225, 247},
    {160, 205, 200, 239},  // silver
    {255, 198, 111, 184},  // soft pink
}};

// Aurora 2 — greenish/turquoise/pink/purple (WLED: Aurora2_gp, palette #55/42)
// Source: 5 stops resampled to 16
const WLEDPalette16 PAL_DATA_AURORA2 = {{
    {0, 17, 177, 13},  // green
    {16, 42, 196, 7},
    {32, 68, 215, 4},
    {48, 95, 232, 4},
    {64, 121, 242, 5},  // bright green
    {80, 89, 222, 33},
    {96, 57, 202, 62},
    {112, 41, 188, 91},
    {128, 25, 173, 121},  // turquoise
    {144, 88, 149, 123},
    {160, 151, 125, 124},
    {176, 200, 101, 125},
    {192, 250, 77, 127},  // pink
    {208, 232, 85, 158},
    {224, 214, 92, 189},
    {255, 171, 101, 221},  // purple
}};

// Retro Clown — yellow-orange to pink to purple (WLED: retro_clown_gp / bhw1_01_gp, palette #56/43)
// Source: 3 stops expanded to 16
const WLEDPalette16 PAL_DATA_RETRO_CLOWN = {{
    {0, 242, 168, 38},  // golden orange
    {17, 241, 159, 44},
    {34, 239, 149, 52},
    {51, 237, 139, 60},
    {68, 236, 130, 68},
    {85, 234, 120, 76},
    {102, 232, 110, 84},
    {117, 226, 78, 80},  // warm pink
    {132, 218, 73, 100},
    {149, 211, 68, 120},
    {166, 203, 63, 141},
    {183, 196, 58, 162},
    {200, 189, 56, 183},
    {217, 181, 55, 203},
    {238, 172, 54, 217},
    {255, 161, 54, 225},  // purple
}};

// Autumn — warm brown/gold/green (WLED: es_autumn_19_gp, palette #39/26)
// Source: 13 stops; 16 selected to cover the full range
const WLEDPalette16 PAL_DATA_AUTUMN = {{
    {0, 90, 14, 5},  // dark brown
    {26, 117, 28, 9},
    {51, 139, 41, 13},  // warm brown
    {68, 162, 57, 15},
    {84, 180, 70, 17},  // olive
    {104, 192, 202, 125},  // sage green
    {112, 177, 137, 3},  // golden green
    {122, 190, 200, 131},
    {135, 177, 137, 3},  // back to golden
    {142, 194, 203, 118},
    {155, 185, 136, 68},
    {163, 177, 68, 17},  // orange-brown
    {184, 153, 52, 15},
    {204, 128, 35, 12},  // dark amber
    {249, 74, 5, 2},  // deep red-brown
    {255, 74, 5, 2},
}};

// Magenta — black to blue to magenta to white (WLED: BlacK_Blue_Magenta_White_gp, palette #40/27)
// Source: 7 stops expanded to 16
const WLEDPalette16 PAL_DATA_MAGENTA = {{
    {0, 0, 0, 0},  // black
    {14, 0, 0, 39},
    {28, 0, 0, 78},
    {42, 0, 0, 117},  // dark blue
    {56, 0, 0, 157},
    {70, 0, 0, 196},
    {84, 0, 0, 255},  // blue
    {99, 38, 0, 255},
    {113, 75, 0, 255},
    {127, 113, 0, 255},  // blue-magenta
    {141, 152, 0, 255},
    {155, 191, 0, 255},
    {170, 255, 0, 255},  // magenta
    {184, 255, 43, 255},
    {198, 255, 85, 255},
    {255, 255, 255, 255},  // white
}};

// Magred — black to magenta to red (WLED: BlacK_Magenta_Red_gp, palette #41/28)
// Source: 5 stops expanded to 16
const WLEDPalette16 PAL_DATA_MAGRED = {{
    {0, 0, 0, 0},  // black
    {16, 28, 0, 29},
    {32, 57, 0, 58},
    {47, 85, 0, 88},
    {63, 113, 0, 117},  // dark magenta
    {79, 170, 0, 174},
    {95, 213, 0, 213},
    {111, 241, 0, 241},
    {127, 255, 0, 255},  // magenta
    {143, 255, 0, 220},
    {159, 255, 0, 186},
    {175, 255, 0, 152},
    {191, 255, 0, 117},  // mid magenta-red
    {207, 255, 0, 78},
    {229, 255, 0, 39},
    {255, 255, 0, 0},  // red
}};

// Yelmag — black to red to magenta to yellow (WLED: BlacK_Red_Magenta_Yellow_gp, palette #42/29)
// Source: 7 stops expanded to 16
const WLEDPalette16 PAL_DATA_YELMAG = {{
    {0, 0, 0, 0},  // black
    {14, 37, 0, 0},
    {28, 75, 0, 0},
    {42, 113, 0, 0},  // dark red
    {56, 155, 0, 0},
    {70, 196, 0, 0},
    {84, 255, 0, 0},  // red
    {99, 255, 0, 39},
    {113, 255, 0, 78},
    {127, 255, 0, 117},  // red-magenta
    {141, 255, 0, 157},
    {155, 255, 0, 196},
    {170, 255, 0, 255},  // magenta
    {184, 255, 43, 213},
    {198, 255, 85, 157},
    {255, 255, 255, 0},  // yellow
}};

// Yelblu — blue to cyan to yellow (WLED: Blue_Cyan_Yellow_gp, palette #43/30)
// Source: 5 stops expanded to 16
const WLEDPalette16 PAL_DATA_YELBLU = {{
    {0, 0, 0, 255},  // blue
    {16, 0, 32, 255},
    {32, 0, 64, 255},
    {47, 0, 96, 255},
    {63, 0, 128, 255},  // medium blue
    {79, 0, 160, 255},
    {95, 0, 191, 255},
    {111, 0, 223, 255},
    {127, 0, 255, 255},  // cyan
    {143, 28, 255, 226},
    {159, 57, 255, 198},
    {175, 85, 255, 170},
    {191, 113, 255, 117},  // cyan-yellow
    {207, 142, 255, 78},
    {229, 191, 255, 39},
    {255, 255, 255, 0},  // yellow
}};

// Orange & Teal — teal to orange two-tone (WLED: Orange_Teal_gp, palette #44/31)
// Source: 4 stops expanded to 16
const WLEDPalette16 PAL_DATA_ORANGE_TEAL = {{
    {0, 0, 150, 92},  // teal
    {18, 0, 150, 92},
    {36, 0, 150, 92},
    {55, 0, 150, 92},  // teal (hard edge follows)
    {56, 17, 147, 79},
    {80, 51, 138, 52},
    {104, 85, 128, 27},
    {128, 119, 119, 8},
    {152, 153, 109, 2},
    {168, 187, 100, 1},
    {180, 204, 90, 0},
    {192, 221, 82, 0},
    {200, 255, 72, 0},  // orange
    {215, 255, 72, 0},
    {235, 255, 72, 0},
    {255, 255, 72, 0},
}};

// Aqua Flash — black/aqua/yellow/white strobe (WLED: aqua_flash_gp / bhw2_23_gp, palette #63/50)
// Source: 7 stops expanded to 16
const WLEDPalette16 PAL_DATA_AQUA_FLASH = {{
    {0, 0, 0, 0},  // black
    {22, 43, 80, 81},
    {44, 87, 161, 163},
    {66, 130, 242, 245},  // aqua
    {80, 174, 249, 120},
    {96, 255, 255, 53},  // yellow
    {110, 255, 255, 154},
    {124, 255, 255, 255},  // white
    {138, 255, 255, 154},
    {153, 255, 255, 53},  // yellow again
    {168, 218, 250, 148},
    {180, 182, 247, 198},
    {188, 130, 242, 245},  // aqua again
    {214, 87, 161, 163},
    {235, 43, 80, 81},
    {255, 0, 0, 0},  // black
}};

// Lite Light — dark dim purple (WLED: lite_light_gp / bhw2_45_gp, palette #65/52)
// Source: 6 stops expanded to 16
const WLEDPalette16 PAL_DATA_LITE_LIGHT = {{
    {0, 0, 0, 0},  // black
    {5, 10, 10, 11},
    {9, 20, 21, 22},  // very dim warm white
    {18, 30, 29, 32},
    {28, 38, 35, 40},
    {40, 46, 43, 49},  // dim purple-grey
    {50, 46, 43, 49},
    {57, 46, 43, 49},
    {66, 46, 43, 49},  // plateau
    {77, 55, 36, 58},
    {88, 60, 28, 62},
    {101, 61, 16, 65},  // dim purple
    {128, 46, 12, 49},
    {160, 31, 8, 33},
    {200, 15, 4, 16},
    {255, 0, 0, 0},  // black
}};

// Red Flash — black/red/white strobe (WLED: red_flash_gp / bhw2_22_gp, palette #66/53)
// Source: 5 stops expanded to 16
const WLEDPalette16 PAL_DATA_RED_FLASH = {{
    {0, 0, 0, 0},  // black
    {25, 60, 3, 2},
    {50, 121, 6, 4},
    {74, 181, 9, 6},
    {99, 242, 12, 8},  // bright red
    {107, 246, 57, 44},
    {116, 249, 114, 90},
    {122, 251, 163, 127},
    {130, 253, 228, 163},  // near-white warm
    {138, 251, 163, 127},
    {144, 249, 114, 90},
    {150, 246, 57, 44},
    {155, 242, 12, 8},  // back to red
    {185, 161, 8, 5},
    {220, 80, 4, 2},
    {255, 0, 0, 0},  // black
}};

// Blink Red — dark purple/pink/violet (WLED: blink_red_gp / bhw3_40_gp, palette #67/54)
// Source: 8 stops expanded to 16
const WLEDPalette16 PAL_DATA_BLINK_RED = {{
    {0, 4, 7, 4},  // near-black green tint
    {22, 22, 16, 33},
    {43, 40, 25, 62},  // dark purple
    {58, 50, 20, 49},
    {76, 61, 15, 36},  // very dark
    {93, 134, 27, 66},
    {109, 207, 39, 96},  // vivid red-purple
    {118, 231, 97, 140},
    {127, 255, 156, 184},  // bright pink
    {143, 228, 120, 197},
    {156, 212, 96, 202},
    {165, 185, 73, 207},  // medium purple
    {182, 155, 69, 220},
    {204, 105, 66, 240},  // blue-purple
    {228, 91, 47, 159},
    {255, 77, 29, 78},  // dark red-violet
}};

// Red Shift — purple/red/orange/magenta (WLED: red_shift_gp / bhw3_52_gp, palette #68/55)
// Source: 7 stops expanded to 16
const WLEDPalette16 PAL_DATA_RED_SHIFT = {{
    {0, 98, 22, 93},  // mid purple
    {15, 99, 22, 86},
    {30, 101, 22, 79},
    {45, 103, 22, 73},  // darker purple
    {62, 130, 30, 67},
    {78, 158, 36, 61},
    {99, 192, 45, 56},  // red-orange
    {113, 210, 99, 57},
    {132, 235, 187, 59},  // golden
    {150, 232, 143, 43},
    {163, 230, 113, 34},
    {175, 228, 85, 26},  // orange
    {185, 228, 75, 34},
    {197, 228, 65, 42},
    {201, 228, 56, 48},  // orange-red
    {255, 2, 0, 2},  // near-black
}};

// Red Tide — warm fire/red/orange waves (WLED: red_tide_gp / bhw4_097_gp, palette #69/56)
// Source: 11 stops; 16 selected covering full range
const WLEDPalette16 PAL_DATA_RED_TIDE = {{
    {0, 251, 46, 0},  // orange-red
    {14, 253, 92, 12},
    {28, 255, 139, 25},  // warm orange
    {36, 250, 148, 44},
    {43, 246, 158, 63},  // peach
    {51, 246, 187, 93},
    {58, 246, 216, 123},  // pale gold
    {71, 244, 155, 67},
    {84, 243, 94, 10},  // back to orange
    {99, 210, 79, 11},
    {114, 177, 65, 11},  // dark red-orange
    {127, 216, 153, 63},
    {140, 255, 241, 115},  // bright yellow
    {168, 177, 65, 11},  // dark orange again
    {196, 250, 233, 158},  // pale yellow
    {255, 126, 8, 4},  // deep red
}};

// Candy2 — grey/purple/gold/teal muted tones (WLED: candy2_gp / bhw4_017_gp, palette #70/57)
// Source: 10 stops expanded to 16
const WLEDPalette16 PAL_DATA_CANDY2 = {{
    {0, 109, 102, 102},  // warm grey
    {13, 76, 76, 87},
    {25, 42, 49, 71},  // blue-grey
    {37, 81, 72, 78},
    {48, 121, 96, 84},  // warm tan
    {61, 181, 155, 55},
    {73, 241, 214, 26},  // golden yellow
    {81, 228, 159, 35},
    {89, 216, 104, 44},  // amber
    {110, 129, 77, 58},
    {130, 42, 49, 71},  // blue-grey
    {147, 149, 113, 59},
    {163, 255, 177, 47},  // golden
    {175, 248, 196, 37},
    {186, 241, 214, 26},  // bright gold
    {255, 20, 19, 13},  // near-black brown
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
    {/* 19 */ "C9", &PAL_DATA_C9},
    {/* 20 */ "Atlantica", &PAL_DATA_ATLANTICA},
    {/* 21 */ "C9 2", &PAL_DATA_C9_2},
    {/* 22 */ "C9 New", &PAL_DATA_C9_NEW},
    {/* 23 */ "Temperature", &PAL_DATA_TEMPERATURE},
    {/* 24 */ "Candy", &PAL_DATA_CANDY},
    {/* 25 */ "Toxy Reaf", &PAL_DATA_TOXY_REAF},
    {/* 26 */ "Fairy Reaf", &PAL_DATA_FAIRY_REAF},
    {/* 27 */ "Semi Blue", &PAL_DATA_SEMI_BLUE},
    {/* 28 */ "Pink Candy", &PAL_DATA_PINK_CANDY},
    {/* 29 */ "Red Reaf", &PAL_DATA_RED_REAF},
    {/* 30 */ "Yelblu Hot", &PAL_DATA_YELBLU_HOT},
    {/* 31 */ "Sunset Real", &PAL_DATA_SUNSET_REAL},
    {/* 32 */ "Rivendell", &PAL_DATA_RIVENDELL},
    {/* 33 */ "Red & Blue", &PAL_DATA_RED_BLUE},
    {/* 34 */ "Tertiary", &PAL_DATA_TERTIARY},
    {/* 35 */ "Fire", &PAL_DATA_FIRE},
    {/* 36 */ "Icefire", &PAL_DATA_ICEFIRE},
    {/* 37 */ "Cyane", &PAL_DATA_CYANE},
    {/* 38 */ "Light Pink", &PAL_DATA_LIGHT_PINK},
    {/* 39 */ "Aurora 2", &PAL_DATA_AURORA2},
    {/* 40 */ "Retro Clown", &PAL_DATA_RETRO_CLOWN},
    {/* 41 */ "Autumn", &PAL_DATA_AUTUMN},
    {/* 42 */ "Magenta", &PAL_DATA_MAGENTA},
    {/* 43 */ "Magred", &PAL_DATA_MAGRED},
    {/* 44 */ "Yelmag", &PAL_DATA_YELMAG},
    {/* 45 */ "Yelblu", &PAL_DATA_YELBLU},
    {/* 46 */ "Orange & Teal", &PAL_DATA_ORANGE_TEAL},
    {/* 47 */ "Aqua Flash", &PAL_DATA_AQUA_FLASH},
    {/* 48 */ "Lite Light", &PAL_DATA_LITE_LIGHT},
    {/* 49 */ "Red Flash", &PAL_DATA_RED_FLASH},
    {/* 50 */ "Blink Red", &PAL_DATA_BLINK_RED},
    {/* 51 */ "Red Shift", &PAL_DATA_RED_SHIFT},
    {/* 52 */ "Red Tide", &PAL_DATA_RED_TIDE},
    {/* 53 */ "Candy2", &PAL_DATA_CANDY2},
};

}  // namespace wled_bridge
}  // namespace esphome
