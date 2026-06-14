#!/usr/bin/env python3
"""Regenerate the gradient-palette tables in ``wled_palette.cpp`` from WLED source.

WLED stores its cpt-city gradient palettes as flat ``const byte NAME_gp[] = {pos,r,g,b, ...}``
arrays in ``wled00/palettes.cpp`` (palette IDs 13..71, via ``gGradientPalettes[]``).
The component's ``PaletteEntry`` is exactly ``{pos, r, g, b}``, so each WLED gradient
stop maps 1:1 — this transcribes them faithfully instead of the previous hand-tuned
tables that had drifted from upstream.

Palette IDs 0..5 are dynamic (no data) and 6..12 are FastLED built-ins
(``PartyColors``/``CloudColors``/… , some gamma-corrected); those are NOT regenerated
here — only the literal ``*_gp`` gradient palettes (IDs 13..71).

Usage:
    python gen_palettes.py /path/to/WLED/wled00/palettes.cpp > generated_block.txt

Then paste the emitted block into ``wled_palette.cpp`` (between the BEGIN/END markers).
"""
import re
import sys

# WLED gradient-palette name (from gGradientPalettes[]) -> component PAL_DATA_* symbol.
# Order/IDs taken from WLED palettes.cpp gGradientPalettes[] and the component's
# WLED_PALETTES[] table. Index in this list + 13 == WLED palette ID.
GP_TO_SYMBOL = [
    ("Sunset_Real_gp", "PAL_DATA_SUNSET_REAL"),          # 13 Sunset
    ("es_rivendell_15_gp", "PAL_DATA_RIVENDELL"),        # 14 Rivendell
    ("es_ocean_breeze_036_gp", "PAL_DATA_BREEZE"),       # 15 Breeze
    ("rgi_15_gp", "PAL_DATA_RED_BLUE"),                  # 16 Red & Blue
    ("retro2_16_gp", "PAL_DATA_YELLOWOUT"),              # 17 Yellowout
    ("Analogous_1_gp", "PAL_DATA_ANALOGOUS"),            # 18 Analogous
    ("es_pinksplash_08_gp", "PAL_DATA_SPLASH"),          # 19 Splash
    ("Sunset_Yellow_gp", "PAL_DATA_PASTEL"),             # 20 Pastel
    ("Another_Sunset_gp", "PAL_DATA_SUNSET2"),           # 21 Sunset2
    ("Beech_gp", "PAL_DATA_BEECH"),                      # 22 Beech
    ("es_vintage_01_gp", "PAL_DATA_VINTAGE"),            # 23 Vintage
    ("departure_gp", "PAL_DATA_DEPARTURE"),              # 24 Departure
    ("es_landscape_64_gp", "PAL_DATA_LANDSCAPE"),        # 25 Landscape
    ("es_landscape_33_gp", "PAL_DATA_BEACH"),            # 26 Beach
    ("rainbowsherbet_gp", "PAL_DATA_SHERBET"),           # 27 Sherbet
    ("gr65_hult_gp", "PAL_DATA_HULT"),                   # 28 Hult
    ("gr64_hult_gp", "PAL_DATA_HULT64"),                 # 29 Hult64
    ("GMT_drywet_gp", "PAL_DATA_DRYWET"),                # 30 Drywet
    ("ib_jul01_gp", "PAL_DATA_JUL"),                     # 31 Jul
    ("es_vintage_57_gp", "PAL_DATA_GRINTAGE"),           # 32 Grintage
    ("ib15_gp", "PAL_DATA_REWHI"),                       # 33 Rewhi
    ("Tertiary_01_gp", "PAL_DATA_TERTIARY"),             # 34 Tertiary
    ("lava_gp", "PAL_DATA_FIRE"),                        # 35 Fire (WLED maps to lava_gp)
    ("fierce_ice_gp", "PAL_DATA_ICEFIRE"),               # 36 Icefire
    ("Colorfull_gp", "PAL_DATA_CYANE"),                  # 37 Cyane
    ("Pink_Purple_gp", "PAL_DATA_LIGHT_PINK"),           # 38 Light Pink
    ("es_autumn_19_gp", "PAL_DATA_AUTUMN"),              # 39 Autumn
    ("BlacK_Blue_Magenta_White_gp", "PAL_DATA_MAGENTA"),  # 40 Magenta
    ("BlacK_Magenta_Red_gp", "PAL_DATA_MAGRED"),         # 41 Magred
    ("BlacK_Red_Magenta_Yellow_gp", "PAL_DATA_YELMAG"),  # 42 Yelmag
    ("Blue_Cyan_Yellow_gp", "PAL_DATA_YELBLU"),          # 43 Yelblu
    ("Orange_Teal_gp", "PAL_DATA_ORANGE_TEAL"),          # 44 Orange & Teal
    ("Tiamat_gp", "PAL_DATA_TIAMAT"),                    # 45 Tiamat
    ("April_Night_gp", "PAL_DATA_APRIL_NIGHT"),          # 46 April Night
    ("Orangery_gp", "PAL_DATA_ORANGERY"),                # 47 Orangery
    ("C9_gp", "PAL_DATA_C9"),                            # 48 C9
    ("Sakura_gp", "PAL_DATA_SAKURA"),                    # 49 Sakura
    ("Aurora_gp", "PAL_DATA_AURORA"),                    # 50 Aurora
    ("Atlantica_gp", "PAL_DATA_ATLANTICA"),              # 51 Atlantica
    ("C9_2_gp", "PAL_DATA_C9_2"),                        # 52 C9 2
    ("C9_new_gp", "PAL_DATA_C9_NEW"),                    # 53 C9 New
    ("temperature_gp", "PAL_DATA_TEMPERATURE"),          # 54 Temperature
    ("Aurora2_gp", "PAL_DATA_AURORA2"),                  # 55 Aurora 2
    ("retro_clown_gp", "PAL_DATA_RETRO_CLOWN"),          # 56 Retro Clown
    ("candy_gp", "PAL_DATA_CANDY"),                      # 57 Candy
    ("toxy_reaf_gp", "PAL_DATA_TOXY_REAF"),              # 58 Toxy Reaf
    ("fairy_reaf_gp", "PAL_DATA_FAIRY_REAF"),            # 59 Fairy Reaf
    ("semi_blue_gp", "PAL_DATA_SEMI_BLUE"),              # 60 Semi Blue
    ("pink_candy_gp", "PAL_DATA_PINK_CANDY"),            # 61 Pink Candy
    ("red_reaf_gp", "PAL_DATA_RED_REAF"),                # 62 Red Reaf
    ("aqua_flash_gp", "PAL_DATA_AQUA_FLASH"),            # 63 Aqua Flash
    ("yelblu_hot_gp", "PAL_DATA_YELBLU_HOT"),            # 64 Yelblu Hot
    ("lite_light_gp", "PAL_DATA_LITE_LIGHT"),            # 65 Lite Light
    ("red_flash_gp", "PAL_DATA_RED_FLASH"),              # 66 Red Flash
    ("blink_red_gp", "PAL_DATA_BLINK_RED"),              # 67 Blink Red
    ("red_shift_gp", "PAL_DATA_RED_SHIFT"),              # 68 Red Shift
    ("red_tide_gp", "PAL_DATA_RED_TIDE"),                # 69 Red Tide
    ("candy2_gp", "PAL_DATA_CANDY2"),                    # 70 Candy2
    ("trafficlight_gp", "PAL_DATA_TRAFFIC_LIGHT"),       # 71 Traffic Light
]

MAX_STOPS = 16


def parse_gp_arrays(src: str) -> dict:
    """Extract every ``const byte NAME_gp[] PROGMEM = { ... };`` as a list of (pos,r,g,b)."""
    out = {}
    pat = re.compile(
        r"const\s+(?:byte|uint8_t)\s+(\w+_gp)\s*\[\]\s*PROGMEM\s*=\s*\{(.*?)\}\s*;",
        re.DOTALL,
    )
    for m in pat.finditer(src):
        name = m.group(1)
        body = re.sub(r"//.*", "", m.group(2))  # strip line comments
        nums = [int(n) for n in re.findall(r"\d+", body)]
        if len(nums) % 4 != 0:
            raise SystemExit(f"{name}: byte count {len(nums)} not a multiple of 4")
        quads = [tuple(nums[i:i + 4]) for i in range(0, len(nums), 4)]
        out[name] = quads
    return out


def lerp(a: int, b: int, num: int, den: int) -> int:
    return a + (b - a) * num // den


def sample(stops, pos: int):
    """Linear-interpolate the gradient (list of (pos,r,g,b)) at position 0..255."""
    if pos <= stops[0][0]:
        return stops[0][1:]
    if pos >= stops[-1][0]:
        return stops[-1][1:]
    for i in range(len(stops) - 1):
        p0, r0, g0, b0 = stops[i]
        p1, r1, g1, b1 = stops[i + 1]
        if p0 <= pos <= p1:
            if p1 == p0:
                return (r0, g0, b0)
            d = p1 - p0
            n = pos - p0
            return (lerp(r0, r1, n, d), lerp(g0, g1, n, d), lerp(b0, b1, n, d))
    return stops[-1][1:]


def to_16_stops(stops):
    """Fit a WLED gradient into exactly 16 PaletteEntry stops.

    <=16 stops: copy verbatim, pad the tail with the final stop (pos=255).
    >16 stops:  resample at 16 evenly spaced positions (0,17,34,...,255).
    """
    if len(stops) <= MAX_STOPS:
        out = list(stops)
        while len(out) < MAX_STOPS:
            last = out[-1]
            out.append((255, last[1], last[2], last[3]))
        return out
    out = []
    for i in range(MAX_STOPS):
        pos = i * 255 // (MAX_STOPS - 1)
        r, g, b = sample(stops, pos)
        out.append((pos, r, g, b))
    return out


def main():
    if len(sys.argv) not in (2, 3):
        raise SystemExit(__doc__)
    src = open(sys.argv[1], encoding="utf-8").read()
    arrays = parse_gp_arrays(src)
    if len(sys.argv) == 3:
        # Write to a file (avoids shell redirection so reference-dir hooks don't trip).
        sys.stdout = open(sys.argv[2], "w", encoding="utf-8")

    print("// === BEGIN generated gradient palettes (tools/gen_palettes.py) ===")
    print("// Source: WLED wled00/palettes.cpp gGradientPalettes[] (IDs 13..71).")
    print("// Do not edit by hand — rerun gen_palettes.py against WLED source.\n")
    over = []
    for gp_name, symbol in GP_TO_SYMBOL:
        if gp_name not in arrays:
            raise SystemExit(f"missing gradient array in WLED source: {gp_name}")
        raw = arrays[gp_name]
        if len(raw) > MAX_STOPS:
            over.append((gp_name, len(raw)))
        stops = to_16_stops(raw)
        print(f"// {symbol}  <-  WLED {gp_name} ({len(raw)} stops)")
        print(f"const WLEDPalette16 {symbol} = {{{{")
        for (p, r, g, b) in stops:
            print(f"    {{{p}, {r}, {g}, {b}}},")
        print("}};\n")
    print("// === END generated gradient palettes ===")
    if over:
        sys.stderr.write("resampled (>16 stops): " + ", ".join(f"{n}={c}" for n, c in over) + "\n")


if __name__ == "__main__":
    main()
