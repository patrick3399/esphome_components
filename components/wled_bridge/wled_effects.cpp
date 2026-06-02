// WLED Effect Engine — 30 1D effects ported from WLED v16.0.0
// Algorithms are derived from WLED's FX.cpp (EUPL-1.2 original).
// This implementation is an independent rewrite under ESPHome's MIT license.
// Reference: WLED/wled00/FX.cpp, FX_fcn.cpp (tag v16.0.0)
#include "wled_effects.h"
#include "wled_fx_math.h"
#include <algorithm>
#include <cmath>
#include <string.h>

namespace esphome {
namespace wled_bridge {

// ---------- global PRNG (seeded once at startup) ----------
PRNG g_prng(static_cast<uint16_t>(esp_random()));

// ============================================================
// Helper macros — keep effects readable while staying explicit
// ============================================================
#define SEGLEN ctx.len
#define SPEED ctx.params->speed
#define INTENSITY ctx.params->intensity
#define C1 ctx.params->custom1
#define C2 ctx.params->custom2
#define C3 ctx.params->custom3
#define COLOR0 ctx.params->colors[0]
#define COLOR1 ctx.params->colors[1]
#define COLOR2 ctx.params->colors[2]
#define NOW ctx.now
#define STEP ctx.env->step
#define CALL ctx.env->call
#define AUX0 ctx.env->aux0
#define AUX1 ctx.env->aux1

// ============================================================
// 0 — Solid
// ============================================================
void fx_solid(EffectContext &ctx) {
  uint32_t col = (ctx.params->palette_id == 0) ? COLOR0 : ctx.pal_color(0);
  ctx.fill(col);
}

// ============================================================
// 1 — Blink
// ============================================================
void fx_blink(EffectContext &ctx) {
  uint32_t cycleTime = (1000u * (uint32_t(9) - (SPEED >> 5))) / 10u + 1u;
  uint32_t onTime = (cycleTime * INTENSITY) >> 8;
  bool on = (NOW % cycleTime) < onTime;
  ctx.fill(on ? COLOR0 : COLOR1);
}

// ============================================================
// 2 — Breathe
// ============================================================
void fx_breathe(EffectContext &ctx) {
  // sinusoidal brightness pulse
  uint16_t bpm = 12 + (SPEED >> 4);
  uint8_t s = beatsin8(static_cast<uint8_t>(bpm), 0, 255, NOW);
  uint32_t col = color_fade(COLOR0, s);
  ctx.fill(col);
}

// ============================================================
// 3 — Color Wipe (forward)
// ============================================================
void fx_color_wipe(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  uint32_t cycleTime = 750u + (255u - SPEED) * 150u;
  uint32_t perc = NOW % cycleTime;
  int32_t prog = static_cast<int32_t>((perc * SEGLEN * 2) / cycleTime);
  bool forward = prog < SEGLEN;
  int32_t led = forward ? prog : (SEGLEN * 2 - prog - 1);

  for (int32_t i = 0; i < SEGLEN; i++) {
    bool colored = forward ? (i <= led) : (i > led);
    uint32_t col = colored ? ctx.pal_color_at(i) : COLOR1;
    ctx.set_pixel(i, col);
  }
}

// ============================================================
// 4 — Color Wipe Random
// ============================================================
void fx_color_wipe_random(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  uint32_t cycleTime = 750u + (255u - SPEED) * 150u;
  uint32_t perc = NOW % cycleTime;
  uint32_t prog = (perc * SEGLEN * 2) / cycleTime;
  bool forward = prog < static_cast<uint32_t>(SEGLEN);
  int32_t led = forward ? static_cast<int32_t>(prog) : static_cast<int32_t>(SEGLEN * 2 - prog - 1);

  // Re-seed aux on each new wipe cycle
  uint32_t cycle = NOW / cycleTime;
  if (cycle != STEP) {
    AUX0 = AUX1;
    AUX1 = hw_random8();
    STEP = cycle;
  }

  uint32_t col1 = ctx.wheel(AUX0);
  uint32_t col2 = ctx.wheel(AUX1);

  for (int32_t i = 0; i < SEGLEN; i++) {
    bool colored = forward ? (i <= led) : (i > led);
    ctx.set_pixel(i, colored ? col2 : col1);
  }
}

// ============================================================
// 5 — Random Color (solid, changes periodically)
// ============================================================
void fx_random_color(EffectContext &ctx) {
  uint32_t interval = 500u + (255u - SPEED) * 20u;
  if (STEP == 0 || NOW - STEP > interval) {
    AUX0 = hw_random8();
    STEP = NOW;
  }
  ctx.fill(ctx.wheel(AUX0));
}

// ============================================================
// 6 — Color Sweep (fills backward like a windshield wiper)
// ============================================================
void fx_color_sweep(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  uint32_t cycleTime = 750u + (255u - SPEED) * 150u;
  uint32_t perc = NOW % cycleTime;
  int32_t prog = static_cast<int32_t>((perc * SEGLEN * 2) / cycleTime);
  bool forward = prog < SEGLEN;
  int32_t led = forward ? prog : (SEGLEN * 2 - prog - 1);

  for (int32_t i = 0; i < SEGLEN; i++) {
    // Reversed compared to color_wipe
    bool colored = forward ? (i >= SEGLEN - 1 - led) : (i < SEGLEN - led);
    ctx.set_pixel(i, colored ? ctx.pal_color_at(i) : COLOR1);
  }
}

// ============================================================
// 7 — Dynamic (random pixels)
// ============================================================
void fx_dynamic(EffectContext &ctx) {
  uint32_t interval = 50u + (255u - SPEED) * 4u;
  if (STEP == 0 || NOW - STEP > interval) {
    if (INTENSITY > 128) {
      ctx.fill_black();
    }
    int32_t n = 1 + (INTENSITY >> 4);
    for (int32_t i = 0; i < n; i++) {
      int32_t pos = hw_random16(SEGLEN);
      ctx.set_pixel(pos, ctx.wheel(hw_random8()));
    }
    STEP = NOW;
  }
}

// ============================================================
// 8 — Colorloop (cycling solid hue)
// ============================================================
void fx_colorloop(EffectContext &ctx) {
  uint8_t counter = static_cast<uint8_t>((NOW * ((SPEED >> 2) + 2)) >> 8);
  uint32_t col;
  if (INTENSITY < 128) {
    col = color_blend(ctx.wheel(counter), 0xFFFFFF, static_cast<uint8_t>(128 - INTENSITY));
  } else {
    col = ctx.wheel(counter);
  }
  ctx.fill(col);
}

// ============================================================
// 9 — Rainbow Cycle
// ============================================================
void fx_rainbow_cycle(EffectContext &ctx) {
  if (SEGLEN <= 0)
    return;
  uint8_t counter = static_cast<uint8_t>((NOW * ((SPEED >> 2) + 2)) >> 8);
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t idx = static_cast<uint8_t>(i * (16 << (INTENSITY / 29)) / SEGLEN + counter);
    ctx.set_pixel(i, ctx.wheel(idx));
  }
}

// ============================================================
// 10 — Scan
// ============================================================
static void do_scan(EffectContext &ctx, bool dual) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  int32_t width = 1 + (INTENSITY >> 4);
  uint32_t interval = 50u + (255u - SPEED) * 2u;
  if (STEP == 0 || NOW - STEP > interval) {
    if (AUX0 == 1) {
      if (AUX1 + width >= static_cast<uint16_t>(SEGLEN))
        AUX0 = 0;
      else
        AUX1++;
    } else {
      if (AUX1 == 0)
        AUX0 = 1;
      else
        AUX1--;
    }
    STEP = NOW;
  }
  ctx.fill(COLOR1);
  for (int32_t i = 0; i < width; i++) {
    ctx.set_pixel(AUX1 + i, ctx.pal_color_at(AUX1 + i));
    if (dual)
      ctx.set_pixel(SEGLEN - 1 - AUX1 - i, ctx.pal_color_at(SEGLEN - 1 - AUX1 - i));
  }
}

void fx_scan(EffectContext &ctx) {
  do_scan(ctx, false);
}
void fx_dual_scan(EffectContext &ctx) {
  do_scan(ctx, true);
}

// ============================================================
// 12 — Fade (brightness fade between two colors)
// ============================================================
void fx_fade(EffectContext &ctx) {
  uint32_t cycleTime = 500u + (255u - SPEED) * 10u;
  uint32_t pos = NOW % (cycleTime * 2);
  uint8_t blend =
      static_cast<uint8_t>((pos > cycleTime) ? 255 - (pos - cycleTime) * 255 / cycleTime : pos * 255 / cycleTime);
  uint32_t col = color_blend(COLOR0, COLOR1, blend);
  ctx.fill(col);
}

// ============================================================
// 13 — Theater Chase
// ============================================================
void fx_theater_chase(EffectContext &ctx) {
  uint32_t interval = 50u + (255u - SPEED);
  if (STEP == 0 || NOW - STEP > interval) {
    AUX0 = (AUX0 + 1) % 3;
    STEP = NOW;
  }
  for (int32_t i = 0; i < SEGLEN; i++) {
    ctx.set_pixel(i, (i % 3 == AUX0) ? ctx.pal_color_at(i) : COLOR1);
  }
}

// ============================================================
// 14 — Theater Chase Rainbow
// ============================================================
void fx_theater_chase_rainbow(EffectContext &ctx) {
  uint32_t interval = 50u + (255u - SPEED);
  if (STEP == 0 || NOW - STEP > interval) {
    AUX0 = (AUX0 + 1) % 3;
    AUX1 = (AUX1 + 1) & 0xFF;
    STEP = NOW;
  }
  for (int32_t i = 0; i < SEGLEN; i++) {
    ctx.set_pixel(i, (i % 3 == AUX0) ? ctx.wheel(static_cast<uint8_t>(AUX1 + i)) : COLOR1);
  }
}

// ============================================================
// 15 — Running Lights (sine wave)
// ============================================================
void fx_running_lights(EffectContext &ctx) {
  if (SEGLEN <= 0)
    return;
  uint32_t x_scale = 256u / SEGLEN;
  uint32_t counter = (NOW * SPEED) >> 9;
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t a = static_cast<uint8_t>((i * x_scale) - counter);
    uint8_t s = sin8(a);
    ctx.set_pixel(i, color_blend(COLOR1, ctx.pal_color_at(i), s));
  }
}

// ============================================================
// 16 — Saw (sawtooth wave)
// ============================================================
void fx_saw(EffectContext &ctx) {
  if (SEGLEN <= 0)
    return;
  uint32_t x_scale = 256u / SEGLEN;
  uint32_t counter = (NOW * SPEED) >> 9;
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t a = static_cast<uint8_t>((i * x_scale) - counter);
    // sawtooth: remap 0..255 to triangle-ish
    uint8_t s;
    if (a < 16)
      s = 192 + a * 8;
    else
      s = static_cast<uint8_t>(map_range(a, 16, 255, 64, 192));
    s = 255 - s;
    ctx.set_pixel(i, color_blend(COLOR1, ctx.pal_color_at(i), s));
  }
}

// ============================================================
// 17 — Twinkle
// ============================================================
void fx_twinkle(EffectContext &ctx) {
  ctx.fade_to_black(224);
  uint32_t interval = 20u + (255u - SPEED) * 5u;
  uint32_t it = NOW / interval;
  if (it != STEP) {
    uint32_t maxOn = 1 + (static_cast<uint32_t>(SEGLEN) * INTENSITY / 255);
    if (AUX0 >= maxOn) {
      AUX0 = 0;
      AUX1 = hw_random16();  // new PRNG seed
    }
    AUX0++;
    STEP = it;
  }
  // deterministic pixel selection from seed
  uint16_t prng16 = AUX1;
  for (uint32_t i = 0; i < AUX0; i++) {
    prng16 = static_cast<uint16_t>(prng16 * 2053u + 13849u);
    uint16_t idx = prng16 % SEGLEN;
    ctx.set_pixel(idx, ctx.pal_color_at(idx));
  }
}

// ============================================================
// 18 — Twinkle Random (same but random colors)
// ============================================================
void fx_twinkle_random(EffectContext &ctx) {
  ctx.fade_to_black(224);
  uint32_t interval = 20u + (255u - SPEED) * 5u;
  uint32_t it = NOW / interval;
  if (it != STEP) {
    uint32_t maxOn = 1 + static_cast<uint32_t>(SEGLEN) * INTENSITY / 255;
    if (AUX0 >= maxOn) {
      AUX0 = 0;
      AUX1 = hw_random16();
    }
    AUX0++;
    STEP = it;
  }
  uint16_t prng16 = AUX1;
  for (uint32_t i = 0; i < AUX0; i++) {
    prng16 = static_cast<uint16_t>(prng16 * 2053u + 13849u);
    uint16_t idx = prng16 % SEGLEN;
    uint8_t hue = static_cast<uint8_t>(prng16 >> 8);
    ctx.set_pixel(idx, ctx.wheel(hue));
  }
}

// ============================================================
// 19 — Strobe
// ============================================================
void fx_strobe(EffectContext &ctx) {
  uint32_t cycleTime = 50u + (255u - SPEED) * 2u;
  bool on = (NOW % cycleTime) < (cycleTime / 10u);
  ctx.fill(on ? COLOR0 : COLOR1);
}

// ============================================================
// 20 — Strobe Rainbow
// ============================================================
void fx_strobe_rainbow(EffectContext &ctx) {
  uint32_t cycleTime = 50u + (255u - SPEED) * 2u;
  if (NOW / cycleTime != STEP) {
    AUX0 = (AUX0 + 1) & 0xFF;
    STEP = NOW / cycleTime;
  }
  bool on = (NOW % cycleTime) < (cycleTime / 10u);
  ctx.fill(on ? ctx.wheel(AUX0) : 0u);
}

// ============================================================
// 21 — BPM (palette-colored sin waves at set BPM)
// ============================================================
void fx_bpm(EffectContext &ctx) {
  uint8_t bpm = SPEED >> 1;
  if (bpm < 1)
    bpm = 1;
  uint8_t beat = beat8(bpm, NOW);
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t idx = static_cast<uint8_t>(beat - i * INTENSITY);
    ctx.set_pixel(i, ctx.pal_color(idx));
  }
}

// ============================================================
// 22 — Larson Scanner (Cylon eye)
// ============================================================
void fx_larson_scanner(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  int32_t width = 1 + (INTENSITY >> 5);

  // Fade existing trail
  ctx.fade_to_black(192);

  uint32_t interval = 10u + (255u - SPEED) * 3u;
  if (STEP == 0 || NOW - STEP > interval) {
    if (AUX0 == 0) {
      AUX1++;
      if (static_cast<int32_t>(AUX1) >= SEGLEN - width)
        AUX0 = 1;
    } else {
      if (AUX1 == 0)
        AUX0 = 0;
      else
        AUX1--;
    }
    STEP = NOW;
  }
  for (int32_t i = 0; i < width; i++) {
    ctx.set_pixel(AUX1 + i, ctx.pal_color_at(AUX1 + i));
  }
}

// ============================================================
// 23 — Comet
// ============================================================
void fx_comet(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  ctx.fade_to_black(192);
  uint32_t interval = 10u + (255u - SPEED) * 2u;
  if (STEP == 0 || NOW - STEP > interval) {
    AUX0 = (AUX0 + 1) % SEGLEN;
    STEP = NOW;
  }
  ctx.set_pixel(AUX0, ctx.pal_color_at(AUX0));
}

// ============================================================
// 24 — Fire 2012 (Mark Kriegsman's classic)
// ============================================================
void fx_fire_2012(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(0xFF4400);
    return;
  }
  if (!ctx.env->allocate_data(SEGLEN)) {
    ctx.fill(0xFF4400);
    return;
  }
  uint8_t *heat = ctx.env->data;

  uint32_t it = NOW >> 5;  // every 32ms
  if (it != STEP) {
    // Step 1: cool each cell
    for (int32_t i = 0; i < SEGLEN; i++) {
      uint8_t cool = hw_random8(0, ((20 + (SPEED / 3)) * 16 / SEGLEN) + 2);
      heat[i] = qsub8(heat[i], cool);
    }
    // Step 2: diffuse heat upward
    for (int32_t k = SEGLEN - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
    }
    // Step 3: random sparks at bottom
    if (hw_random8() < INTENSITY) {
      uint8_t y = hw_random8(3);
      heat[y] = qadd8(heat[y], hw_random8(160, 255));
    }
    STEP = it;
  }

  // Step 4: render heat to pixels using Heat palette
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint32_t col;
    if (ctx.params->palette_id == PAL_DEFAULT || ctx.params->palette_id == PAL_HEAT) {
      col = palette_color(PAL_DATA_HEAT, heat[i]);
    } else {
      col = ctx.pal_color(heat[i]);
    }
    ctx.set_pixel(i, col);
  }
}

// ============================================================
// 25 — Meteor
// ============================================================
void fx_meteor(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  int32_t trail_len = 1 + (INTENSITY >> 4);

  uint32_t interval = 10u + (255u - SPEED) * 3u;
  if (STEP == 0 || NOW - STEP > interval) {
    AUX0 = (AUX0 + 1) % (SEGLEN + trail_len);
    STEP = NOW;
  }

  ctx.fill_black();

  int32_t head = static_cast<int32_t>(AUX0) - trail_len;
  for (int32_t i = 0; i < trail_len; i++) {
    int32_t pos = head + i;
    if (pos < 0 || pos >= SEGLEN)
      continue;
    uint8_t fade = static_cast<uint8_t>(255 - (trail_len - 1 - i) * 255 / trail_len);
    ctx.set_pixel(pos, color_fade(ctx.pal_color_at(pos), fade));
  }
  // bright head
  if (static_cast<int32_t>(AUX0) < SEGLEN) {
    ctx.set_pixel(AUX0, ctx.pal_color_at(AUX0));
  }
}

// ============================================================
// 26 — Meteor Smooth (same with smooth decay)
// ============================================================
void fx_meteor_smooth(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  int32_t trail_len = 1 + (INTENSITY >> 4);

  // Smooth decay: fade each pixel a bit every frame
  ctx.fade_to_black(240);

  uint32_t interval = 10u + (255u - SPEED) * 3u;
  if (STEP == 0 || NOW - STEP > interval) {
    AUX0 = (AUX0 + 1) % (SEGLEN + trail_len);
    STEP = NOW;
  }

  for (int32_t i = 0; i < trail_len; i++) {
    int32_t pos = static_cast<int32_t>(AUX0) - trail_len + i;
    if (pos < 0 || pos >= SEGLEN)
      continue;
    uint8_t bright = static_cast<uint8_t>(255 - (trail_len - 1 - i) * 255 / trail_len);
    ctx.set_pixel(pos, color_fade(ctx.pal_color_at(pos), bright));
  }
  if (static_cast<int32_t>(AUX0) < SEGLEN) {
    ctx.set_pixel(AUX0, ctx.pal_color_at(AUX0));
  }
}

// ============================================================
// 27 — Noise 1D
// ============================================================
void fx_noise1d(EffectContext &ctx) {
  uint32_t scale = 1 + INTENSITY;
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint16_t x = static_cast<uint16_t>(i * scale + (NOW >> 2));
    uint8_t n = inoise8(x);
    ctx.set_pixel(i, ctx.pal_color(n));
  }
}

// ============================================================
// 28 — Palette (scrolling palette)
// ============================================================
void fx_palette(EffectContext &ctx) {
  uint32_t counter = (NOW * ((SPEED >> 3) + 1)) >> 5;
  int32_t shift = static_cast<int32_t>(counter & 0xFF);
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t idx = static_cast<uint8_t>((i * 255 / SEGLEN + shift) & 0xFF);
    ctx.set_pixel(i, ctx.pal_color(idx));
  }
}

// ============================================================
// 29 — Ripple
// ============================================================
struct RippleState {
  int32_t center;
  int32_t age;  // -1 = dead
  uint8_t color_idx;
};

void fx_ripple(EffectContext &ctx) {
  static constexpr int32_t MAX_RIPPLES = 5;
  static constexpr size_t DATA_SIZE = sizeof(RippleState) * MAX_RIPPLES;
  if (!ctx.env->allocate_data(DATA_SIZE)) {
    ctx.fill_black();
    return;
  }

  auto *ripples = reinterpret_cast<RippleState *>(ctx.env->data);

  // Init on first call
  if (CALL == 0) {
    for (int32_t i = 0; i < MAX_RIPPLES; i++)
      ripples[i].age = -1;
  }

  ctx.fade_to_black(200);

  // Spawn new ripple occasionally
  uint32_t interval = 100u + (255u - SPEED) * 10u;
  if (NOW - STEP > interval) {
    STEP = NOW;
    for (int32_t i = 0; i < MAX_RIPPLES; i++) {
      if (ripples[i].age < 0) {
        ripples[i].center = hw_random8() % SEGLEN;
        ripples[i].age = 0;
        ripples[i].color_idx = hw_random8();
        break;
      }
    }
  }

  int32_t max_radius = SEGLEN / 2 + 1;
  for (int32_t i = 0; i < MAX_RIPPLES; i++) {
    RippleState &r = ripples[i];
    if (r.age < 0)
      continue;

    int32_t radius = r.age;
    uint8_t fade = static_cast<uint8_t>(255 - r.age * 255 / max_radius);
    uint32_t col = color_fade(ctx.pal_color(r.color_idx), fade);

    ctx.set_pixel(r.center, col);
    for (int32_t d = 1; d <= radius; d++) {
      if (r.center - d >= 0)
        ctx.set_pixel(r.center - d, col);
      if (r.center + d < SEGLEN)
        ctx.set_pixel(r.center + d, col);
    }

    r.age++;
    if (r.age >= max_radius)
      r.age = -1;
  }
}

// ============================================================
// Batch 2 — effects 30-43
// ============================================================

// 30 — Juggle
void fx_juggle(EffectContext &ctx) {
  ctx.fade_to_black(192);
  uint8_t numdots = 1 + (INTENSITY >> 5);
  uint8_t max_pos = static_cast<uint8_t>(SEGLEN > 1 ? SEGLEN - 1 : 0);
  for (uint8_t i = 0; i < numdots; i++) {
    uint8_t pos = beatsin8(static_cast<uint8_t>(i * 2u + 7u), 0u, max_pos, NOW, static_cast<uint8_t>(i * 36u));
    ctx.set_pixel(static_cast<int32_t>(pos), ctx.pal_color_at(static_cast<int32_t>(pos)));
  }
}

// 31 — Bouncing Balls
void fx_bouncing_balls(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  struct Ball {
    float impact_vel;
    uint32_t last_frame;
    uint32_t color;
  };
  uint8_t num_balls = 1u + (INTENSITY >> 5u);
  if (!ctx.env->allocate_data(num_balls * sizeof(Ball))) {
    ctx.fill_black();
    return;
  }
  auto *balls = reinterpret_cast<Ball *>(ctx.env->data);

  if (CALL == 0) {
    for (uint8_t i = 0; i < num_balls; i++) {
      balls[i].impact_vel = 0.11f * (0.7f + i * 0.1f);
      balls[i].last_frame = 0;
      balls[i].color = ctx.pal_color(hw_random8());
    }
  }

  // Gravity in [0..1]/frame^2, speed-scaled
  float g = -0.006f * (0.5f + SPEED / 255.0f);

  ctx.fill_black();
  for (uint8_t i = 0; i < num_balls; i++) {
    float t = static_cast<float>(CALL - balls[i].last_frame);
    float h = balls[i].impact_vel * t + 0.5f * g * t * t;

    if (h <= 0.0f) {
      balls[i].impact_vel *= 0.82f;
      balls[i].last_frame = CALL;
      if (balls[i].impact_vel < 0.012f) {
        balls[i].impact_vel = 0.11f;
        balls[i].color = ctx.pal_color(hw_random8());
      }
      h = 0.0f;
    }

    int32_t pos = static_cast<int32_t>(h * static_cast<float>(SEGLEN - 1));
    if (pos >= 0 && pos < SEGLEN)
      ctx.set_pixel(pos, balls[i].color);
  }
}

// 32 — Fireworks
void fx_fireworks(EffectContext &ctx) {
  if (SEGLEN <= 3) {
    ctx.fill_black();
    return;
  }
  ctx.fade_to_black(220);
  uint32_t interval = 200u + (255u - SPEED) * 10u;
  if (NOW - STEP > interval) {
    STEP = NOW;
    int32_t pos = 1 + static_cast<int32_t>(hw_random16(static_cast<uint16_t>(SEGLEN - 2)));
    uint32_t col = ctx.pal_color(hw_random8());
    ctx.set_pixel(pos, col);
    ctx.set_pixel(pos - 1, color_fade(col, 100u));
    ctx.set_pixel(pos + 1, color_fade(col, 100u));
  }
}

// 33 — Police Lights
void fx_police(EffectContext &ctx) {
  int32_t half = SEGLEN / 2;
  uint32_t flash_cycle = 150u + (255u - SPEED) * 3u;
  bool state = (NOW / flash_cycle) % 2u == 0u;

  for (int32_t i = 0; i < SEGLEN; i++) {
    if (i < half)
      ctx.set_pixel(i, state ? 0xFF0000u : 0u);
    else
      ctx.set_pixel(i, state ? 0u : 0x0000FFu);
  }
}

// 34 — Chase Flash
void fx_chase_flash(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  uint32_t chase_period = 500u + (255u - SPEED) * 10u;
  uint32_t flash_cycle = 100u + (255u - SPEED) * 2u;

  // Brief white flash every 4th flash cycle
  if ((NOW / flash_cycle) % 4u == 0u) {
    ctx.fill(0xFFFFFFu);
    return;
  }

  ctx.fill(COLOR1);
  int32_t pos = static_cast<int32_t>((NOW % chase_period) * static_cast<uint32_t>(SEGLEN) / chase_period);
  int32_t dot_size = 1 + (INTENSITY >> 5);
  for (int32_t i = 0; i < dot_size && pos + i < SEGLEN; i++)
    ctx.set_pixel(pos + i, ctx.pal_color_at(pos + i));
}

// 35 — Heartbeat
void fx_heartbeat(EffectContext &ctx) {
  uint8_t bpm = 40u + (SPEED >> 4u);
  uint32_t ms_per_beat = 60000u / bpm;

  if (NOW - STEP >= ms_per_beat)
    STEP = NOW - (NOW % ms_per_beat);

  uint32_t t = NOW - STEP;
  uint8_t bri = 0;
  uint32_t pulse_ms = 25u;
  if (t < pulse_ms) {
    bri = sin8(static_cast<uint8_t>(t * 128u / pulse_ms));
  } else if (t >= ms_per_beat / 2u && t < ms_per_beat / 2u + pulse_ms) {
    uint32_t t2 = t - ms_per_beat / 2u;
    bri = scale8(sin8(static_cast<uint8_t>(t2 * 128u / pulse_ms)), 180u);
  }
  ctx.fill(color_fade(ctx.pal_color(0), bri));
}

// 36 — Rain
void fx_rain(EffectContext &ctx) {
  struct Drop {
    int32_t pos;
    uint8_t bright;
    uint8_t hue;
  };
  static constexpr uint8_t MAX_DROPS = 12;
  if (!ctx.env->allocate_data(MAX_DROPS * sizeof(Drop))) {
    ctx.fill_black();
    return;
  }
  auto *drops = reinterpret_cast<Drop *>(ctx.env->data);

  if (CALL == 0) {
    for (uint8_t i = 0; i < MAX_DROPS; i++)
      drops[i].pos = -1;
  }

  ctx.fade_to_black(230u);

  uint32_t interval = 20u + (255u - SPEED) * 3u;
  if (NOW - STEP > interval) {
    STEP = NOW;
    for (uint8_t i = 0; i < MAX_DROPS; i++) {
      if (drops[i].pos >= 0) {
        drops[i].pos++;
        drops[i].bright = scale8(drops[i].bright, 220u);
        if (drops[i].pos >= SEGLEN)
          drops[i].pos = -1;
      }
    }
    if (hw_random8() < INTENSITY) {
      for (uint8_t i = 0; i < MAX_DROPS; i++) {
        if (drops[i].pos < 0) {
          drops[i].pos = 0;
          drops[i].bright = 255;
          drops[i].hue = hw_random8();
          break;
        }
      }
    }
  }

  for (uint8_t i = 0; i < MAX_DROPS; i++) {
    if (drops[i].pos >= 0 && drops[i].pos < SEGLEN)
      ctx.set_pixel(drops[i].pos, color_fade(ctx.wheel(drops[i].hue), drops[i].bright));
  }
}

// 37 — Sparkle
void fx_sparkle(EffectContext &ctx) {
  ctx.fill(COLOR0);
  uint32_t interval = 10u + (255u - SPEED) * 2u;
  if (NOW - STEP > interval) {
    STEP = NOW;
    uint8_t n = 1u + (INTENSITY >> 4u);
    for (uint8_t i = 0; i < n; i++)
      ctx.set_pixel(static_cast<int32_t>(hw_random16(static_cast<uint16_t>(SEGLEN))), 0xFFFFFFu);
  }
}

// 38 — Pride 2015 (Mark Kriegsman, simplified)
void fx_pride_2015(EffectContext &ctx) {
  uint8_t sat = beatsin8(87u, 220u, 250u, NOW);
  // 341 truncates to 85 (uint8_t); keep original WLED intent (slow variation)
  uint8_t bright_depth = beatsin8(85u, 96u, 224u, NOW);
  uint16_t bright_theta_inc = static_cast<uint16_t>(beatsin8(203u, 25u, 75u, NOW)) * 256u;
  uint16_t hue16 = static_cast<uint16_t>(NOW * (1u + (SPEED >> 5u)));
  uint16_t hue_inc = static_cast<uint16_t>(beatsin8(113u, 1u, 3u, NOW)) * 256u;
  uint16_t bright_theta = static_cast<uint16_t>(NOW * 61u);

  for (int32_t i = 0; i < SEGLEN; i++) {
    hue16 += hue_inc;
    bright_theta += bright_theta_inc;
    uint8_t b = sin8(static_cast<uint8_t>(bright_theta >> 8u));
    b = 255u - scale8(255u - b, bright_depth);
    uint32_t col = color_fade(ctx.wheel(static_cast<uint8_t>(hue16 >> 8u)), b);
    col = color_blend(col, color_fade(0xFFFFFFu, b), static_cast<uint8_t>(255u - sat));
    ctx.set_pixel(i, col);
  }
}

// 39 — Candle
void fx_candle(EffectContext &ctx) {
  if (CALL == 0)
    AUX0 = 150u;
  uint32_t interval = 30u + (255u - SPEED) * 3u;
  if (NOW - STEP > interval) {
    STEP = NOW;
    int16_t delta =
        static_cast<int16_t>(hw_random8(static_cast<uint8_t>(INTENSITY >> 2u))) - static_cast<int16_t>(INTENSITY >> 3u);
    int16_t bri = static_cast<int16_t>(AUX0) + delta;
    if (bri > 200)
      bri = 200;
    if (bri < 40)
      bri = 40;
    AUX0 = static_cast<uint16_t>(bri);
  }
  ctx.fill(color_fade(COLOR0, static_cast<uint8_t>(AUX0)));
}

// 40 — Fill Noise
void fx_fill_noise(EffectContext &ctx) {
  uint16_t time_speed = 1u + (SPEED >> 3u);
  uint16_t space_scale = 1u + (INTENSITY >> 4u);
  uint32_t t = NOW * time_speed;
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint16_t x = static_cast<uint16_t>(i * space_scale);
    uint8_t n = inoise8(x, static_cast<uint16_t>(t >> 1u));
    ctx.set_pixel(i, ctx.pal_color(n));
  }
}

// 41 — Oscillate
void fx_oscillate(EffectContext &ctx) {
  ctx.fill_black();
  uint8_t num = 1u + (INTENSITY >> 5u);
  for (uint8_t i = 0; i < num; i++) {
    uint16_t bpm = static_cast<uint16_t>(SPEED / 8u + i * 3u + 3u);
    int32_t pos = static_cast<int32_t>(beatsin16(bpm, 0u, static_cast<uint16_t>(SEGLEN > 0u ? SEGLEN - 1u : 0u), NOW,
                                                 static_cast<uint16_t>(i * (65535u / (num > 0u ? num : 1u)))));
    uint32_t col = ctx.wheel(static_cast<uint8_t>(i * 255u / num));
    ctx.set_pixel(pos, col);
    if (pos > 0)
      ctx.set_pixel(pos - 1, color_fade(col, 128u));
    if (pos < SEGLEN - 1)
      ctx.set_pixel(pos + 1, color_fade(col, 128u));
  }
}

// 42 — Gradient
void fx_gradient(EffectContext &ctx) {
  uint32_t offset = (NOW * SPEED) >> 8u;
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t blend = static_cast<uint8_t>(
        (static_cast<uint32_t>(i) * 255u / static_cast<uint32_t>(SEGLEN > 0 ? SEGLEN : 1) + offset) & 0xFFu);
    ctx.set_pixel(i, ctx.params->palette_id != 0u ? ctx.pal_color(blend) : color_blend(COLOR0, COLOR1, blend));
  }
}

// 43 — Pacifica (simplified ocean waves)
void fx_pacifica(EffectContext &ctx) {
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t ci = static_cast<uint8_t>(static_cast<uint32_t>(i) * 255u / static_cast<uint32_t>(SEGLEN > 0 ? SEGLEN : 1));
    uint8_t w1 = sin8(static_cast<uint8_t>((ci * 3u + (NOW >> 3u)) & 0xFFu));
    uint8_t w2 = sin8(static_cast<uint8_t>((ci * 5u - (NOW >> 2u)) & 0xFFu));
    uint8_t w3 = sin8(static_cast<uint8_t>((ci * 7u + (NOW >> 2u)) & 0xFFu));
    uint8_t wave = static_cast<uint8_t>((static_cast<uint16_t>(w1) + w2 + w3) / 3u);
    ctx.set_pixel(i, RGBW32(scale8(wave, 10u), scale8(wave, 100u), scale8(wave, 255u)));
  }
}

// ============================================================
// Batch 3 — effects 44-63
// ============================================================

// 44 — Sinelon (stolen from FastLED examples, ported from WLED)
void fx_sinelon(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  ctx.fade_to_black(INTENSITY > 0 ? INTENSITY : 1);
  uint16_t bpm = SPEED / 10u;
  if (bpm < 1)
    bpm = 1;
  int32_t pos = static_cast<int32_t>(beatsin16(bpm, 0u, static_cast<uint16_t>(SEGLEN - 1), NOW));
  if (CALL == 0)
    AUX0 = static_cast<uint16_t>(pos);
  uint32_t col = ctx.pal_color_at(pos);
  // fill any skipped pixels between last pos and current pos
  if (static_cast<int32_t>(AUX0) < pos) {
    for (int32_t i = static_cast<int32_t>(AUX0); i <= pos; i++)
      ctx.set_pixel(i, col);
  } else if (static_cast<int32_t>(AUX0) > pos) {
    for (int32_t i = pos; i <= static_cast<int32_t>(AUX0); i++)
      ctx.set_pixel(i, col);
  } else {
    ctx.set_pixel(pos, col);
  }
  AUX0 = static_cast<uint16_t>(pos);
}

// 45 — Dissolve (random pixel reveal, alternating fill/unfill)
void fx_dissolve(EffectContext &ctx) {
  size_t data_size = static_cast<size_t>(SEGLEN) * sizeof(uint32_t);
  if (!ctx.env->allocate_data(data_size)) {
    ctx.fill_black();
    return;
  }
  auto *pixels = reinterpret_cast<uint32_t *>(ctx.env->data);

  if (CALL == 0) {
    for (int32_t i = 0; i < SEGLEN; i++)
      pixels[i] = COLOR1;
    AUX0 = 1;  // dissolve-in direction
    STEP = 0;
  }

  uint32_t attempts = 1 + static_cast<uint32_t>(SEGLEN) / 15u;
  for (uint32_t j = 0; j < attempts; j++) {
    if (hw_random8() <= INTENSITY) {
      for (uint8_t t = 0; t < 10; t++) {
        uint16_t idx = hw_random16(static_cast<uint16_t>(SEGLEN));
        if (AUX0) {
          if (pixels[idx] == COLOR1) {
            pixels[idx] = ctx.pal_color_at(static_cast<int32_t>(idx));
            break;
          }
        } else {
          if (pixels[idx] != COLOR1) {
            pixels[idx] = COLOR1;
            break;
          }
        }
      }
    }
  }

  for (int32_t i = 0; i < SEGLEN; i++)
    ctx.set_pixel(i, pixels[i]);

  STEP++;
  if (STEP > static_cast<uint32_t>(255u - SPEED) + 15u) {
    AUX0 ^= 1u;
    STEP = 0;
  }
}

// 46 — Android (growing/shrinking bar that chases itself)
void fx_android(EffectContext &ctx) {
  if (!ctx.env->allocate_data(sizeof(uint32_t))) {
    ctx.fill(COLOR0);
    return;
  }
  auto *counter = reinterpret_cast<uint32_t *>(ctx.env->data);

  uint16_t size = AUX1 >> 1;
  uint8_t shrinking = AUX1 & 0x01u;

  uint32_t interval = 3u + (8u * static_cast<uint32_t>(255u - SPEED)) / static_cast<uint32_t>(SEGLEN > 1 ? SEGLEN : 1);
  if (NOW - STEP >= interval) {
    STEP = NOW;
    uint16_t max_size = static_cast<uint16_t>((static_cast<uint32_t>(INTENSITY) * SEGLEN) / 255u);
    if (size > max_size)
      shrinking = 1;
    else if (size < 2)
      shrinking = 0;

    if (!shrinking) {
      if ((*counter % 3u) == 1u)
        AUX0++;
      else
        size++;
    } else {
      AUX0++;
      if ((*counter % 3u) != 1u && size > 0u)
        size--;
    }
    (*counter)++;
    if (AUX0 >= static_cast<uint16_t>(SEGLEN))
      AUX0 = 0;
    AUX1 = static_cast<uint16_t>((size << 1u) | shrinking);
  }

  uint16_t start = AUX0;
  uint16_t end_pos = static_cast<uint16_t>((AUX0 + size) % static_cast<uint16_t>(SEGLEN));
  for (int32_t i = 0; i < SEGLEN; i++) {
    bool in_bar;
    if (start < end_pos)
      in_bar = (static_cast<uint16_t>(i) >= start && static_cast<uint16_t>(i) < end_pos);
    else
      in_bar = (static_cast<uint16_t>(i) >= start || static_cast<uint16_t>(i) < end_pos);
    ctx.set_pixel(i, in_bar ? COLOR0 : ctx.pal_color_at(i));
  }
}

// 47 — Chase Rainbow (rainbow dot chasing on COLOR0/COLOR1 background)
void fx_chase_rainbow(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  uint16_t counter = static_cast<uint16_t>((NOW * ((SPEED >> 2u) + 1u)) >> 0u);
  uint32_t a = static_cast<uint32_t>((static_cast<uint32_t>(counter) * static_cast<uint32_t>(SEGLEN)) >> 16u);
  uint32_t dot_w = 1u + (INTENSITY >> 5u);

  uint8_t color_idx = static_cast<uint8_t>(((a * 256u / static_cast<uint32_t>(SEGLEN)) + (CALL & 0xFFu)) & 0xFFu);
  uint32_t dot_col = ctx.wheel(color_idx);

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint32_t pos_in_a = static_cast<uint32_t>(i) % static_cast<uint32_t>(SEGLEN);
    bool is_dot = (pos_in_a >= a && pos_in_a < a + dot_w);
    bool is_trail = (dot_w > 1 && pos_in_a == (a + dot_w) % static_cast<uint32_t>(SEGLEN));
    if (is_dot)
      ctx.set_pixel(i, dot_col);
    else if (is_trail)
      ctx.set_pixel(i, color_blend(dot_col, COLOR1, 128u));
    else
      ctx.set_pixel(i, COLOR1);
  }
}

// 48 — Colorful (rotating RAGB color blocks)
void fx_colorful(EffectContext &ctx) {
  static const uint32_t DEFAULT_COLS[4] = {0xFF0000u, 0xEEBB00u, 0x00EE00u, 0x0077CCu};
  static const uint32_t PASTEL_COLS[4] = {0xFF8040u, 0xE5D241u, 0x77FF77u, 0x77F0F0u};

  uint32_t cols[8];
  uint8_t num = 4u;

  if (INTENSITY > 160u || ctx.params->palette_id != 0u) {
    if (ctx.params->palette_id == 0u) {
      num = 3u;
      cols[0] = COLOR0;
      cols[1] = COLOR1;
      cols[2] = COLOR2;
    } else {
      for (uint8_t i = 0; i < num; i++)
        cols[i] = ctx.pal_color(static_cast<uint8_t>(i * 64u));
    }
  } else if (INTENSITY < 80u) {
    for (uint8_t i = 0; i < num; i++)
      cols[i] = PASTEL_COLS[i];
  } else {
    for (uint8_t i = 0; i < num; i++)
      cols[i] = DEFAULT_COLS[i];
  }
  // duplicate for wrap
  for (uint8_t i = num; i < num * 2u - 1u; i++)
    cols[i] = cols[i - num];

  uint32_t cycleTime = 50u + 8u * static_cast<uint32_t>(255u - SPEED);
  uint32_t it = NOW / cycleTime;
  if (it != STEP) {
    if (SPEED > 0u)
      AUX0 = (AUX0 + 1u) % num;
    STEP = it;
  }

  for (int32_t i = 0; i < SEGLEN; i += static_cast<int32_t>(num)) {
    for (uint8_t j = 0; j < num; j++) {
      if (i + static_cast<int32_t>(j) < SEGLEN)
        ctx.set_pixel(i + static_cast<int32_t>(j), cols[AUX0 + j]);
    }
  }
}

// 49 — Fire Flicker (random brightness per pixel)
void fx_fire_flicker(EffectContext &ctx) {
  uint32_t cycleTime = 40u + static_cast<uint32_t>(255u - SPEED);
  uint32_t it = NOW / cycleTime;
  if (it == STEP)
    return;
  STEP = it;

  uint8_t r = static_cast<uint8_t>(COLOR0 >> 16u);
  uint8_t g = static_cast<uint8_t>(COLOR0 >> 8u);
  uint8_t b = static_cast<uint8_t>(COLOR0);
  uint8_t lum = (ctx.params->palette_id == 0u) ? static_cast<uint8_t>(r > g ? (r > b ? r : b) : (g > b ? g : b)) : 255u;
  lum /= static_cast<uint8_t>(((256u - INTENSITY) / 16u) + 1u);

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t flicker = hw_random8(lum > 0u ? lum : 1u);
    if (ctx.params->palette_id == 0u) {
      ctx.set_pixel(i, RGBW32(static_cast<uint8_t>(r > flicker ? r - flicker : 0u),
                              static_cast<uint8_t>(g > flicker ? g - flicker : 0u),
                              static_cast<uint8_t>(b > flicker ? b - flicker : 0u)));
    } else {
      ctx.set_pixel(i, color_fade(ctx.pal_color_at(i), static_cast<uint8_t>(255u - flicker)));
    }
  }
}

// 50 — Two Dots (two dots running in opposite directions)
void fx_two_dots(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  // speed-scaled iteration counter
  uint32_t delay_base = 1u + (static_cast<uint32_t>(FRAMETIME_MS) << 3u) / static_cast<uint32_t>(SEGLEN);
  uint32_t period_hi = delay_base << 4u;
  uint32_t period_lo = delay_base;
  uint32_t period = period_lo + (static_cast<uint32_t>(255u - SPEED) * (period_hi - period_lo)) / 255u;
  if (period < 1u)
    period = 1u;
  uint32_t it = NOW / period;
  uint32_t offset = it % static_cast<uint32_t>(SEGLEN);
  uint32_t width = 1u + ((static_cast<uint32_t>(SEGLEN) * (INTENSITY + 1u)) >> 9u);
  if (width < 1u)
    width = 1u;

  uint32_t col2 = (COLOR1 == COLOR2) ? COLOR0 : COLOR1;
  ctx.fill(COLOR2);
  for (uint32_t i = 0; i < width; i++) {
    uint32_t idx_a = (offset + i) % static_cast<uint32_t>(SEGLEN);
    uint32_t idx_b = (offset + i + static_cast<uint32_t>(SEGLEN) / 2u) % static_cast<uint32_t>(SEGLEN);
    ctx.set_pixel(static_cast<int32_t>(idx_a), COLOR0);
    ctx.set_pixel(static_cast<int32_t>(idx_b), col2);
  }
}

// 51 — Tricolor Chase (3-color chasing sections)
void fx_tricolor_chase(EffectContext &ctx) {
  uint32_t cycleTime = 50u + (static_cast<uint32_t>(255u - SPEED) << 1u);
  uint32_t it = NOW / cycleTime;
  uint32_t width = 1u + (INTENSITY >> 4u);
  uint32_t index = it % (width * 3u);

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint32_t j = (index + static_cast<uint32_t>(i)) % (width * 3u);
    uint32_t col;
    if (j < width)
      col = COLOR2;
    else if (j < width * 2u)
      col = COLOR0;
    else
      col = ctx.pal_color_at(SEGLEN - 1 - i);
    ctx.set_pixel(SEGLEN - 1 - i, col);
  }
}

// 52 — ICU (two "eyes" scanning left and right)
void fx_icu(EffectContext &ctx) {
  if (SEGLEN <= 2) {
    ctx.fill(COLOR0);
    return;
  }
  uint32_t space = static_cast<uint32_t>(INTENSITY >> 3u) + 2u;
  uint32_t max_dest = static_cast<uint32_t>(SEGLEN) - static_cast<uint32_t>(SEGLEN) / space;

  // State and timing packed in STEP: upper 16 = state, lower 16 = next update (ms, low16)
  uint16_t state = static_cast<uint16_t>(STEP >> 16u);
  uint16_t next_upd = static_cast<uint16_t>(STEP & 0xFFFFu);
  uint16_t dest = AUX1;
  uint16_t now16 = static_cast<uint16_t>(NOW & 0xFFFFu);

  uint32_t speed_delay =
      5u + (50u * static_cast<uint32_t>(255u - SPEED)) / static_cast<uint32_t>(SEGLEN > 1 ? SEGLEN : 1);

  uint32_t bgcol = COLOR1;
  uint32_t eye_col = ctx.pal_color(static_cast<uint8_t>((dest * 255u) / (max_dest > 0u ? max_dest : 1u)));
  ctx.fill(bgcol);

  if (state != 1u) {
    ctx.set_pixel(static_cast<int32_t>(dest), eye_col);
    ctx.set_pixel(static_cast<int32_t>(dest + static_cast<uint32_t>(SEGLEN) / space), eye_col);
  }

  if (static_cast<int16_t>(now16 - next_upd) >= 0) {
    switch (state) {
      case 0:
        state = 1u;
        if (hw_random8(6u) == 0u) {
          next_upd = static_cast<uint16_t>(now16 + 200u);
          break;
        }
        // fall-through to blink-end
      case 1:
        next_upd = static_cast<uint16_t>(now16 + 500u + hw_random16(1000u));
        state = 2u;
        break;
      case 2:
        AUX0 = static_cast<uint16_t>(hw_random16(static_cast<uint16_t>(max_dest)));
        next_upd = now16;
        state = 3u;
        break;
      default:  // 3 — move toward dest
        AUX1 = dest;
        next_upd = static_cast<uint16_t>(now16 + static_cast<uint16_t>(speed_delay));
        if (AUX0 == dest) {
          next_upd = static_cast<uint16_t>(now16 + 500u + hw_random16(1000u));
          state = 0u;
        } else if (AUX0 > dest) {
          AUX1++;
        } else {
          if (AUX1 > 0u)
            AUX1--;
        }
        break;
    }
  }
  STEP = (static_cast<uint32_t>(state) << 16u) | static_cast<uint32_t>(next_upd);
}

// 53 — Lightning
void fx_lightning(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }

  ctx.fill(COLOR1);

  if (AUX1 == 0u) {
    // Init: leader flash
    AUX1 = static_cast<uint16_t>(hw_random8(4u, 4u + INTENSITY / 20u));
    AUX1 *= 2u;
    AUX0 = 200u;  // ms delay after leader
    STEP = NOW;

    // Draw leader at lower brightness
    uint16_t led_start = hw_random16(static_cast<uint16_t>(SEGLEN));
    uint16_t led_len = 1u + hw_random16(static_cast<uint16_t>(SEGLEN) - led_start);
    for (uint16_t i = led_start; i < led_start + led_len; i++)
      ctx.set_pixel(static_cast<int32_t>(i), color_fade(ctx.pal_color_at(static_cast<int32_t>(i)), 52u));
    return;
  }

  if (AUX1 > 3u && !(AUX1 & 0x01u)) {
    // Flash on
    uint16_t led_start = hw_random16(static_cast<uint16_t>(SEGLEN));
    uint16_t led_len = 1u + hw_random16(static_cast<uint16_t>(SEGLEN) - led_start);
    uint8_t bri = 255u / hw_random8(1u, 3u);
    for (uint16_t i = led_start; i < led_start + led_len; i++)
      ctx.set_pixel(static_cast<int32_t>(i), color_fade(ctx.pal_color_at(static_cast<int32_t>(i)), bri));
    AUX1--;
    STEP = NOW;
  } else {
    if (NOW - STEP > static_cast<uint32_t>(AUX0)) {
      AUX1--;
      if (AUX1 < 2u)
        AUX1 = 0u;
      AUX0 = static_cast<uint16_t>(50u + hw_random8(100u));
      if (AUX1 == 2u)
        AUX0 = static_cast<uint16_t>(static_cast<uint32_t>(hw_random8(255u - SPEED)) * 100u);
      STEP = NOW;
    }
  }
}

// 54 — Glitter (scrolling palette background + white glitter)
void fx_glitter(EffectContext &ctx) {
  // Scrolling palette background
  uint32_t counter = 0u;
  if (SPEED != 0u) {
    counter = (NOW * (static_cast<uint32_t>(SPEED >> 3u) + 1u)) & 0xFFFFu;
    counter >>= 8u;
  }
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t idx = static_cast<uint8_t>(
        (static_cast<uint32_t>(i) * 255u / static_cast<uint32_t>(SEGLEN > 1 ? SEGLEN : 1) - counter) & 0xFFu);
    ctx.set_pixel(i, ctx.pal_color(idx));
  }
  // Glitter
  if (INTENSITY > hw_random8())
    ctx.set_pixel(static_cast<int32_t>(hw_random16(static_cast<uint16_t>(SEGLEN))), 0xFFFFFFu);
}

// 55 — Solid Glitter (solid COLOR0 + white glitter)
void fx_solid_glitter(EffectContext &ctx) {
  ctx.fill(COLOR0);
  if (INTENSITY > hw_random8())
    ctx.set_pixel(static_cast<int32_t>(hw_random16(static_cast<uint16_t>(SEGLEN))), 0xFFFFFFu);
}

// 56 — Spots (bright zones on dim background, width from speed)
void fx_spots(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  ctx.fill(COLOR1);
  uint32_t threshold = static_cast<uint32_t>(255u - SPEED) << 8u;

  uint32_t max_zones = static_cast<uint32_t>(SEGLEN) >> 2u;
  uint32_t zones = 1u + ((static_cast<uint32_t>(INTENSITY) * max_zones) >> 8u);
  uint32_t zone_len = static_cast<uint32_t>(SEGLEN) / zones;
  uint32_t offset = (static_cast<uint32_t>(SEGLEN) - zones * zone_len) >> 1u;

  for (uint32_t z = 0u; z < zones; z++) {
    uint32_t pos = offset + z * zone_len;
    for (uint32_t i = 0u; i < zone_len; i++) {
      // triwave16 of position within zone
      uint32_t wave = (i < zone_len / 2u) ? (i * 0xFFFFu) / zone_len : ((zone_len - 1u - i) * 0xFFFFu) / zone_len;
      wave = (wave * 2u);
      if (wave > 0xFFFFu)
        wave = 0xFFFFu;
      if (wave > threshold) {
        uint32_t s = (wave - threshold) * 255u / (0xFFFFu - threshold);
        int32_t idx = static_cast<int32_t>(pos + i);
        if (idx < SEGLEN)
          ctx.set_pixel(idx, color_blend(ctx.pal_color_at(idx), COLOR1, static_cast<uint8_t>(255u - s)));
      }
    }
  }
}

// 57 — Percent (fill percentage of strip with two colors)
void fx_percent(EffectContext &ctx) {
  uint32_t percent = INTENSITY;
  if (percent > 200u)
    percent = 200u;

  uint32_t active_leds;
  if (percent < 100u)
    active_leds = static_cast<uint32_t>((static_cast<float>(SEGLEN) * percent) / 100.0f + 0.5f);
  else
    active_leds = static_cast<uint32_t>((static_cast<float>(SEGLEN) * (200u - percent)) / 100.0f + 0.5f);

  uint32_t size = 1u + ((static_cast<uint32_t>(SPEED) * static_cast<uint32_t>(SEGLEN)) >> 11u);
  if (SPEED == 255u)
    size = 255u;

  // Smooth AUX1 toward target
  if (active_leds > AUX1) {
    AUX1 = static_cast<uint16_t>(AUX1 + size < active_leds ? AUX1 + size : active_leds);
  } else if (active_leds < AUX1) {
    AUX1 = static_cast<uint16_t>(AUX1 > size ? AUX1 - size : 0u);
    if (AUX1 < active_leds)
      AUX1 = static_cast<uint16_t>(active_leds);
  }

  if (percent <= 100u) {
    for (int32_t i = 0; i < SEGLEN; i++)
      ctx.set_pixel(i, static_cast<uint32_t>(i) < AUX1 ? ctx.pal_color_at(i) : COLOR1);
  } else {
    for (int32_t i = 0; i < SEGLEN; i++)
      ctx.set_pixel(i, static_cast<uint32_t>(i) < static_cast<uint32_t>(SEGLEN) - AUX1 ? COLOR1 : ctx.pal_color_at(i));
  }
}

// 58 — Flow (alternating palette zones)
void fx_flow(EffectContext &ctx) {
  uint32_t counter = 0u;
  if (SPEED != 0u) {
    counter = (NOW * (static_cast<uint32_t>(SPEED >> 2u) + 1u)) >> 8u;
  }

  uint32_t max_zones = static_cast<uint32_t>(SEGLEN) / 6u;
  if (max_zones < 1u)
    max_zones = 1u;
  int32_t zones = static_cast<int32_t>((static_cast<uint32_t>(INTENSITY) * max_zones) >> 8u);
  if (zones & 0x01)
    zones++;
  if (zones < 2)
    zones = 2;
  int32_t zone_len = SEGLEN / zones;
  int32_t req_zones = (SEGLEN + zone_len - 1) / zone_len;
  zones = req_zones + 2;
  int32_t offset_px = (SEGLEN - zones * zone_len) / 2;

  for (int32_t z = 0; z < zones; z++) {
    int32_t pos = offset_px + z * zone_len;
    for (int32_t i = 0; i < zone_len; i++) {
      uint8_t color_idx = static_cast<uint8_t>(
          (static_cast<uint32_t>(i) * 255u / static_cast<uint32_t>(zone_len > 1 ? zone_len : 1) - counter) & 0xFFu);
      int32_t led = (z & 0x01) ? i : (zone_len - 1 - i);
      int32_t px = pos + led;
      if (px >= 0 && px < SEGLEN)
        ctx.set_pixel(px, ctx.pal_color(color_idx));
    }
  }
}

// 59 — Phased (phased sine waves, palette-colored)
void fx_phased(EffectContext &ctx) {
  // phase stored as float in STEP (uint32_t bit-cast)
  float *phase_ptr = reinterpret_cast<float *>(&STEP);
  *phase_ptr += static_cast<float>(SPEED) / 32.0f;

  unsigned allfreq = 16u;
  unsigned cutoff = 255u - INTENSITY;
  unsigned index = NOW / 64u;

  for (int32_t i = 0; i < SEGLEN; i++) {
    unsigned val = static_cast<unsigned>(i + 1u) * allfreq;
    unsigned mod_val = 5u;
    val = static_cast<unsigned>(val + *phase_ptr * static_cast<float>((i % mod_val) + 1u) / 2.0f);
    uint8_t b = cubicwave8(static_cast<uint8_t>(val & 0xFFu));
    b = (b > cutoff) ? (b - static_cast<uint8_t>(cutoff)) : 0u;
    ctx.set_pixel(i, color_blend(COLOR1, ctx.pal_color(static_cast<uint8_t>(index & 0xFFu)), b));
    index += 256u / static_cast<unsigned>(SEGLEN > 0 ? SEGLEN : 1);
  }
}

// 60 — Sinewave (cubic wave with palette coloring, by Andrew Tuline)
void fx_sinewave(EffectContext &ctx) {
  STEP += SPEED / 16u;
  uint32_t freq = INTENSITY / 4u;
  uint32_t color_idx = NOW / 32u;

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t pix_bri = cubicwave8(static_cast<uint8_t>((static_cast<uint32_t>(i) * freq + STEP) & 0xFFu));
    uint8_t pal_idx = static_cast<uint8_t>(
        (static_cast<uint32_t>(i) * color_idx / static_cast<uint32_t>(SEGLEN > 1u ? SEGLEN : 1u)) & 0xFFu);
    ctx.set_pixel(i, color_blend(COLOR1, ctx.pal_color(pal_idx), pix_bri));
  }
}

// 61 — Twinkleup (noise-based twinkle with fade-in, by Andrew Tuline)
void fx_twinkleup(EffectContext &ctx) {
  // Use a fixed seed per frame so every pixel draws consistently this frame
  uint16_t prev_seed = g_prng.get_seed();
  g_prng.set_seed(535u);

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t ran_start = hw_random8();
    uint8_t pix_bri = sin8(static_cast<uint8_t>(ran_start + 16u * NOW / (256u - (SPEED > 0u ? SPEED : 1u))));
    if (hw_random8() > INTENSITY)
      pix_bri = 0u;
    uint8_t pal_idx = static_cast<uint8_t>((hw_random8() + NOW / 100u) & 0xFFu);
    ctx.set_pixel(i, color_blend(COLOR1, ctx.pal_color(pal_idx), pix_bri));
  }

  g_prng.set_seed(prev_seed);
}

// 62 — Colorwaves (palette colorwaves, Mark Kriegsman)
void fx_colorwaves(EffectContext &ctx) {
  // sPseudotime and sHue16 persisted in STEP/AUX0
  uint32_t s_pseudotime = STEP;
  uint16_t s_hue16 = AUX0;

  unsigned duration = 10u + SPEED;
  unsigned bright_depth = beatsin8(85u, 96u, 224u, NOW);
  unsigned bright_theta_inc16 = static_cast<uint16_t>(beatsin8(203u, 25u, 40u, NOW)) * 256u;
  unsigned ms_mult = beatsin8(147u, 23u, 60u, NOW);
  unsigned hue16 = s_hue16;
  uint32_t hue_inc16 =
      static_cast<uint32_t>(beatsin8(113u, 0u, 255u, NOW)) * static_cast<uint32_t>(INTENSITY) * 10u / 255u;

  s_pseudotime += duration * ms_mult;
  s_hue16 = static_cast<uint16_t>(s_hue16 + duration * beatsin8(40u, 5u, 9u, NOW));
  uint32_t bright_theta = s_pseudotime;

  for (int32_t i = 0; i < SEGLEN; i++) {
    hue16 += hue_inc16;
    uint32_t h16_128 = hue16 >> 7u;
    uint8_t hue8 = static_cast<uint8_t>((h16_128 & 0x100u) ? (255u - (h16_128 >> 1u)) : (h16_128 >> 1u));
    bright_theta += bright_theta_inc16;
    uint16_t b16 = static_cast<uint16_t>(sin16(static_cast<uint16_t>(bright_theta >> 16u)) + 32768);
    uint32_t bri16 = (static_cast<uint32_t>(b16) * b16) / 65536u;
    uint8_t bri8 = static_cast<uint8_t>((bri16 * bright_depth) / 65536u + (255u - bright_depth));
    uint32_t new_col = color_fade(ctx.pal_color(hue8), bri8);
    // blend into existing pixel (128/256 = 50% blend like WLED)
    int32_t abs_px = ctx.map_pixel(i);
    if (abs_px >= 0 && abs_px < static_cast<int32_t>(ctx.frame_len)) {
      uint32_t old_col = ctx.frame_buf[abs_px];
      ctx.set_pixel(i, color_blend(old_col, new_col, 128u));
    } else {
      ctx.set_pixel(i, new_col);
    }
  }

  STEP = s_pseudotime;
  AUX0 = s_hue16;
}

// 63 — Chunchun (pendulum birds in a sine wave, by Aircoookie)
void fx_chunchun(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  ctx.fade_to_black(254u);
  uint32_t counter = NOW * (6u + (SPEED >> 4u));
  uint32_t num_birds = 2u + static_cast<uint32_t>(SEGLEN) / 8u;
  uint32_t span = (static_cast<uint32_t>(INTENSITY) << 8u) / (num_birds > 0u ? num_birds : 1u);

  for (uint32_t i = 0u; i < num_birds; i++) {
    counter -= span;
    uint32_t megumin = static_cast<uint32_t>(sin16(static_cast<uint16_t>(counter)) + 32768);
    uint32_t bird = (megumin * static_cast<uint32_t>(SEGLEN)) >> 16u;
    if (bird >= static_cast<uint32_t>(SEGLEN))
      bird = static_cast<uint32_t>(SEGLEN) - 1u;
    uint8_t pal_idx = static_cast<uint8_t>((i * 255u) / (num_birds > 1u ? num_birds : 1u));
    ctx.set_pixel(static_cast<int32_t>(bird), ctx.pal_color(pal_idx));
  }
}

// ============================================================
// Batch 4 — effects 64-83
// ============================================================

// 64 — Stream (Running Random)
// Source: WLED mode_running_random (FX.cpp line 1173)
// Zones of solid colour from a seeded PRNG scroll across the strip.
void fx_stream(EffectContext &ctx) {
  uint32_t cycleTime = 25u + 3u * static_cast<uint32_t>(255u - SPEED);
  uint32_t it = NOW / cycleTime;
  if (CALL == 0)
    AUX0 = hw_random16();  // random seed

  uint32_t zoneSize = static_cast<uint32_t>((255u - INTENSITY) >> 4u) + 1u;
  uint16_t prng16 = static_cast<uint16_t>(AUX0);

  uint32_t z = it % zoneSize;
  bool nzone = (!z && it != STEP);
  for (int32_t i = SEGLEN - 1; i >= 0; i--) {
    if (nzone || z >= zoneSize) {
      uint16_t lastrand = prng16 >> 8u;
      int16_t diff = 0;
      while (diff < 42 && diff > -42) {
        prng16 = static_cast<uint16_t>(prng16 * 2053u) + 13849u;
        diff = static_cast<int16_t>(static_cast<int16_t>(prng16 >> 8u) - static_cast<int16_t>(lastrand));
      }
      if (nzone) {
        AUX0 = prng16;
        nzone = false;
      }
      z = 0u;
    }
    ctx.set_pixel(i, ctx.wheel(static_cast<uint8_t>(prng16 >> 8u)));
    z++;
  }
  STEP = it;
}

// 65 — Scanner Dual (two Larson eyes moving in opposite directions)
// Source: WLED mode_dual_larson_scanner (FX.cpp line 1256)
// Implemented as two independent scanners on the same strip.
void fx_scanner_dual(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  int32_t width = 1 + (INTENSITY >> 5);
  ctx.fade_to_black(192);

  uint32_t interval = 10u + (255u - SPEED) * 3u;
  if (STEP == 0 || NOW - STEP > interval) {
    // AUX0 encodes direction (bit0) + position (bits 1-15)
    uint16_t pos = AUX1;
    bool fwd = (AUX0 & 0x01u) == 0u;
    if (fwd) {
      pos++;
      if (static_cast<int32_t>(pos) >= SEGLEN - width)
        AUX0 |= 0x01u;
    } else {
      if (pos == 0u)
        AUX0 &= ~0x01u;
      else
        pos--;
    }
    AUX1 = pos;
    STEP = NOW;
  }

  int32_t pos_a = static_cast<int32_t>(AUX1);
  int32_t pos_b = SEGLEN - 1 - pos_a - width + 1;
  for (int32_t i = 0; i < width; i++) {
    ctx.set_pixel(pos_a + i, ctx.pal_color_at(pos_a + i));
    if (pos_b + i >= 0 && pos_b + i < SEGLEN)
      ctx.set_pixel(pos_b + i, ctx.pal_color_at(pos_b + i));
  }
}

// 66 — Fairy (palette background + random-timed flashers)
// Source: WLED mode_fairy (FX.cpp line 1470)
// Simplified: flashers tracked per-slot with on/off duration and brightness.
struct FairyFlasher {
  uint16_t state_start;  // low 16 bits of NOW when state began
  uint16_t state_dur;  // duration (units = 10 ms)
  bool state_on;
};

void fx_fairy(EffectContext &ctx) {
  // Deterministic palette background using PRNG re-seeded each frame
  uint16_t prng16 = 5100u;
  for (int32_t i = 0; i < SEGLEN; i++) {
    prng16 = static_cast<uint16_t>(prng16 * 2053u) + 1384u;
    ctx.set_pixel(i, ctx.pal_color(static_cast<uint8_t>(prng16 >> 8u)));
  }

  if (INTENSITY == 0)
    return;

  uint32_t flasher_dist = static_cast<uint32_t>((255u - INTENSITY) / 28u) + 1u;
  uint32_t num_flashers = static_cast<uint32_t>(SEGLEN) / flasher_dist + 1u;
  if (num_flashers > 64u)
    num_flashers = 64u;

  size_t data_size = num_flashers * sizeof(FairyFlasher);
  if (!ctx.env->allocate_data(data_size))
    return;

  auto *flashers = reinterpret_cast<FairyFlasher *>(ctx.env->data);
  uint16_t now16 = static_cast<uint16_t>(NOW & 0xFFFFu);

  prng16 = 5100u;
  for (uint32_t f = 0; f < num_flashers; f++) {
    prng16 = static_cast<uint16_t>(prng16 * 2053u) + 1384u;
    FairyFlasher &fl = flashers[f];

    uint16_t state_time = static_cast<uint16_t>(now16 - fl.state_start);
    if (state_time > static_cast<uint16_t>(fl.state_dur) * 10u) {
      fl.state_on = !fl.state_on;
      if (fl.state_on)
        fl.state_dur = static_cast<uint16_t>(12u + hw_random8(12u + ((255u - SPEED) >> 2u)));
      else
        fl.state_dur = static_cast<uint16_t>(20u + hw_random8(6u + ((255u - SPEED) >> 2u)));
      fl.state_start = now16;
      state_time = 0u;
    }
    if (state_time > 255u)
      state_time = 255u;
    uint8_t bri = fl.state_on ? static_cast<uint8_t>(state_time) : static_cast<uint8_t>(255u - state_time);

    int32_t fp = static_cast<int32_t>(f * flasher_dist);
    if (fp < SEGLEN)
      ctx.set_pixel(fp, color_blend(COLOR1, ctx.pal_color(static_cast<uint8_t>(prng16 >> 8u)), bri));
  }
}

// 67 — Fairytwinkle (all pixels with independent fade-in/out)
// Source: WLED mode_fairytwinkle (FX.cpp line 1547)
struct FairyTwinkleFlasher {
  uint16_t state_start;
  uint16_t state_dur;
  bool state_on;
};

void fx_fairytwinkle(EffectContext &ctx) {
  size_t data_size = static_cast<size_t>(SEGLEN) * sizeof(FairyTwinkleFlasher);
  if (!ctx.env->allocate_data(data_size)) {
    ctx.fill(COLOR0);
    return;
  }
  auto *flashers = reinterpret_cast<FairyTwinkleFlasher *>(ctx.env->data);
  uint16_t now16 = static_cast<uint16_t>(NOW & 0xFFFFu);

  uint32_t rise_fall = 400u + (255u - SPEED) * 3u;
  uint32_t max_dur = rise_fall / 100u + ((255u - INTENSITY) >> 2u) + 13u + ((255u - INTENSITY) >> 1u);
  uint16_t prng16 = 5100u;

  for (int32_t f = 0; f < SEGLEN; f++) {
    FairyTwinkleFlasher &fl = flashers[f];
    uint32_t state_time = static_cast<uint32_t>(static_cast<uint16_t>(now16 - fl.state_start));

    if (state_time > static_cast<uint32_t>(fl.state_dur) * 100u) {
      fl.state_on = !fl.state_on;
      bool init = (fl.state_dur == 0u);
      if (fl.state_on)
        fl.state_dur = static_cast<uint16_t>(rise_fall / 100u + ((255u - INTENSITY) >> 2u) +
                                             hw_random8(12u + ((255u - INTENSITY) >> 1u)) + 1u);
      else
        fl.state_dur = static_cast<uint16_t>(rise_fall / 100u + hw_random8(3u + ((255u - SPEED) >> 6u)) + 1u);
      fl.state_start = now16;
      state_time = 0u;
      if (init) {
        fl.state_start = static_cast<uint16_t>(now16 - static_cast<uint16_t>(rise_fall));
        fl.state_dur = static_cast<uint16_t>(rise_fall / 100u + hw_random8(12u + ((255u - INTENSITY) >> 1u)) + 5u);
        state_time = rise_fall;
      }
    }
    if (fl.state_on && fl.state_dur > static_cast<uint16_t>(max_dur))
      fl.state_dur = static_cast<uint16_t>(max_dur);
    if (state_time > rise_fall)
      state_time = rise_fall;

    uint32_t fade_prog = 255u - ((state_time * 255u) / (rise_fall > 0u ? rise_fall : 1u));
    uint8_t flasher_bri = fl.state_on ? static_cast<uint8_t>(255u - gamma8(static_cast<uint8_t>(fade_prog)))
                                      : gamma8(static_cast<uint8_t>(fade_prog));

    // ensure adjacent pixels differ enough in palette index
    uint16_t last_r = prng16;
    uint16_t diff = 0u;
    while (diff < 0x4000u) {
      prng16 = static_cast<uint16_t>(prng16 * 2053u) + 1384u;
      diff = (prng16 > last_r) ? prng16 - last_r : last_r - prng16;
    }
    ctx.set_pixel(f, color_blend(COLOR1, ctx.pal_color(static_cast<uint8_t>(prng16 >> 8u)), flasher_bri));
  }
}

// 68 — Tri Wipe (3-color sequential wipe)
// Source: WLED mode_tricolor_wipe (FX.cpp line 1697)
void fx_tri_wipe(EffectContext &ctx) {
  uint32_t cycleTime = 1000u + static_cast<uint32_t>(255u - SPEED) * 200u;
  uint32_t perc = NOW % cycleTime;
  uint32_t prog = (perc * 65535u) / cycleTime;
  uint32_t led_index = (prog * static_cast<uint32_t>(SEGLEN) * 3u) >> 16u;
  uint32_t led_offset = led_index;

  // Default fill with palette / color2
  for (int32_t i = 0; i < SEGLEN; i++)
    ctx.set_pixel(i, ctx.params->palette_id != 0u ? ctx.pal_color_at(i) : COLOR2);

  if (led_index < static_cast<uint32_t>(SEGLEN)) {
    // Phase 1: wipe color0 over color1 from left
    for (int32_t i = 0; i < SEGLEN; i++)
      ctx.set_pixel(i, static_cast<uint32_t>(i) > led_offset ? COLOR0 : COLOR1);
  } else if (led_index < static_cast<uint32_t>(SEGLEN) * 2u) {
    // Phase 2: extend color1 coverage
    led_offset = led_index - static_cast<uint32_t>(SEGLEN);
    for (int32_t i = static_cast<int32_t>(led_offset) + 1; i < SEGLEN; i++)
      ctx.set_pixel(i, COLOR1);
  } else {
    // Phase 3: fill color0 from left
    led_offset = led_index - static_cast<uint32_t>(SEGLEN) * 2u;
    for (int32_t i = 0; i <= static_cast<int32_t>(led_offset) && i < SEGLEN; i++)
      ctx.set_pixel(i, COLOR0);
  }
}

// 69 — Tri Fade (smooth 3-color cycle fade)
// Source: WLED mode_tricolor_fade (FX.cpp line 1737)
void fx_tri_fade(EffectContext &ctx) {
  uint16_t counter = static_cast<uint16_t>(NOW * ((SPEED >> 3u) + 1u));
  uint32_t prog = (static_cast<uint32_t>(counter) * 768u) >> 16u;

  uint32_t col1, col2;
  uint8_t stage;
  if (prog < 256u) {
    col1 = COLOR0;
    col2 = COLOR1;
    stage = 0u;
  } else if (prog < 512u) {
    col1 = COLOR1;
    col2 = COLOR2;
    stage = 1u;
  } else {
    col1 = COLOR2;
    col2 = COLOR0;
    stage = 2u;
  }

  uint8_t stp = static_cast<uint8_t>(prog);
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint32_t col;
    if (stage == 2u)
      col = color_blend(ctx.pal_color_at(i), col2, stp);
    else if (stage == 1u)
      col = color_blend(col1, ctx.pal_color_at(i), stp);
    else
      col = color_blend(col1, col2, stp);
    ctx.set_pixel(i, col);
  }
}

// 70 — Multi Comet (up to 8 independent comets)
// Source: WLED mode_multi_comet (FX.cpp line 1778)
void fx_multi_comet(EffectContext &ctx) {
  static constexpr uint8_t MAX_COMETS = 8;
  uint32_t cycleTime = 10u + static_cast<uint32_t>(255u - SPEED);
  uint32_t it = NOW / cycleTime;
  if (it == STEP)
    return;

  if (!ctx.env->allocate_data(sizeof(uint16_t) * MAX_COMETS)) {
    ctx.fill_black();
    return;
  }

  // fade_to_black: INTENSITY/2 + 128 maps to fade amount as scale8 multiplier
  uint8_t fade_amt = static_cast<uint8_t>(INTENSITY / 2u + 128u);
  ctx.fade_to_black(fade_amt);

  auto *comets = reinterpret_cast<uint16_t *>(ctx.env->data);
  for (uint8_t i = 0; i < MAX_COMETS; i++) {
    if (comets[i] < static_cast<uint16_t>(SEGLEN)) {
      uint16_t idx = comets[i];
      uint32_t col = (i & 1u && COLOR2 != 0u) ? COLOR2 : ctx.pal_color_at(static_cast<int32_t>(idx));
      ctx.set_pixel(static_cast<int32_t>(idx), col);
      comets[i]++;
    } else {
      if (!hw_random16(static_cast<uint16_t>(SEGLEN > 0 ? SEGLEN : 1)))
        comets[i] = 0u;
    }
  }
  STEP = it;
}

// 71 — Colortwinkles (pixels fade up/down between palette colours)
// Source: WLED mode_colortwinkle (FX.cpp line 2303)
// Uses 1 bit per pixel to record fade direction; pixel state lives in frame_buf.
void fx_colortwinkles(EffectContext &ctx) {
  size_t data_size = static_cast<size_t>((SEGLEN + 7) >> 3);  // 1 bit per LED
  if (!ctx.env->allocate_data(data_size)) {
    ctx.fill_black();
    return;
  }

  uint8_t *bits = ctx.env->data;
  uint8_t fade_up = 8u + (SPEED >> 2u);
  uint8_t fade_dn = 8u + (SPEED >> 3u);

  for (int32_t i = 0; i < SEGLEN; i++) {
    int32_t abs_px = ctx.map_pixel(i);
    if (abs_px < 0 || abs_px >= static_cast<int32_t>(ctx.frame_len))
      continue;
    uint32_t cur = ctx.frame_buf[abs_px];
    uint32_t idx = static_cast<uint32_t>(i) >> 3u;
    uint8_t bit = static_cast<uint8_t>(i & 0x07u);
    bool fade_up_dir = (bits[idx] >> bit) & 0x01u;

    uint32_t col;
    if (fade_up_dir) {
      col = color_add(cur, color_fade(cur, fade_up, true));
      if (R(col) == 255u || G(col) == 255u || B(col) == 255u)
        bits[idx] &= static_cast<uint8_t>(~(1u << bit));
      if (col == cur)
        col = color_add(col, col);
    } else {
      col = color_fade(cur, static_cast<uint8_t>(255u - fade_dn));
    }
    ctx.set_pixel(i, col);
  }

  // Spawn new pixels
  uint32_t spawn_count = static_cast<uint32_t>(SEGLEN) / 50u + 1u;
  for (uint32_t j = 0; j < spawn_count; j++) {
    if (hw_random8() <= INTENSITY) {
      for (uint8_t t = 0; t < 5u; t++) {
        int32_t pi = static_cast<int32_t>(hw_random16(static_cast<uint16_t>(SEGLEN)));
        int32_t abs_px = ctx.map_pixel(pi);
        if (abs_px >= 0 && abs_px < static_cast<int32_t>(ctx.frame_len) && ctx.frame_buf[abs_px] == 0u) {
          uint32_t idx = static_cast<uint32_t>(pi) >> 3u;
          uint8_t bit = static_cast<uint8_t>(pi & 0x07u);
          bits[idx] |= static_cast<uint8_t>(1u << bit);
          ctx.set_pixel(pi, color_fade(ctx.pal_color(hw_random8()), 64u));
          break;
        }
      }
    }
  }
}

// 72 — Lake (calm, wave-driven palette colours)
// Source: WLED mode_lake (FX.cpp line 2359)
void fx_lake(EffectContext &ctx) {
  uint8_t sp = SPEED / 10u;
  int32_t wave1 = static_cast<int32_t>(beatsin8(static_cast<uint8_t>(sp + 2u), 0u, 127u, NOW)) - 64;
  int32_t wave2 = static_cast<int32_t>(beatsin8(static_cast<uint8_t>(sp + 1u), 0u, 127u, NOW)) - 64;
  int32_t wave3 = static_cast<int32_t>(beatsin8(static_cast<uint8_t>(sp + 2u), 0u, 80u, NOW));

  for (int32_t i = 0; i < SEGLEN; i++) {
    int32_t index = static_cast<int32_t>(cos8(static_cast<uint8_t>((i * 15) + wave1))) / 2 +
                    static_cast<int32_t>(cubicwave8(static_cast<uint8_t>((i * 23) + wave2))) / 2;
    uint8_t lum = (index > wave3) ? static_cast<uint8_t>(index - wave3) : 0u;
    ctx.set_pixel(i, color_fade(ctx.pal_color(static_cast<uint8_t>(index & 0xFFu)), lum));
  }
}

// 73 — Railway (alternating palette pixels cross-fade between even/odd)
// Source: WLED mode_railway (FX.cpp line 2446)
void fx_railway(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  uint32_t dur = static_cast<uint32_t>(256u - SPEED) * 40u;
  uint32_t ramp_dur = (dur * INTENSITY) >> 8u;

  STEP += FRAMETIME_MS;
  if (STEP > dur) {
    STEP = 0u;
    AUX0 ^= 1u;
  }

  uint32_t pos = 255u;
  if (ramp_dur != 0u) {
    uint32_t p0 = (STEP * 255u) / ramp_dur;
    if (p0 < 255u)
      pos = p0;
  }
  if (AUX0)
    pos = 255u - pos;

  for (int32_t i = 0; i < SEGLEN; i += 2) {
    ctx.set_pixel(i, ctx.pal_color(static_cast<uint8_t>(255u - pos)));
    if (i + 1 < SEGLEN)
      ctx.set_pixel(i + 1, ctx.pal_color(static_cast<uint8_t>(pos)));
  }
}

// 74 — Halloween Eyes (two glowing eyes appear, blink, disappear)
// Source: WLED mode_halloween_eyes (FX.cpp line 2715)
struct HalloweenEyeData {
  uint8_t state;  // 0=init_on 1=on 2=blink 3=init_off 4=off
  uint8_t color_idx;
  uint16_t start_pos;
  uint16_t duration;  // ms
  uint32_t start_time;
  uint32_t blink_end;
};

void fx_halloween_eyes(EffectContext &ctx) {
  if (SEGLEN <= 4) {
    ctx.fill(COLOR1);
    return;
  }
  if (!ctx.env->allocate_data(sizeof(HalloweenEyeData))) {
    ctx.fill(COLOR1);
    return;
  }
  auto &d = *reinterpret_cast<HalloweenEyeData *>(ctx.env->data);

  uint32_t eye_space = static_cast<uint32_t>(SEGLEN) >> 5u;
  if (eye_space < 2u)
    eye_space = 2u;
  uint32_t eye_w = eye_space / 2u;
  if (eye_w < 1u)
    eye_w = 1u;
  uint32_t eye_len = 2u * eye_w + eye_space;

  if (eye_len >= static_cast<uint32_t>(SEGLEN)) {
    ctx.fill(COLOR1);
    return;
  }

  ctx.fill(COLOR1);

  if (d.state >= 5u)
    d.state = 0u;

  uint32_t elapsed = NOW - d.start_time;
  uint32_t duration = d.duration > 0u ? d.duration : 1u;

  switch (d.state) {
    case 0:  // init_on
      d.start_pos = hw_random16(static_cast<uint16_t>(SEGLEN - eye_len - 1u));
      d.color_idx = hw_random8();
      d.duration = static_cast<uint16_t>(128u + hw_random16(static_cast<uint16_t>(INTENSITY) * 64u));
      duration = d.duration;
      d.state = 1u;
      d.start_time = NOW;
      elapsed = 0u;
      [[fallthrough]];
    case 1: {  // on
      uint32_t fade_in = elapsed * 2048u / duration;
      uint32_t eye_col = ctx.pal_color(d.color_idx);
      uint32_t c = (fade_in < 256u) ? color_blend(COLOR1, eye_col, static_cast<uint8_t>(fade_in)) : eye_col;
      for (uint32_t i = 0u; i < eye_w; i++) {
        ctx.set_pixel(static_cast<int32_t>(d.start_pos + i), c);
        ctx.set_pixel(static_cast<int32_t>(d.start_pos + eye_w + eye_space + i), c);
      }
      if (elapsed > 1024u && hw_random8() < 4u) {
        d.state = 2u;
        d.blink_end = NOW + hw_random8(8u, 128u);
      }
      break;
    }
    case 2:  // blink
      if (NOW >= d.blink_end)
        d.state = 1u;
      break;
    case 3:  // init_off
    {
      uint32_t off_base = static_cast<uint32_t>(SPEED) * 128u;
      d.duration = static_cast<uint16_t>(off_base + hw_random16(static_cast<uint16_t>(off_base)));
      d.state = 4u;
      d.start_time = NOW;
      break;
    }
    default:  // off
      break;
  }

  if (elapsed > duration) {
    d.start_time = NOW;
    if (d.state == 1u || d.state == 2u)
      d.state = 3u;
    else
      d.state = 0u;
  }
}

// 75 — Plasma (phase-shifted cubicwave/cos8 plasma, by Andrew Tuline)
// Source: WLED mode_plasma (FX.cpp line 4041)
void fx_plasma(EffectContext &ctx) {
  if (CALL == 0)
    AUX0 = hw_random8(0u, 2u);

  uint8_t this_phase = beatsin8(static_cast<uint8_t>(6u + AUX0), 0u, 127u, NOW) - 64u;
  uint8_t that_phase = beatsin8(static_cast<uint8_t>(7u + AUX0), 0u, 127u, NOW) - 64u;

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t speed_factor = static_cast<uint8_t>(2u + 3u * (SPEED >> 5u));
    uint8_t color_index =
        cubicwave8(static_cast<uint8_t>((static_cast<uint32_t>(i) * speed_factor + this_phase) & 0xFFu)) / 2u +
        cos8(static_cast<uint8_t>((static_cast<uint32_t>(i) * (speed_factor - 1u) + that_phase) & 0xFFu)) / 2u;
    uint8_t this_bright = qsub8(color_index, beatsin8(7u, 0u, static_cast<uint8_t>(128u - (INTENSITY >> 1u)), NOW));
    ctx.set_pixel(i, color_fade(ctx.pal_color(color_index), this_bright));
  }
}

// 76 — Sunrise (sun rising/setting across the strip)
// Source: WLED mode_sunrise (FX.cpp line 4260)
void fx_sunrise(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  // On speed change or first call, re-record start time
  if (CALL == 0 || static_cast<uint8_t>(AUX0) != SPEED) {
    STEP = NOW;
    AUX0 = SPEED;
  }

  ctx.fill_black();
  uint32_t stage = 0xFFFFu;

  uint32_t s10_since = (NOW - STEP) / 100u;  // tenths of seconds

  if (SPEED > 120u) {
    // Fast breathing sunrise/sunset
    uint32_t counter = (NOW >> 1u) * static_cast<uint32_t>((SPEED - 120u) >> 1u + 1u);
    stage = static_cast<uint32_t>(triwave16(static_cast<uint16_t>(counter & 0xFFFFu)));
  } else if (SPEED > 0u) {
    uint32_t dur_mins = SPEED > 60u ? SPEED - 60u : SPEED;
    uint32_t s10_target = dur_mins * 600u;
    if (s10_since > s10_target)
      s10_since = s10_target;
    stage = (s10_target > 0u) ? (s10_since * 0xFFFFu) / s10_target : 0xFFFFu;
    if (SPEED > 60u)
      stage = 0xFFFFu - stage;  // sunset
  }

  for (int32_t i = 0; i <= SEGLEN / 2; i++) {
    uint32_t wave = triwave16(
        static_cast<uint16_t>((static_cast<uint32_t>(i) * stage) / static_cast<uint32_t>(SEGLEN > 0 ? SEGLEN : 1)));
    uint32_t w8 = (wave >> 8u) + static_cast<uint32_t>((wave * INTENSITY) >> 15u);
    uint32_t col;
    if (w8 > 240u)
      col = ctx.pal_color(240u);
    else
      col = ctx.pal_color(static_cast<uint8_t>(w8));
    ctx.set_pixel(i, col);
    ctx.set_pixel(SEGLEN - i - 1, col);
  }
}

// 77 — Washing Machine (sine wave pattern reversing direction periodically)
// Source: WLED mode_washing_machine (FX.cpp line 4626)
// tristate_square8 approximated: returns +val, 0, or -val in three phases.
static int8_t tristate_square8_approx(uint32_t now_ms, uint8_t lim, uint8_t ontime) {
  uint32_t period = 256u;
  uint32_t phase = now_ms % period;
  if (phase < ontime)
    return static_cast<int8_t>(lim);
  if (phase < static_cast<uint32_t>(ontime) + 20u)
    return 0;
  return -static_cast<int8_t>(lim);
}

void fx_washing_machine(EffectContext &ctx) {
  int8_t speed_dir = tristate_square8_approx(NOW >> 7u, 90u, 15u);

  STEP += static_cast<uint32_t>(static_cast<int32_t>(speed_dir) * 2048 /
                                static_cast<int32_t>(512u - SPEED > 1u ? 512u - SPEED : 1u));

  uint32_t zones = static_cast<uint32_t>(INTENSITY / 25u) + 1u;
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t col = sin8(static_cast<uint8_t>((static_cast<uint32_t>(zones) * 255u * static_cast<uint32_t>(i) /
                                             static_cast<uint32_t>(SEGLEN > 0 ? SEGLEN : 1)) +
                                            (STEP >> 7u)));
    ctx.set_pixel(i, ctx.pal_color(col));
  }
}

// 78 — Blends (cross-blending scrolling palette colours)
// Source: WLED mode_blends (FX.cpp line 4660)
void fx_blends(EffectContext &ctx) {
  // Cap pixel buffer at 255 entries to match WLED's uint8-indexed approach
  uint32_t pixel_len = (SEGLEN > 255) ? 255u : static_cast<uint32_t>(SEGLEN);
  size_t data_size = (pixel_len + 1u) * sizeof(uint32_t);
  if (!ctx.env->allocate_data(data_size)) {
    ctx.fill(ctx.pal_color(0u));
    return;
  }
  auto *pixels = reinterpret_cast<uint32_t *>(ctx.env->data);

  uint8_t blend_speed = static_cast<uint8_t>(10u + (static_cast<uint32_t>(INTENSITY) * 118u) / 255u);
  uint32_t shift = (NOW * (static_cast<uint32_t>(SPEED >> 3u) + 1u)) >> 8u;

  for (uint32_t i = 0; i < pixel_len; i++) {
    // quadwave8: triangle wave of 0..255 repeated twice per period
    uint8_t qw = triwave8(static_cast<uint8_t>((i + 1u) * 16u));
    pixels[i] = color_blend(pixels[i], ctx.pal_color(static_cast<uint8_t>(shift + qw)), blend_speed);
    shift += 3u;
  }

  uint32_t offset = 0u;
  for (int32_t i = 0; i < SEGLEN; i++) {
    ctx.set_pixel(i, pixels[offset++]);
    if (offset >= pixel_len)
      offset = 0u;
  }
}

// 79 — Strobe Mega (multi-strobe: bursts of strobe flashes separated by pauses)
// Source: WLED mode_multi_strobe (FX.cpp line 826)
void fx_strobe_mega(EffectContext &ctx) {
  uint32_t count = 2u * (static_cast<uint32_t>(INTENSITY / 10u) + 1u);
  uint32_t delay_ms;

  if (AUX1 < count) {
    if ((AUX1 & 1u) == 0u) {
      ctx.fill(COLOR0);
      delay_ms = 15u;
    } else {
      ctx.fill(ctx.pal_color_at(0u));
      delay_ms = 50u;
    }
  } else {
    ctx.fill(ctx.pal_color_at(0u));
    delay_ms = 50u + 20u * static_cast<uint32_t>(255u - SPEED);
  }

  if (NOW - STEP > delay_ms) {
    AUX1++;
    if (AUX1 > count)
      AUX1 = 0u;
    STEP = NOW;
  }
}

// 80 — Chase (bicolor chase: palette dot on COLOR1 background)
// Source: WLED mode_chase_color / chase() helper (FX.cpp lines 895, 963)
static void do_chase(EffectContext &ctx, bool random_color) {
  uint16_t counter = static_cast<uint16_t>(NOW * ((SPEED >> 2u) + 1u));
  uint32_t a = (static_cast<uint32_t>(counter) * static_cast<uint32_t>(SEGLEN)) >> 16u;

  // On wrap-around, pick a new random hue
  if (random_color && a < static_cast<uint32_t>(AUX0)) {
    AUX1 = AUX0;  // save previous hue
    AUX0 = static_cast<uint16_t>(hw_random8());
  }
  AUX0 = static_cast<uint16_t>(a);

  uint32_t size = 1u + (static_cast<uint32_t>(INTENSITY) * static_cast<uint32_t>(SEGLEN)) / 1024u;
  if (size < 1u)
    size = 1u;
  uint32_t b = (a + size) % static_cast<uint32_t>(SEGLEN);
  uint32_t c = (b + size) % static_cast<uint32_t>(SEGLEN);

  uint32_t col1 = random_color ? ctx.wheel(static_cast<uint8_t>(AUX1)) : COLOR1;
  uint32_t col2 = random_color ? ctx.wheel(static_cast<uint8_t>(AUX0)) : (COLOR2 ? COLOR2 : COLOR0);
  uint32_t col3 = COLOR0;

  // Background
  if (!random_color) {
    for (int32_t i = 0; i < SEGLEN; i++)
      ctx.set_pixel(i, ctx.pal_color_at(i));
  } else {
    ctx.fill(col1);
    // fill old background from a to end
    for (uint32_t i = a; i < static_cast<uint32_t>(SEGLEN); i++)
      ctx.set_pixel(static_cast<int32_t>(i), col1);
  }

  // Segment a..b (color2)
  if (a < b) {
    for (uint32_t i = a; i < b; i++)
      ctx.set_pixel(static_cast<int32_t>(i), col2);
  } else {
    for (uint32_t i = a; i < static_cast<uint32_t>(SEGLEN); i++)
      ctx.set_pixel(static_cast<int32_t>(i), col2);
    for (uint32_t i = 0u; i < b; i++)
      ctx.set_pixel(static_cast<int32_t>(i), col2);
  }

  // Segment b..c (color3)
  if (b < c) {
    for (uint32_t i = b; i < c; i++)
      ctx.set_pixel(static_cast<int32_t>(i), col3);
  } else {
    for (uint32_t i = b; i < static_cast<uint32_t>(SEGLEN); i++)
      ctx.set_pixel(static_cast<int32_t>(i), col3);
    for (uint32_t i = 0u; i < c; i++)
      ctx.set_pixel(static_cast<int32_t>(i), col3);
  }
}

void fx_chase_color(EffectContext &ctx) {
  do_chase(ctx, false);
}

// 81 — Chase Random (same chase but with randomised colours on each revolution)
// Source: WLED mode_chase_random (FX.cpp line 972)
void fx_chase_color_random(EffectContext &ctx) {
  do_chase(ctx, true);
}

// 82 — Solid Pattern Tri (three solid colour bands repeating)
// Source: WLED mode_tri_static_pattern (FX.cpp line 2890)
void fx_solid_pattern_tri(EffectContext &ctx) {
  uint32_t seg_size = static_cast<uint32_t>(INTENSITY >> 5u) + 1u;
  uint32_t curr_seg = 0u;
  uint32_t curr_count = 0u;
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint32_t which = curr_seg % 3u;
    ctx.set_pixel(i, which == 0u ? COLOR0 : which == 1u ? COLOR1 : COLOR2);
    if (++curr_count >= seg_size) {
      curr_seg++;
      curr_count = 0u;
    }
  }
}

// 83 — Solid Pattern (alternating lit/unlit bands)
// Source: WLED mode_static_pattern (FX.cpp line 2872)
void fx_solid_pattern(EffectContext &ctx) {
  uint32_t lit = 1u + SPEED;
  uint32_t unlit = 1u + INTENSITY;
  bool drawing_lit = true;
  uint32_t cnt = 0u;
  for (int32_t i = 0; i < SEGLEN; i++) {
    ctx.set_pixel(i, drawing_lit ? ctx.pal_color_at(i) : COLOR1);
    if (++cnt >= (drawing_lit ? lit : unlit)) {
      cnt = 0u;
      drawing_lit = !drawing_lit;
    }
  }
}

// ============================================================
// Batch 5 — effects 84-101
// ============================================================

// 84 — Blink Rainbow (rainbow-colored blink, cycling hue per call)
// Source: WLED mode_blink_rainbow (FX.cpp line 233)
void fx_blink_rainbow(EffectContext &ctx) {
  uint32_t cycleTime = (1000u * (uint32_t(9) - (SPEED >> 5))) / 10u + 1u;
  uint32_t onTime = (cycleTime * INTENSITY) >> 8;
  bool on = (NOW % cycleTime) < onTime;
  ctx.fill(on ? ctx.wheel(static_cast<uint8_t>(CALL & 0xFFu)) : COLOR1);
}

// 85 — Dynamic Smooth (smooth version of Dynamic: new pixels blend into bg)
// Source: WLED mode_dynamic_smooth (FX.cpp line 420) — mode_dynamic with check1 forced
void fx_dynamic_smooth(EffectContext &ctx) {
  // Same as Dynamic but always with smooth-blend-over-bg behaviour (check1 forced true):
  // each tick, blend a random pixel to a new wheel colour instead of snapping.
  uint32_t interval = 50u + (255u - SPEED) * 4u;
  if (STEP == 0 || NOW - STEP > interval) {
    // clear bg first (smooth: leave old pixels — only update random subset)
    int32_t n = 1 + (INTENSITY >> 4);
    for (int32_t i = 0; i < n; i++) {
      int32_t pos = hw_random16(SEGLEN);
      uint32_t old_col;
      int32_t abs_px = ctx.map_pixel(pos);
      old_col = (abs_px >= 0 && abs_px < static_cast<int32_t>(ctx.frame_len)) ? ctx.frame_buf[abs_px] : 0u;
      uint32_t new_col = ctx.wheel(hw_random8());
      ctx.set_pixel(pos, color_blend(old_col, new_col, 128u));
    }
    STEP = NOW;
  }
}

// 86 — Running Dual (sine wave running from both ends toward centre)
// Source: WLED mode_running_dual / running_base(false, true) (FX.cpp line 627)
void fx_running_dual(EffectContext &ctx) {
  if (SEGLEN <= 0)
    return;
  uint32_t x_scale = 256u / static_cast<uint32_t>(SEGLEN > 1 ? SEGLEN : 1);
  uint32_t counter = (NOW * SPEED) >> 9;
  for (int32_t i = 0; i < SEGLEN; i++) {
    // Forward wave (colour 0 / left)
    uint8_t a_fwd = static_cast<uint8_t>((static_cast<uint32_t>(i) * x_scale) - counter);
    uint8_t s_fwd = sin8(a_fwd);
    // Reverse wave (colour 2 / right) mirrored
    uint8_t a_rev = static_cast<uint8_t>((static_cast<uint32_t>(SEGLEN - 1 - i) * x_scale) - counter);
    uint8_t s_rev = sin8(a_rev);
    uint32_t col_fwd = color_blend(COLOR1, ctx.pal_color_at(i), s_fwd);
    uint32_t col_rev = color_blend(COLOR1, (COLOR2 ? COLOR2 : ctx.pal_color_at(i)), s_rev);
    ctx.set_pixel(i, color_blend(col_fwd, col_rev, 128u));
  }
}

// 87 — Traffic Light (R/Y/G cycling per group of 3, optional US style)
// Source: WLED mode_traffic_light (FX.cpp line 1052)
void fx_traffic_light(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR1);
    return;
  }
  // Palette background
  for (int32_t i = 0; i < SEGLEN; i++)
    ctx.set_pixel(i, ctx.pal_color_at(i));

  // States: 0=red, 1=red+amber(not US), 2=green, 3=amber
  uint32_t mdelay = 500u;
  switch (AUX0) {
    case 0:
      mdelay = 150u + 100u * static_cast<uint32_t>(255u - SPEED);
      break;
    case 1:
      mdelay = 150u + 20u * static_cast<uint32_t>(255u - SPEED);
      break;
    case 2:
      mdelay = 150u + 100u * static_cast<uint32_t>(255u - SPEED);
      break;
    default:
      mdelay = 150u + 20u * static_cast<uint32_t>(255u - SPEED);
      break;
  }

  for (int32_t i = 0; i < SEGLEN - 2; i += 3) {
    switch (AUX0) {
      case 0:
        ctx.set_pixel(i, 0xFF0000u);
        break;
      case 1:
        ctx.set_pixel(i, 0xFF0000u);
        ctx.set_pixel(i + 1, 0xEECC00u);
        break;
      case 2:
        ctx.set_pixel(i + 2, 0x00FF00u);
        break;
      default:
        ctx.set_pixel(i + 1, 0xEECC00u);
        break;
    }
  }

  if (NOW - STEP > mdelay) {
    AUX0++;
    // Skip Red+Amber phase in US style (INTENSITY > 140)
    if (AUX0 == 1u && INTENSITY > 140u)
      AUX0 = 2u;
    if (AUX0 > 3u)
      AUX0 = 0u;
    STEP = NOW;
  }
}

// 88 — Loading (forward-sweeping gradient bar)
// Source: WLED mode_loading / gradient_base(true) (FX.cpp line 1429)
void fx_loading(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  uint32_t counter = (NOW * (static_cast<uint32_t>(SPEED >> 3u) + 1u)) >> 5u;
  int32_t prog = static_cast<int32_t>(counter % static_cast<uint32_t>(SEGLEN * 2));
  bool forward = (prog < SEGLEN);
  int32_t pos = forward ? prog : (SEGLEN * 2 - prog - 1);

  for (int32_t i = 0; i < SEGLEN; i++) {
    if (forward) {
      // Fade: leading edge bright, behind it dimmed
      uint8_t blend = (i <= pos) ? static_cast<uint8_t>(255u - (static_cast<uint32_t>(pos - i) * INTENSITY /
                                                                static_cast<uint32_t>(SEGLEN > 0 ? SEGLEN : 1)))
                                 : 0u;
      ctx.set_pixel(i, color_blend(COLOR1, ctx.pal_color_at(i), blend));
    } else {
      uint8_t blend = (i >= pos) ? static_cast<uint8_t>(255u - (static_cast<uint32_t>(i - pos) * INTENSITY /
                                                                static_cast<uint32_t>(SEGLEN > 0 ? SEGLEN : 1)))
                                 : 0u;
      ctx.set_pixel(i, color_blend(COLOR1, ctx.pal_color_at(i), blend));
    }
  }
}

// 89 — Stream 2 (random colour walk: each pixel derives from neighbour, reseeds on cycle)
// Source: WLED mode_random_chase (FX.cpp line 1815)
void fx_stream2(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  // Packed data: [4] seed-colour uint32, [2] prng seed uint16, [2] last_it uint16
  static constexpr size_t DATA_SIZE = sizeof(uint32_t) + sizeof(uint16_t) * 2;
  if (!ctx.env->allocate_data(DATA_SIZE)) {
    ctx.fill_black();
    return;
  }
  auto *col_seed = reinterpret_cast<uint32_t *>(ctx.env->data);
  auto *prng_seed = reinterpret_cast<uint16_t *>(ctx.env->data + sizeof(uint32_t));
  auto *last_it = reinterpret_cast<uint16_t *>(ctx.env->data + sizeof(uint32_t) + sizeof(uint16_t));

  if (CALL == 0) {
    *col_seed = ctx.wheel(hw_random8());
    *prng_seed = hw_random16();
    *last_it = 0u;
  }

  uint32_t cycleTime = 25u + 3u * static_cast<uint32_t>(255u - SPEED);
  uint32_t it = NOW / cycleTime;

  // Save/restore PRNG state so we generate deterministic same sequence each render
  uint16_t saved_seed = g_prng.get_seed();
  g_prng.set_seed(*prng_seed);

  uint32_t color = *col_seed;
  for (int32_t i = SEGLEN - 1; i >= 0; i--) {
    uint8_t r = (hw_random8(6u) != 0u) ? static_cast<uint8_t>(color >> 16u) : hw_random8();
    uint8_t g_c = (hw_random8(6u) != 0u) ? static_cast<uint8_t>(color >> 8u) : hw_random8();
    uint8_t b = (hw_random8(6u) != 0u) ? static_cast<uint8_t>(color) : hw_random8();
    color = RGBW32(r, g_c, b, 0u);
    ctx.set_pixel(i, color);
    if (i == SEGLEN - 1 && static_cast<uint16_t>(it & 0xFFFFu) != *last_it) {
      // New frame: commit leading colour and prng state
      *col_seed = color;
      *prng_seed = g_prng.get_seed();
    }
  }

  *last_it = static_cast<uint16_t>(it & 0xFFFFu);
  g_prng.set_seed(saved_seed);
}

// 90 — Spots Fade (animated triwave-brightness spots)
// Source: WLED mode_spots_fade / spots_base(tr) (FX.cpp line 2949)
void fx_spots_fade(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  ctx.fill(COLOR1);

  // Triwave-modulated threshold — same formula as WLED spots_base(tr)
  uint32_t raw_counter = NOW * (static_cast<uint32_t>(SPEED >> 2u) + 8u);
  uint32_t t = triwave16(static_cast<uint16_t>(raw_counter & 0xFFFFu));
  uint32_t tr = (t >> 1u) + (t >> 2u);

  uint32_t threshold = tr;
  uint32_t max_zones = static_cast<uint32_t>(SEGLEN) >> 2u;
  uint32_t zones = 1u + ((static_cast<uint32_t>(INTENSITY) * max_zones) >> 8u);
  uint32_t zone_len = static_cast<uint32_t>(SEGLEN) / zones;
  uint32_t offset = (static_cast<uint32_t>(SEGLEN) - zones * zone_len) >> 1u;

  for (uint32_t z = 0u; z < zones; z++) {
    uint32_t pos = offset + z * zone_len;
    for (uint32_t i = 0u; i < zone_len; i++) {
      uint32_t wave = (i < zone_len / 2u) ? (i * 0xFFFFu) / zone_len : ((zone_len - 1u - i) * 0xFFFFu) / zone_len;
      wave = wave * 2u;
      if (wave > 0xFFFFu)
        wave = 0xFFFFu;
      if (wave > threshold) {
        uint32_t s = (wave - threshold) * 255u / (0xFFFFu - threshold + 1u);
        int32_t idx = static_cast<int32_t>(pos + i);
        if (idx < SEGLEN)
          ctx.set_pixel(idx, color_blend(ctx.pal_color_at(idx), COLOR1, static_cast<uint8_t>(255u - s)));
      }
    }
  }
}

// 91 — Sinelon Dual (two sinelon dots at opposite ends of strip)
// Source: WLED mode_sinelon_dual / sinelon_base(true) (FX.cpp line 3376)
void fx_sinelon_dual(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  ctx.fade_to_black(INTENSITY > 0u ? INTENSITY : 1u);
  uint16_t bpm = SPEED / 10u;
  if (bpm < 1u)
    bpm = 1u;
  int32_t pos = static_cast<int32_t>(beatsin16(bpm, 0u, static_cast<uint16_t>(SEGLEN - 1), NOW));
  if (CALL == 0)
    AUX0 = static_cast<uint16_t>(pos);

  uint32_t col1 = ctx.pal_color_at(pos);
  uint32_t col2 = COLOR2 ? COLOR2 : ctx.pal_color_at(SEGLEN - 1 - pos);

  // Fill gap from last position
  if (static_cast<int32_t>(AUX0) < pos) {
    for (int32_t i = static_cast<int32_t>(AUX0); i <= pos; i++) {
      ctx.set_pixel(i, col1);
      ctx.set_pixel(SEGLEN - 1 - i, col2);
    }
  } else if (static_cast<int32_t>(AUX0) > pos) {
    for (int32_t i = pos; i <= static_cast<int32_t>(AUX0); i++) {
      ctx.set_pixel(i, col1);
      ctx.set_pixel(SEGLEN - 1 - i, col2);
    }
  } else {
    ctx.set_pixel(pos, col1);
    ctx.set_pixel(SEGLEN - 1 - pos, col2);
  }
  AUX0 = static_cast<uint16_t>(pos);
}

// 92 — Sinelon Rainbow (sinelon with wheel colour derived from position)
// Source: WLED mode_sinelon_rainbow / sinelon_base(false, true) (FX.cpp line 3382)
void fx_sinelon_rainbow(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  ctx.fade_to_black(INTENSITY > 0u ? INTENSITY : 1u);
  uint16_t bpm = SPEED / 10u;
  if (bpm < 1u)
    bpm = 1u;
  int32_t pos = static_cast<int32_t>(beatsin16(bpm, 0u, static_cast<uint16_t>(SEGLEN - 1), NOW));
  if (CALL == 0)
    AUX0 = static_cast<uint16_t>(pos);

  uint32_t col = ctx.wheel(static_cast<uint8_t>((static_cast<uint32_t>(pos) & 0x07u) * 32u));

  if (static_cast<int32_t>(AUX0) < pos) {
    for (int32_t i = static_cast<int32_t>(AUX0); i <= pos; i++)
      ctx.set_pixel(i, col);
  } else if (static_cast<int32_t>(AUX0) > pos) {
    for (int32_t i = pos; i <= static_cast<int32_t>(AUX0); i++)
      ctx.set_pixel(i, col);
  } else {
    ctx.set_pixel(pos, col);
  }
  AUX0 = static_cast<uint16_t>(pos);
}

// 93 — Ripple Rainbow (dimmed cycling rainbow background + ripple propagation)
// Source: WLED mode_ripple_rainbow (FX.cpp line 2557)
void fx_ripple_rainbow(EffectContext &ctx) {
  static constexpr int32_t MAX_RIPPLES = 5;
  static constexpr size_t DATA_SIZE = sizeof(RippleState) * MAX_RIPPLES;
  if (!ctx.env->allocate_data(DATA_SIZE)) {
    ctx.fill_black();
    return;
  }

  auto *ripples = reinterpret_cast<RippleState *>(ctx.env->data);

  if (CALL == 0) {
    for (int32_t i = 0; i < MAX_RIPPLES; i++)
      ripples[i].age = -1;
    AUX0 = hw_random8();
    AUX1 = hw_random8();
  }

  // Slowly cycle background hue
  if (AUX0 == AUX1) {
    AUX1 = hw_random8();
  } else if (static_cast<uint8_t>(AUX1) > static_cast<uint8_t>(AUX0)) {
    AUX0 = static_cast<uint16_t>((AUX0 + 1u) & 0xFFu);
  } else {
    AUX0 = static_cast<uint16_t>((AUX0 - 1u) & 0xFFu);
  }

  // Dimmed rainbow background (blend with black at ~92%)
  uint32_t bg = color_blend(ctx.wheel(static_cast<uint8_t>(AUX0)), 0u, 235u);
  ctx.fill(bg);

  // Spawn new ripple occasionally
  uint32_t interval = 100u + (255u - SPEED) * 10u;
  if (NOW - STEP > interval) {
    STEP = NOW;
    for (int32_t i = 0; i < MAX_RIPPLES; i++) {
      if (ripples[i].age < 0) {
        ripples[i].center = hw_random8() % SEGLEN;
        ripples[i].age = 0;
        ripples[i].color_idx = hw_random8();
        break;
      }
    }
  }

  int32_t max_radius = SEGLEN / 2 + 1;
  for (int32_t i = 0; i < MAX_RIPPLES; i++) {
    RippleState &r = ripples[i];
    if (r.age < 0)
      continue;
    int32_t radius = r.age;
    uint8_t fade = static_cast<uint8_t>(255u - static_cast<uint32_t>(r.age) * 255u / static_cast<uint32_t>(max_radius));
    uint32_t col = color_fade(ctx.pal_color(r.color_idx), fade);
    ctx.set_pixel(r.center, col);
    for (int32_t d = 1; d <= radius; d++) {
      if (r.center - d >= 0)
        ctx.set_pixel(r.center - d, col);
      if (r.center + d < SEGLEN)
        ctx.set_pixel(r.center + d, col);
    }
    r.age++;
    if (r.age >= max_radius)
      r.age = -1;
  }
}

// 94 — Candle Multi (per-pixel independent candle flicker)
// Source: WLED mode_candle_multi / candle(true) (FX.cpp line 3500)
// Data layout: [4 bytes last_call_ms] + [SEGLEN*3 bytes: brightness/target/step per pixel]
void fx_candle_multi(EffectContext &ctx) {
  size_t data_size = sizeof(uint32_t) + static_cast<size_t>(SEGLEN > 1 ? SEGLEN - 1 : 0) * 3u;
  if (!ctx.env->allocate_data(data_size)) {
    ctx.fill(COLOR0);
    return;
  }
  auto *last_call = reinterpret_cast<uint32_t *>(ctx.env->data);
  uint8_t *candle_data = ctx.env->data + sizeof(uint32_t);

  // Limit update rate to ~42 fps
  if (NOW - *last_call < FRAMETIME_MS)
    ;  // still render with current state
  else
    *last_call = NOW;

  uint8_t val_range = INTENSITY;
  uint8_t rnd_val = val_range >> 1u;

  uint8_t speed_factor = 4u;
  if (SPEED > 252u)
    speed_factor = 1u;
  else if (SPEED > 99u)
    speed_factor = 2u;
  else if (SPEED > 49u)
    speed_factor = 3u;

  // Pixel 0 uses AUX0 (s), AUX1 (s_target), STEP (fadeStep)
  {
    uint16_t s = AUX0;
    uint16_t s_target = AUX1;
    uint32_t fade_step = STEP;

    if (fade_step == 0u) {
      s = 128u;
      s_target = 130u + hw_random8(4u);
      fade_step = 1u;
    }

    bool new_target = false;
    if (s_target > s) {
      s = static_cast<uint16_t>(qadd8(static_cast<uint8_t>(s), static_cast<uint8_t>(fade_step)));
      if (s >= s_target)
        new_target = true;
    } else {
      s = static_cast<uint16_t>(qsub8(static_cast<uint8_t>(s), static_cast<uint8_t>(fade_step)));
      if (s <= s_target)
        new_target = true;
    }

    if (new_target) {
      s_target = static_cast<uint16_t>(hw_random8(rnd_val) + hw_random8(rnd_val));
      if (s_target < (rnd_val >> 1u))
        s_target = static_cast<uint16_t>((rnd_val >> 1u) + hw_random8(rnd_val));
      s_target += static_cast<uint16_t>(255u - val_range);
      uint16_t dif = (s_target > s) ? s_target - s : s - s_target;
      fade_step = dif >> speed_factor;
      if (fade_step == 0u)
        fade_step = 1u;
    }

    // Pixel 0 controls all pixels in single-candle mode — here pixel 0 only
    ctx.set_pixel(0, color_blend(COLOR1, ctx.pal_color_at(0), static_cast<uint8_t>(s)));
    AUX0 = s;
    AUX1 = s_target;
    STEP = fade_step;
  }

  // Remaining pixels use candle_data[]
  int32_t num = SEGLEN > 1 ? SEGLEN - 1 : 0;
  for (int32_t i = 0; i < num; i++) {
    size_t d = static_cast<size_t>(i) * 3u;
    uint8_t s = candle_data[d];
    uint8_t s_target = candle_data[d + 1u];
    uint8_t fade_step = candle_data[d + 2u];

    if (fade_step == 0u) {
      s = 128u;
      s_target = 130u + hw_random8(4u);
      fade_step = 1u;
    }

    bool new_target = false;
    if (s_target > s) {
      s = qadd8(s, fade_step);
      if (s >= s_target)
        new_target = true;
    } else {
      s = qsub8(s, fade_step);
      if (s <= s_target)
        new_target = true;
    }

    if (new_target) {
      s_target = hw_random8(rnd_val) + hw_random8(rnd_val);
      if (s_target < (rnd_val >> 1u))
        s_target = (rnd_val >> 1u) + hw_random8(rnd_val);
      s_target += 255u - val_range;
      uint8_t dif = (s_target > s) ? s_target - s : s - s_target;
      fade_step = dif >> speed_factor;
      if (fade_step == 0u)
        fade_step = 1u;
    }

    ctx.set_pixel(i + 1, color_blend(COLOR1, ctx.pal_color_at(i + 1), s));
    candle_data[d] = s;
    candle_data[d + 1u] = s_target;
    candle_data[d + 2u] = fade_step;
  }
}

// 95 — Noise Pal (slow noise with randomised evolving palette, by Andrew Tuline)
// Source: WLED mode_noisepal (FX.cpp line 4362)
// Simplified: two 4-stop palettes blended slowly; uses inoise8 for pixel colouring.
void fx_noise_pal(EffectContext &ctx) {
  // Store two 16-entry palettes as 16*4=64 bytes each (using pal_color approximation)
  // We store 4 colour stops (each 4 bytes) for target palette + 4 for current palette.
  static constexpr size_t NUM_STOPS = 4u;
  static constexpr size_t DATA_SIZE = NUM_STOPS * 2u * sizeof(uint32_t);
  if (!ctx.env->allocate_data(DATA_SIZE)) {
    ctx.fill(ctx.pal_color(0u));
    return;
  }
  auto *pal_cur = reinterpret_cast<uint32_t *>(ctx.env->data);
  auto *pal_tgt = pal_cur + NUM_STOPS;

  uint32_t change_ms = 4000u + static_cast<uint32_t>(SPEED) * 10u;
  if (NOW - STEP > change_ms) {
    STEP = NOW;
    uint8_t base_h = hw_random8();
    pal_tgt[0] = ctx.wheel(static_cast<uint8_t>(base_h + hw_random8(64u)));
    pal_tgt[1] = ctx.wheel(static_cast<uint8_t>(base_h + 128u));
    pal_tgt[2] = ctx.wheel(static_cast<uint8_t>(base_h + hw_random8(92u)));
    pal_tgt[3] = ctx.wheel(static_cast<uint8_t>(base_h + hw_random8(92u)));
  }

  // Blend current palette toward target (48/256 per frame)
  for (uint32_t s = 0u; s < NUM_STOPS; s++)
    pal_cur[s] = color_blend(pal_cur[s], pal_tgt[s], 48u);

  uint32_t scale = 15u + (static_cast<uint32_t>(INTENSITY) >> 2u);
  AUX0 = static_cast<uint16_t>(AUX0 + beatsin8(10u, 1u, 4u, NOW));

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t index = inoise8(static_cast<uint16_t>(static_cast<uint32_t>(i) * scale),
                            static_cast<uint16_t>(AUX0 + static_cast<uint32_t>(i) * scale));
    // Look up colour from our 4-stop mini-palette (interpolate between stops)
    uint32_t stop_f = static_cast<uint32_t>(index) * (NUM_STOPS - 1u);
    uint32_t stop_lo = stop_f >> 8u;
    uint8_t frac = static_cast<uint8_t>(stop_f & 0xFFu);
    if (stop_lo >= NUM_STOPS - 1u)
      stop_lo = NUM_STOPS - 2u;
    uint32_t col = color_blend(pal_cur[stop_lo], pal_cur[stop_lo + 1u], frac);
    // If a real palette is selected, use it instead
    if (ctx.params->palette_id != 0u)
      col = ctx.pal_color(index);
    ctx.set_pixel(i, col);
  }
}

// 96 — Twinklefox (holiday twinkle with palette colours and warm-dim incandescent fade)
// Source: WLED twinklefox_base(false) (FX.cpp line 2642)
void fx_twinklefox(EffectContext &ctx) {
  // aux0 stores the tick divisor (speed-dependent)
  if (SPEED > 100u)
    AUX0 = static_cast<uint16_t>(3u + ((255u - SPEED) >> 3u));
  else
    AUX0 = static_cast<uint16_t>(22u + ((100u - SPEED) >> 1u));
  if (AUX0 < 1u)
    AUX0 = 1u;

  // Background colour (scale down COLOR1 to very dim)
  uint32_t bg = color_fade(COLOR1, 16u);

  uint16_t prng16 = 11337u;

  for (int32_t i = 0; i < SEGLEN; i++) {
    prng16 = static_cast<uint16_t>(prng16 * 2053u) + 1384u;
    uint16_t clock_offset = prng16;
    prng16 = static_cast<uint16_t>(prng16 * 2053u) + 1384u;
    uint8_t speed_mult = static_cast<uint8_t>((((prng16 & 0xFFu) >> 4u) + (prng16 & 0x0Fu)) & 0x0Fu) + 8u;
    uint32_t clock30 = static_cast<uint32_t>((NOW * speed_mult) >> 3u) + clock_offset;
    uint8_t salt = static_cast<uint8_t>(prng16 >> 8u);

    // Twinkle calc
    uint32_t ticks = clock30 / static_cast<uint32_t>(AUX0 > 0u ? AUX0 : 1u);
    uint8_t fast8 = static_cast<uint8_t>(ticks);
    uint16_t slow16 = static_cast<uint16_t>((ticks >> 8u) + salt);
    slow16 += sin8(static_cast<uint8_t>(slow16));
    slow16 = static_cast<uint16_t>(slow16 * 2053u) + 1384u;
    uint8_t slow8 = static_cast<uint8_t>((slow16 & 0xFFu) + (slow16 >> 8u));

    uint32_t density = static_cast<uint32_t>(INTENSITY >> 5u) + 1u;
    uint8_t bright = 0u;
    if (((slow8 & 0x0Eu) >> 1u) < density) {
      // Twinklefox triangle: fast rise, slow fall
      if (fast8 < 86u) {
        bright = fast8 * 3u;
      } else {
        uint8_t ph = fast8 - 86u;
        bright = static_cast<uint8_t>(255u - (ph + (ph >> 1u)));
      }
    }

    uint8_t hue = static_cast<uint8_t>(slow8 - salt);
    uint32_t col;
    if (bright > 0u) {
      col = color_fade(ctx.pal_color(hue), bright);
      // Warm incandescent dim: shift toward red as brightness drops
      if (fast8 >= 128u) {
        uint8_t cooling = (fast8 - 128u) >> 4u;
        uint8_t cg = G(col) > cooling ? G(col) - cooling : 0u;
        uint8_t cb = B(col) > cooling * 2u ? B(col) - cooling * 2u : 0u;
        col = RGBW32(R(col), cg, cb, W(col));
      }
    } else {
      col = 0u;
    }

    // Show brighter of twinkle or background
    uint8_t cbright = static_cast<uint8_t>((static_cast<uint16_t>(R(col)) + G(col) + B(col)) / 3u);
    uint8_t bgbright = static_cast<uint8_t>((static_cast<uint16_t>(R(bg)) + G(bg) + B(bg)) / 3u);
    int16_t delta = static_cast<int16_t>(cbright) - static_cast<int16_t>(bgbright);
    if (delta >= 32 || bg == 0u) {
      ctx.set_pixel(i, col);
    } else if (delta > 0) {
      ctx.set_pixel(i, color_blend(bg, col, static_cast<uint8_t>(delta * 8u)));
    } else {
      ctx.set_pixel(i, bg);
    }
  }
}

// 97 — Flow Stripe (hue-scrolling sine stripe pattern, by WLED / Aircoookie)
// Source: WLED mode_FlowStripe (FX.cpp line 5069)
void fx_flow_stripe(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  int32_t hl = SEGLEN * 10 / 13;
  uint8_t hue = static_cast<uint8_t>(NOW / (static_cast<uint32_t>(SPEED) + 1u));
  uint32_t t = NOW / (static_cast<uint32_t>(INTENSITY) / 8u + 1u);

  for (int32_t i = 0; i < SEGLEN; i++) {
    int32_t dist = i - hl;
    if (dist < 0)
      dist = -dist;
    int32_t c = (dist * 127) / (hl > 0 ? hl : 1);
    uint8_t cv = static_cast<uint8_t>(c & 0xFFu);
    cv = sin8(cv);
    cv = sin8(static_cast<uint8_t>((static_cast<uint32_t>(cv) / 2u + t) & 0xFFu));
    uint8_t b = sin8(static_cast<uint8_t>((static_cast<uint32_t>(cv) + (t >> 3u)) & 0xFFu));
    ctx.set_pixel(i, ctx.pal_color(static_cast<uint8_t>(b + hue)));
  }
}

// 98 — Twinklecat (instant-on, slow fade-off twinkle variant)
// Source: WLED twinklefox_base(true) (FX.cpp line 2708)
void fx_twinklecat(EffectContext &ctx) {
  if (SPEED > 100u)
    AUX0 = static_cast<uint16_t>(3u + ((255u - SPEED) >> 3u));
  else
    AUX0 = static_cast<uint16_t>(22u + ((100u - SPEED) >> 1u));
  if (AUX0 < 1u)
    AUX0 = 1u;

  uint32_t bg = color_fade(COLOR1, 16u);
  uint16_t prng16 = 11337u;

  for (int32_t i = 0; i < SEGLEN; i++) {
    prng16 = static_cast<uint16_t>(prng16 * 2053u) + 1384u;
    uint16_t clock_offset = prng16;
    prng16 = static_cast<uint16_t>(prng16 * 2053u) + 1384u;
    uint8_t speed_mult = static_cast<uint8_t>((((prng16 & 0xFFu) >> 4u) + (prng16 & 0x0Fu)) & 0x0Fu) + 8u;
    uint32_t clock30 = static_cast<uint32_t>((NOW * speed_mult) >> 3u) + clock_offset;
    uint8_t salt = static_cast<uint8_t>(prng16 >> 8u);

    uint32_t ticks = clock30 / static_cast<uint32_t>(AUX0 > 0u ? AUX0 : 1u);
    uint8_t fast8 = static_cast<uint8_t>(ticks);
    uint16_t slow16 = static_cast<uint16_t>((ticks >> 8u) + salt);
    slow16 += sin8(static_cast<uint8_t>(slow16));
    slow16 = static_cast<uint16_t>(slow16 * 2053u) + 1384u;
    uint8_t slow8 = static_cast<uint8_t>((slow16 & 0xFFu) + (slow16 >> 8u));

    uint32_t density = static_cast<uint32_t>(INTENSITY >> 5u) + 1u;
    uint8_t bright = 0u;
    if (((slow8 & 0x0Eu) >> 1u) < density) {
      // Twinklecat: instant on (255 - phase) → fades to 0
      bright = 255u - fast8;
    }

    uint8_t hue = static_cast<uint8_t>(slow8 - salt);
    uint32_t col = (bright > 0u) ? color_fade(ctx.pal_color(hue), bright) : 0u;

    uint8_t cbright = static_cast<uint8_t>((static_cast<uint16_t>(R(col)) + G(col) + B(col)) / 3u);
    uint8_t bgbright = static_cast<uint8_t>((static_cast<uint16_t>(R(bg)) + G(bg) + B(bg)) / 3u);
    int16_t delta = static_cast<int16_t>(cbright) - static_cast<int16_t>(bgbright);
    if (delta >= 32 || bg == 0u) {
      ctx.set_pixel(i, col);
    } else if (delta > 0) {
      ctx.set_pixel(i, color_blend(bg, col, static_cast<uint8_t>(delta * 8u)));
    } else {
      ctx.set_pixel(i, bg);
    }
  }
}

// 99 — Dissolve Random (dissolve using random wheel colours instead of palette)
// Source: WLED mode_dissolve_random (FX.cpp line 755) — dissolve() with random col arg
void fx_dissolve_random(EffectContext &ctx) {
  size_t data_size = static_cast<size_t>(SEGLEN) * sizeof(uint32_t);
  if (!ctx.env->allocate_data(data_size)) {
    ctx.fill_black();
    return;
  }
  auto *pixels = reinterpret_cast<uint32_t *>(ctx.env->data);

  if (CALL == 0) {
    for (int32_t i = 0; i < SEGLEN; i++)
      pixels[i] = COLOR1;
    AUX0 = 1u;
    STEP = 0u;
  }

  uint32_t attempts = 1u + static_cast<uint32_t>(SEGLEN) / 15u;
  for (uint32_t j = 0u; j < attempts; j++) {
    if (hw_random8() <= INTENSITY) {
      for (uint8_t t = 0u; t < 10u; t++) {
        uint16_t idx = hw_random16(static_cast<uint16_t>(SEGLEN));
        if (AUX0) {
          if (pixels[idx] == COLOR1) {
            pixels[idx] = ctx.wheel(hw_random8());
            break;
          }
        } else {
          if (pixels[idx] != COLOR1) {
            pixels[idx] = COLOR1;
            break;
          }
        }
      }
    }
  }

  for (int32_t i = 0; i < SEGLEN; i++)
    ctx.set_pixel(i, pixels[i]);

  STEP++;
  if (STEP > static_cast<uint32_t>(255u - SPEED) + 15u) {
    AUX0 ^= 1u;
    STEP = 0u;
  }
}

// 100 — Sweep Random (random colour wipe alternating from each end)
// Source: WLED mode_color_sweep_random / color_wipe(true, true) (FX.cpp line 344)
void fx_sweep_random(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  uint32_t cycleTime = 750u + (255u - SPEED) * 150u;
  uint32_t perc = NOW % cycleTime;
  int32_t prog = static_cast<int32_t>((perc * static_cast<uint32_t>(SEGLEN) * 2u) / cycleTime);
  bool forward = prog < SEGLEN;
  int32_t led = forward ? prog : (SEGLEN * 2 - prog - 1);

  uint32_t cycle = NOW / cycleTime;
  if (cycle != STEP) {
    AUX0 = AUX1;
    AUX1 = hw_random8();
    STEP = cycle;
  }

  uint32_t col1 = ctx.wheel(static_cast<uint8_t>(AUX0));
  uint32_t col2 = ctx.wheel(static_cast<uint8_t>(AUX1));

  // Reversed wipe (sweep from right / alternating like color_wipe rev=true)
  for (int32_t i = 0; i < SEGLEN; i++) {
    bool use_col2;
    if (forward)
      use_col2 = (i >= SEGLEN - 1 - led);
    else
      use_col2 = (i < SEGLEN - led);
    ctx.set_pixel(i, use_col2 ? col2 : col1);
  }
}

// 101 — Chase Flash Random (chase flash with rainbow-cycling flash colour)
// Source: WLED mode_chase_flash_random (FX.cpp line 1158)
// Identical to Chase Flash but flash colour cycles through wheel each trigger.
void fx_chase_flash_random(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }
  uint32_t chase_period = 500u + (255u - SPEED) * 10u;
  uint32_t flash_cycle = 100u + (255u - SPEED) * 2u;

  uint32_t cur_flash_cycle = NOW / flash_cycle;
  if (cur_flash_cycle != STEP) {
    if ((cur_flash_cycle % 4u) == 0u) {
      // new flash: pick a new wheel colour
      AUX1 = static_cast<uint16_t>((AUX1 + hw_random8(42u, 128u)) & 0xFFu);
    }
    STEP = cur_flash_cycle;
  }

  if ((NOW / flash_cycle) % 4u == 0u) {
    ctx.fill(ctx.wheel(static_cast<uint8_t>(AUX1)));
    return;
  }

  ctx.fill(COLOR1);
  int32_t pos = static_cast<int32_t>((NOW % chase_period) * static_cast<uint32_t>(SEGLEN) / chase_period);
  int32_t dot_size = 1 + (INTENSITY >> 5);
  for (int32_t i = 0; i < dot_size && pos + i < SEGLEN; i++)
    ctx.set_pixel(pos + i, ctx.pal_color_at(pos + i));
}

// ============================================================
// Batch 6 — effects 102-111
// ============================================================

// 102 — Sparkle Dark (Flash Sparkle)
// Source: WLED mode_flash_sparkle (FX.cpp)
// AUX0 = interval length ms, STEP = last fire timestamp
void fx_sparkle_dark(EffectContext &ctx) {
  for (int32_t i = 0; i < SEGLEN; i++)
    ctx.set_pixel(i, ctx.pal_color_at(i));

  // throttle: fire at most once per (255-SPEED) ms
  uint32_t interval = static_cast<uint32_t>(255u - SPEED);
  if (NOW - STEP > static_cast<uint32_t>(AUX0)) {
    if (hw_random8(static_cast<uint8_t>((255u - INTENSITY) >> 4u)) == 0u) {
      uint16_t idx = hw_random16(static_cast<uint16_t>(SEGLEN));
      ctx.set_pixel(static_cast<int32_t>(idx), COLOR1);
    }
    STEP = NOW;
    AUX0 = static_cast<uint16_t>(interval);
  }
}

// 103 — Sparkle+ (Hyper Sparkle) — same as Sparkle Dark but flashes a burst of pixels
// Source: WLED mode_hyper_sparkle (FX.cpp)
void fx_sparkle_plus(EffectContext &ctx) {
  for (int32_t i = 0; i < SEGLEN; i++)
    ctx.set_pixel(i, ctx.pal_color_at(i));

  uint32_t interval = static_cast<uint32_t>(255u - SPEED);
  if (NOW - STEP > static_cast<uint32_t>(AUX0)) {
    if (hw_random8(static_cast<uint8_t>((255u - INTENSITY) >> 4u)) == 0u) {
      int32_t burst = std::max<int32_t>(1, SEGLEN / 3);
      for (int32_t j = 0; j < burst; j++) {
        uint16_t idx = hw_random16(static_cast<uint16_t>(SEGLEN));
        ctx.set_pixel(static_cast<int32_t>(idx), COLOR1);
      }
    }
    STEP = NOW;
    AUX0 = static_cast<uint16_t>(interval);
  }
}

// 104 — Chase 2 (Running Color)
// Source: WLED mode_running_color → running(c0, c1, false) (FX.cpp)
// AUX0 = scroll offset within [0, width*2)
void fx_chase2(EffectContext &ctx) {
  int32_t width = 1 + static_cast<int32_t>(INTENSITY >> 4u);
  uint32_t cycleTime = 50u + static_cast<uint32_t>(255u - SPEED);
  uint32_t it = NOW / cycleTime;
  bool use_palette = (ctx.params->palette_id != 0);

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint32_t color1 = use_palette ? ctx.pal_color_at(i) : COLOR0;
    // Shift so that the lit band sits around AUX0 in the repeating period
    int32_t rel = (i % (width * 2) + width * 2 - static_cast<int32_t>(AUX0) % (width * 2)) % (width * 2);
    ctx.set_pixel(i, (rel < width) ? color1 : COLOR1);
  }

  if (it != STEP) {
    AUX0 = static_cast<uint16_t>((static_cast<int32_t>(AUX0) + 1) % (width * 2));
    STEP = it;
  }
}

// 105 — Rainbow Runner (Chase Rainbow White)
// Source: WLED mode_chase_rainbow_white (FX.cpp)
// AUX0 = leading-edge position 'a'
void fx_rainbow_runner(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(ctx.pal_color(0));
    return;
  }
  uint32_t counter = NOW * (static_cast<uint32_t>(SPEED >> 2u) + 1u);
  uint32_t a = (static_cast<uint64_t>(counter) * static_cast<uint32_t>(SEGLEN)) >> 16u;
  a %= static_cast<uint32_t>(SEGLEN);
  AUX0 = static_cast<uint16_t>(a);

  uint32_t size = 1u + (static_cast<uint32_t>(INTENSITY) * static_cast<uint32_t>(SEGLEN)) / 1024u;
  if (size < 1u)
    size = 1u;

  uint32_t b = (a + size) % static_cast<uint32_t>(SEGLEN);
  uint32_t c = (b + size) % static_cast<uint32_t>(SEGLEN);

  uint32_t color2 =
      ctx.wheel(static_cast<uint8_t>(((a * 256u / static_cast<uint32_t>(SEGLEN)) + (CALL & 0xFFu)) & 0xFFu));
  uint32_t color3 =
      ctx.wheel(static_cast<uint8_t>((((a + 1u) * 256u / static_cast<uint32_t>(SEGLEN)) + (CALL & 0xFFu)) & 0xFFu));

  // Fill background
  ctx.fill(COLOR0);

  // Band a..b = color2
  if (a <= b) {
    for (uint32_t i = a; i < b; i++)
      ctx.set_pixel(static_cast<int32_t>(i), color2);
  } else {
    for (uint32_t i = a; i < static_cast<uint32_t>(SEGLEN); i++)
      ctx.set_pixel(static_cast<int32_t>(i), color2);
    for (uint32_t i = 0u; i < b; i++)
      ctx.set_pixel(static_cast<int32_t>(i), color2);
  }

  // Band b..c = color3
  if (b <= c) {
    for (uint32_t i = b; i < c; i++)
      ctx.set_pixel(static_cast<int32_t>(i), color3);
  } else {
    for (uint32_t i = b; i < static_cast<uint32_t>(SEGLEN); i++)
      ctx.set_pixel(static_cast<int32_t>(i), color3);
    for (uint32_t i = 0u; i < c; i++)
      ctx.set_pixel(static_cast<int32_t>(i), color3);
  }
}

// 106 — Noise 1 (Perlin noise, palette-mapped)
// Source: WLED mode_noise16_1 (FX.cpp)
void fx_noise1(EffectContext &ctx) {
  static constexpr uint32_t SCALE = 320u;
  STEP += 1u + static_cast<uint32_t>(SPEED) / 16u;

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t shift_x = beatsin8(11u, 0u, 255u, NOW);
    uint32_t shift_y = STEP / 42u;
    uint32_t real_x = static_cast<uint32_t>(static_cast<uint32_t>(i) + shift_x) * SCALE;
    uint32_t real_y = static_cast<uint32_t>(static_cast<uint32_t>(i) + shift_y) * SCALE;
    uint32_t real_z = STEP;
    uint8_t noise_val = static_cast<uint8_t>(inoise16(real_x, real_y, real_z) >> 8u);
    uint8_t index = sin8(static_cast<uint8_t>(noise_val * 3u));
    ctx.set_pixel(i, ctx.pal_color(index));
  }
}

// 107 — Noise 2 (Perlin noise horizontal scroll)
// Source: WLED mode_noise16_2 (FX.cpp)
void fx_noise2(EffectContext &ctx) {
  static constexpr uint32_t SCALE = 1000u;
  STEP += 1u + static_cast<uint32_t>(SPEED) / 2u;

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint32_t shift_x = STEP >> 6u;
    uint32_t real_x = (static_cast<uint32_t>(i) + shift_x) * SCALE;
    uint16_t raw16 = static_cast<uint16_t>(inoise16(real_x, 0u, 4223u) >> 8u);
    uint8_t noise_val = static_cast<uint8_t>(raw16 & 0xFFu);
    uint8_t index = sin8(static_cast<uint8_t>(noise_val * 3u));
    ctx.set_pixel(i, ctx.pal_color(index, noise_val));
  }
}

// 108 — Noise 3 (3D Perlin noise, brightness-modulated)
// Source: WLED mode_noise16_3 (FX.cpp)
void fx_noise3(EffectContext &ctx) {
  static constexpr uint32_t SCALE = 800u;
  STEP += 1u + static_cast<uint32_t>(SPEED);

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint32_t real_x = (static_cast<uint32_t>(i) + 4223u) * SCALE;
    uint32_t real_y = (static_cast<uint32_t>(i) + 1234u) * SCALE;
    uint32_t real_z = STEP * 8u;
    uint8_t noise_val = static_cast<uint8_t>(inoise16(real_x, real_y, real_z) >> 8u);
    uint8_t index = sin8(static_cast<uint8_t>(noise_val * 3u));
    ctx.set_pixel(i, ctx.pal_color(index, noise_val));
  }
}

// 109 — Noise 4 (time-driven Perlin noise)
// Source: WLED mode_noise16_4 (FX.cpp)
void fx_noise4(EffectContext &ctx) {
  uint32_t stp = (NOW * static_cast<uint32_t>(SPEED)) >> 7u;
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint32_t raw = inoise16(static_cast<uint32_t>(i) << 12u, stp);
    uint8_t index = static_cast<uint8_t>(raw >> 8u);
    ctx.set_pixel(i, ctx.pal_color(index));
  }
}

// 110 — Phased Noise (Perlin-based phased waves, by Andrew Tuline)
// Source: WLED mode_phased_noise → phased_base(1) (FX.cpp)
void fx_phased_noise(EffectContext &ctx) {
  STEP += 100u;
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint16_t n = static_cast<uint16_t>(
        inoise16(static_cast<uint32_t>(i) * 4000u, STEP + static_cast<uint32_t>(i) * 2000u) >> 8u);
    uint8_t val = static_cast<uint8_t>(n & 0xFFu);
    ctx.set_pixel(i, color_blend(COLOR1, ctx.pal_color(val), val));
  }
}

// 111 — Wavesins (sine wave brightness with beatsin palette, by Andrew Tuline)
// Source: WLED mode_wavesins (FX.cpp)
void fx_wavesins(EffectContext &ctx) {
  for (int32_t i = 0; i < SEGLEN; i++) {
    uint8_t bri = sin8(static_cast<uint8_t>((NOW / 4u) + static_cast<uint32_t>(i) * static_cast<uint32_t>(INTENSITY)));
    uint8_t c1_plus_c2 = static_cast<uint8_t>(C1 + C2);  // intentional uint8 wrap
    uint8_t phase = static_cast<uint8_t>(static_cast<uint32_t>(i) * (static_cast<uint32_t>(C3) << 3u));
    uint8_t index = beatsin8(static_cast<uint8_t>(SPEED), C1, c1_plus_c2, NOW, phase);
    ctx.set_pixel(i, ctx.pal_color(index, bri));
  }
}

// ============================================================
// Batch 7 — effects 112-121  (complex / physics effects)
// ============================================================

// 112 — Shimmer (moving spotlight with optional granular modulation)
// Source: WLED mode_shimmer (FX.cpp)
// Data: sizeof(uint32_t) for last_time; AUX0=pause_timer, AUX1=input_state_cache, STEP=position<<8
void fx_shimmer(EffectContext &ctx) {
  if (!ctx.env->allocate_data(sizeof(uint32_t))) {
    ctx.fill(COLOR0);
    return;
  }
  auto *last_time = reinterpret_cast<uint32_t *>(ctx.env->data);

  uint32_t radius = (static_cast<uint32_t>(C1) * static_cast<uint32_t>(SEGLEN) >> 7u) + 1u;
  uint32_t traversal_dist = (static_cast<uint32_t>(SEGLEN) + 2u * radius) << 8u;
  uint32_t traversal_time = 200u + static_cast<uint32_t>(255u - SPEED) * 80u;
  uint32_t speed_fac = (traversal_dist << 5u) / traversal_time;

  int32_t position = static_cast<int32_t>(STEP);
  uint16_t input_state = static_cast<uint16_t>((static_cast<uint16_t>(INTENSITY) << 8u) | C1);

  if (CALL == 0u || input_state != AUX1) {
    position = -static_cast<int32_t>(radius << 8u);
    AUX0 = 0u;
    *last_time = NOW;
    AUX1 = input_state;
  }

  if (SPEED > 0u) {
    uint32_t delta_time = (NOW - *last_time) & 0x7Fu;
    *last_time = NOW;
    if (AUX0 > 0u) {
      AUX0 = static_cast<uint16_t>(AUX0 > delta_time ? AUX0 - static_cast<uint16_t>(delta_time) : 0u);
    } else {
      int32_t step_val = static_cast<int32_t>(1u + ((speed_fac * delta_time) >> 5u));
      position += step_val;
      int32_t limit = static_cast<int32_t>((static_cast<uint32_t>(SEGLEN) + radius) << 8u);
      if (position > limit) {
        uint16_t pause = static_cast<uint16_t>(static_cast<uint32_t>(INTENSITY) * 236u);
        if (ctx.params->check3)
          pause = hw_random16(static_cast<uint16_t>(pause + 1000u));
        AUX0 = pause;
        position = -static_cast<int32_t>(radius << 8u);
      }
      STEP = static_cast<uint32_t>(position);
    }
    if (ctx.params->check2)
      position = static_cast<int32_t>(static_cast<uint32_t>(SEGLEN) << 8u) - position;
  } else {
    position = static_cast<int32_t>(static_cast<uint32_t>(SEGLEN) << 7u);
  }

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint32_t dist = static_cast<uint32_t>(abs(position - (i << 8)));
    uint32_t r8 = radius << 8u;
    if (dist < r8) {
      uint32_t spot_col = ctx.pal_color(
          static_cast<uint8_t>((static_cast<uint32_t>(i) * 255u) / static_cast<uint32_t>(SEGLEN > 1 ? SEGLEN : 1)));
      if (C2 > 0u) {
        uint8_t mod_val;
        if (ctx.params->check1) {
          mod_val = static_cast<uint8_t>(
              (sin16(static_cast<uint16_t>(static_cast<uint32_t>(i) * static_cast<uint32_t>(C2) * 64u +
                                           NOW * static_cast<uint32_t>(C3) * 32u)) >>
               8u) +
              128);
        } else {
          mod_val = static_cast<uint8_t>(inoise16(static_cast<uint32_t>(i) * static_cast<uint32_t>(C2) * 128u,
                                                  NOW * static_cast<uint32_t>(C3) * 32u) >>
                                         8u);
        }
        spot_col = color_fade(spot_col, mod_val);
      }
      uint8_t blend = static_cast<uint8_t>((dist * 255u) / r8);
      ctx.set_pixel(i, color_blend(spot_col, COLOR1, blend));
    } else {
      ctx.set_pixel(i, COLOR1);
    }
  }
}

// 113 — Aurora (overlapping sine-wave aurora streamers)
// Source: WLED AuroraWave class + mode_aurora (FX.cpp)
// Data: MAX_WAVES * sizeof(AuroraWave)
void fx_aurora(EffectContext &ctx) {
  static constexpr int32_t MAX_WAVES = 20;

  struct AuroraWave {
    int32_t center;  // fixed-point position * 65536
    uint32_t age_factor;  // 0..65536 fade envelope
    uint16_t ttl;  // lifetime in frames
    uint16_t age;  // current age in frames
    uint16_t width;  // half-width in pixels
    uint32_t base_alpha;  // 0..65536
    int32_t speed_fac;  // signed pixels/frame * 65536
    bool going_left;
    bool alive;
    uint32_t color;
  };

  if (!ctx.env->allocate_data(static_cast<size_t>(MAX_WAVES) * sizeof(AuroraWave))) {
    ctx.fill(COLOR0);
    return;
  }
  auto *waves = reinterpret_cast<AuroraWave *>(ctx.env->data);

  // Backlight from user colours
  uint8_t bg_bri = 0u;
  uint8_t col_count = 0u;
  if (COLOR0 != 0u)
    col_count++;
  if (COLOR1 != 0u)
    col_count++;
  if (COLOR2 != 0u)
    col_count++;
  bg_bri = gamma8_inv(col_count);

  int32_t num_waves =
      std::max<int32_t>(1, std::min<int32_t>(MAX_WAVES, 2 + static_cast<int32_t>(INTENSITY) * MAX_WAVES / 255));

  // Update / respawn waves
  for (int32_t w = 0; w < num_waves; w++) {
    AuroraWave &wave = waves[w];
    if (!wave.alive || wave.age >= wave.ttl) {
      // Spawn
      wave.ttl = static_cast<uint16_t>(hw_random16(120u) + 80u);
      wave.age = 0u;
      wave.width = static_cast<uint16_t>(
          std::max<int32_t>(1, SEGLEN / 4 + static_cast<int32_t>(hw_random8(static_cast<uint8_t>(SEGLEN / 4 + 1u)))));
      wave.center = (hw_random16(static_cast<uint16_t>(SEGLEN)) * 65536);
      wave.going_left = (hw_random8(2u) == 0u);
      uint32_t speed_pct = static_cast<uint32_t>(SPEED) * 65536u / 255u;
      wave.speed_fac = static_cast<int32_t>(1u + speed_pct / 16u);
      if (wave.going_left)
        wave.speed_fac = -wave.speed_fac;
      wave.base_alpha = 39322u + static_cast<uint32_t>(hw_random8(static_cast<uint8_t>(26214u >> 8u))) * 256u;
      wave.color = ctx.pal_color(hw_random8());
      wave.alive = true;
    } else {
      // Advance
      wave.center += wave.speed_fac;
      wave.age++;
    }

    // Age factor: fade in first 25% of TTL, out last 25%
    uint16_t fade_frames = wave.ttl / 4u;
    if (fade_frames < 1u)
      fade_frames = 1u;
    if (wave.age < fade_frames) {
      wave.age_factor = (static_cast<uint32_t>(wave.age) * 65536u) / fade_frames;
    } else if (wave.age > wave.ttl - fade_frames) {
      uint16_t remaining = wave.ttl - wave.age;
      wave.age_factor = (static_cast<uint32_t>(remaining) * 65536u) / fade_frames;
    } else {
      wave.age_factor = 65536u;
    }
  }

  // Render: clear then additively mix waves
  ctx.fill_black();

  for (int32_t i = 0; i < SEGLEN; i++) {
    uint32_t accum_r = 0u, accum_g = 0u, accum_b = 0u;

    for (int32_t w = 0; w < num_waves; w++) {
      const AuroraWave &wave = waves[w];
      if (!wave.alive)
        continue;
      int32_t center_px = wave.center / 65536;
      int32_t dist_px = abs(i - center_px);
      if (dist_px >= static_cast<int32_t>(wave.width))
        continue;
      uint32_t falloff = static_cast<uint32_t>(wave.width - dist_px) * 65536u / static_cast<uint32_t>(wave.width);
      uint32_t alpha = (wave.age_factor >> 8u) * (wave.base_alpha >> 8u) * (falloff >> 8u) >> 8u;
      alpha = std::min<uint32_t>(alpha, 255u);
      accum_r += static_cast<uint32_t>(R(wave.color)) * alpha >> 8u;
      accum_g += static_cast<uint32_t>(G(wave.color)) * alpha >> 8u;
      accum_b += static_cast<uint32_t>(B(wave.color)) * alpha >> 8u;
    }

    // Backlight
    if (bg_bri > 0u) {
      uint32_t bg_col = COLOR0 ? COLOR0 : (COLOR1 ? COLOR1 : COLOR2);
      accum_r += static_cast<uint32_t>(R(bg_col)) * bg_bri >> 8u;
      accum_g += static_cast<uint32_t>(G(bg_col)) * bg_bri >> 8u;
      accum_b += static_cast<uint32_t>(B(bg_col)) * bg_bri >> 8u;
    }

    ctx.set_pixel(i, RGBW32(static_cast<uint8_t>(std::min<uint32_t>(accum_r, 255u)),
                            static_cast<uint8_t>(std::min<uint32_t>(accum_g, 255u)),
                            static_cast<uint8_t>(std::min<uint32_t>(accum_b, 255u)), 0u));
  }
}

// 114 — Tetrix (falling blocks stack up, then fade out)
// Source: WLED mode_tetrix (FX.cpp)
void fx_tetrix(EffectContext &ctx) {
  struct TetrisBrick {
    float pos;
    float speed;
    uint8_t col;
    uint16_t brick_w;
    uint16_t stack;
    uint32_t step_ts;
  };

  if (!ctx.env->allocate_data(sizeof(TetrisBrick))) {
    ctx.fill(COLOR0);
    return;
  }
  auto *brick = reinterpret_cast<TetrisBrick *>(ctx.env->data);

  if (CALL == 0u) {
    brick->pos = static_cast<float>(SEGLEN);
    brick->speed = static_cast<float>(std::max<int32_t>(1, 5000 / std::max<int32_t>(1, SEGLEN))) / 1000.0f;
    brick->brick_w = static_cast<uint16_t>((static_cast<uint32_t>(INTENSITY >> 5u) + 1u) *
                                           (1u + static_cast<uint32_t>(SEGLEN) / 64u));
    brick->col = hw_random8();
    brick->stack = 0u;
    brick->step_ts = NOW;
  }

  // Background
  ctx.fill(COLOR1);

  // Check for fade-out state (AUX0 == 1 → stack is full, fading)
  if (AUX0 == 1u) {
    // Fade out all pixels then reset
    if (NOW - STEP > 2000u) {
      AUX0 = 0u;
      brick->stack = 0u;
      brick->pos = static_cast<float>(SEGLEN);
      brick->col = hw_random8();
      STEP = NOW;
    }
    ctx.fade_to_black(220u);
    return;
  }

  // Draw stack
  for (int32_t i = 0; i < static_cast<int32_t>(brick->stack) && i < SEGLEN; i++)
    ctx.set_pixel(i, ctx.pal_color(brick->col));

  // Advance brick position (frame-rate independent via timestamp)
  uint32_t elapsed = NOW - brick->step_ts;
  if (elapsed > 20u) {
    float frames = static_cast<float>(elapsed) / 20.0f;
    brick->pos -= brick->speed * frames * static_cast<float>(SEGLEN);
    brick->step_ts = NOW;
  }

  int32_t brick_bot = static_cast<int32_t>(brick->pos);
  if (brick_bot < 0)
    brick_bot = 0;

  // Land condition
  if (brick_bot <= static_cast<int32_t>(brick->stack)) {
    brick->stack += brick->brick_w;
    if (static_cast<int32_t>(brick->stack) >= SEGLEN) {
      brick->stack = static_cast<uint16_t>(SEGLEN);
      AUX0 = 1u;  // trigger fade-out
      STEP = NOW;
    }
    // New brick
    brick->pos = static_cast<float>(SEGLEN);
    brick->col = ctx.params->check1 ? brick->col : hw_random8();
    brick->speed = static_cast<float>(std::max<int32_t>(1, 5000 / std::max<int32_t>(1, SEGLEN))) / 1000.0f;
    brick->step_ts = NOW;
    return;
  }

  // Draw falling brick
  uint32_t brick_col = ctx.pal_color(brick->col);
  for (int32_t i = brick_bot; i < std::min<int32_t>(brick_bot + static_cast<int32_t>(brick->brick_w), SEGLEN); i++)
    ctx.set_pixel(i, brick_col);
}

// 115 — Rolling Balls (gravity-bounce physics, multi-ball)
// Source: WLED mode_rollingBalls (FX.cpp)
struct RollingBall {
  uint32_t last_bounce;
  float mass;
  float vel;
  float height;
};

void fx_rolling_balls(EffectContext &ctx) {
  static constexpr int32_t MAX_BALLS = 16;
  int32_t num_balls = std::min<int32_t>(MAX_BALLS, static_cast<int32_t>(INTENSITY / 16u) + 1);

  if (!ctx.env->allocate_data(static_cast<size_t>(MAX_BALLS) * sizeof(RollingBall))) {
    ctx.fill(COLOR0);
    return;
  }
  auto *balls = reinterpret_cast<RollingBall *>(ctx.env->data);

  if (CALL == 0u) {
    for (int32_t i = 0; i < MAX_BALLS; i++) {
      balls[i].last_bounce = NOW;
      balls[i].mass = 0.5f + static_cast<float>(i) * 0.1f;
      balls[i].vel = 0.0f;
      balls[i].height = 1.0f;
    }
  }

  float gravity = -0.001f * (0.5f + static_cast<float>(SPEED) / 255.0f);

  if (ctx.params->check3) {
    ctx.fade_to_black(220u);
  } else if (!ctx.params->check2) {
    ctx.fill(COLOR1);
  }

  for (int32_t i = 0; i < num_balls; i++) {
    uint32_t dt_ms = NOW - balls[i].last_bounce;
    float t = static_cast<float>(dt_ms) / 1000.0f;
    float h = balls[i].vel * t + 0.5f * gravity * t * t;

    if (h < 0.0f) {
      h = 0.0f;
      balls[i].vel = -balls[i].vel * 0.85f;
      if (balls[i].vel > -0.002f)
        balls[i].vel = 0.15f + static_cast<float>(hw_random8(50u)) / 500.0f;
      balls[i].last_bounce = NOW;
    }
    if (h > 1.0f)
      h = 1.0f;
    balls[i].height = h;

    int32_t pos = static_cast<int32_t>(h * static_cast<float>(SEGLEN - 1));
    if (pos >= 0 && pos < SEGLEN) {
      uint32_t col;
      if (ctx.params->palette_id != 0) {
        col = ctx.pal_color(static_cast<uint8_t>(i * 42u));
      } else {
        col = ctx.params->colors[static_cast<size_t>(i) % 3u];
      }
      ctx.set_pixel(pos, col);
    }

    // Collide with other balls
    if (ctx.params->check1) {
      for (int32_t j = i + 1; j < num_balls; j++) {
        int32_t pos_j = static_cast<int32_t>(balls[j].height * static_cast<float>(SEGLEN - 1));
        if (abs(pos - pos_j) <= 1) {
          float v1 = balls[i].vel;
          float v2 = balls[j].vel;
          float m1 = balls[i].mass;
          float m2 = balls[j].mass;
          float total_m = m1 + m2;
          balls[i].vel = ((m1 - m2) * v1 + 2.0f * m2 * v2) / total_m;
          balls[j].vel = ((m2 - m1) * v2 + 2.0f * m1 * v1) / total_m;
        }
      }
    }
  }
}

// 116 — Fireworks Starburst
// Source: WLED mode_starburst (FX.cpp)
static constexpr int32_t STARBURST_MAX_FRAG = 12;

struct StarParticle {
  uint32_t color;
  uint32_t birth;
  uint32_t last;
  float vel;
  uint16_t pos;
  float fragment[STARBURST_MAX_FRAG];
};

void fx_starburst(EffectContext &ctx) {
  if (SEGLEN <= 1) {
    ctx.fill(COLOR0);
    return;
  }

  static constexpr size_t MAX_STARS = 8u;
  size_t data_size = MAX_STARS * sizeof(StarParticle);
  if (!ctx.env->allocate_data(data_size)) {
    ctx.fill(COLOR0);
    return;
  }
  auto *stars = reinterpret_cast<StarParticle *>(ctx.env->data);

  // Number of active stars based on segment size
  int32_t num_stars = std::min<int32_t>(static_cast<int32_t>(MAX_STARS), 1 + SEGLEN / 8);

  if (!ctx.params->check1)
    ctx.fill_black();

  for (int32_t s = 0; s < num_stars; s++) {
    StarParticle &star = stars[s];
    uint32_t age = NOW - star.birth;

    // Birth chance proportional to SPEED
    bool spawn = (star.birth == 0u) || (age > 2000u);
    if (!spawn && hw_random8(static_cast<uint8_t>(255u - SPEED)) == 0u)
      spawn = true;

    if (spawn) {
      star.birth = NOW;
      star.last = NOW;
      star.pos = static_cast<uint16_t>(hw_random16(static_cast<uint16_t>(SEGLEN)));
      star.color = ctx.pal_color(hw_random8());
      int32_t num_frags = 2 + static_cast<int32_t>(INTENSITY * STARBURST_MAX_FRAG / 255);
      if (num_frags > STARBURST_MAX_FRAG)
        num_frags = STARBURST_MAX_FRAG;
      for (int32_t f = 0; f < num_frags; f++) {
        float v = 1.0f + static_cast<float>(hw_random8(static_cast<uint8_t>(SEGLEN / 4 + 1u))) / 4.0f;
        star.fragment[f] = (f % 2 == 0) ? v : -v;
      }
      age = 0u;
    }

    // Advance fragment positions
    uint32_t dt = NOW - star.last;
    star.last = NOW;
    float dt_s = static_cast<float>(dt) / 1000.0f;

    int32_t num_frags2 = 2 + static_cast<int32_t>(INTENSITY * STARBURST_MAX_FRAG / 255);
    if (num_frags2 > STARBURST_MAX_FRAG)
      num_frags2 = STARBURST_MAX_FRAG;

    for (int32_t f = 0; f < num_frags2; f++)
      star.fragment[f] += star.fragment[f] * dt_s * 2.0f;

    // Render: flash white → sparkle colour → fade
    float lifetime = std::min<float>(static_cast<float>(age) / 1500.0f, 1.0f);
    uint8_t bri = static_cast<uint8_t>((1.0f - lifetime) * 255.0f);
    uint32_t draw_col;
    if (age < 50u) {
      draw_col = 0xFFFFFFu;  // white flash
    } else {
      draw_col = color_fade(star.color, bri);
    }

    for (int32_t f = 0; f < num_frags2; f++) {
      int32_t px = static_cast<int32_t>(star.pos) + static_cast<int32_t>(star.fragment[f]);
      if (px >= 0 && px < SEGLEN)
        ctx.set_pixel(px, draw_col);
    }
  }
}

// Shared Spark struct used by Fireworks 1D, Popcorn, Drip
struct Spark1D {
  float pos;
  float vel;
  uint16_t col;
  uint8_t col_index;
};

// 117 — Fireworks 1D (gravity-sim rocket + explosion)
// Source: WLED mode_fireworks1D (FX.cpp)
// AUX0 = state: 0=init flare, 1=rising, 2=ready, 3=exploding, >=4=countdown reset
void fx_fireworks_1d(EffectContext &ctx) {
  if (SEGLEN <= 4) {
    ctx.fill_black();
    return;
  }

  int32_t max_sparks = std::min<int32_t>(640 / static_cast<int32_t>(sizeof(Spark1D)), SEGLEN / 2 + 5);
  size_t data_size = static_cast<size_t>(max_sparks) * sizeof(Spark1D) + sizeof(float);
  if (!ctx.env->allocate_data(data_size)) {
    ctx.fill_black();
    return;
  }
  auto *sparks = reinterpret_cast<Spark1D *>(ctx.env->data);
  float *dying_gravity = reinterpret_cast<float *>(ctx.env->data + static_cast<size_t>(max_sparks) * sizeof(Spark1D));

  float gravity = -0.0004f - static_cast<float>(SPEED) / 800000.0f * static_cast<float>(SEGLEN);

  ctx.fade_to_black(252u);

  if (AUX0 == 0u) {
    // Init flare
    sparks[0].pos = (INTENSITY > 128u) ? static_cast<float>(SEGLEN - 1) : 0.0f;
    sparks[0].vel = (INTENSITY > 128u) ? -gravity * static_cast<float>(SEGLEN) * 0.5f
                                       : gravity * static_cast<float>(SEGLEN) * (-0.5f);
    sparks[0].vel = fabsf(gravity * static_cast<float>(SEGLEN)) * (0.4f + static_cast<float>(hw_random8(60u)) / 200.0f);
    sparks[0].col = 345u;
    sparks[0].col_index = 0u;
    *dying_gravity = gravity;
    AUX0 = 1u;
  }

  if (AUX0 == 1u) {
    // Flare rising
    sparks[0].pos += sparks[0].vel;
    sparks[0].vel += gravity;
    if (sparks[0].vel <= 0.0f)
      AUX0 = 2u;
    int32_t px = static_cast<int32_t>(sparks[0].pos);
    if (px >= 0 && px < SEGLEN)
      ctx.set_pixel(px, 0xFFFFFFu);
    return;
  }

  if (AUX0 == 2u) {
    // Explode: create sparks
    float peak = sparks[0].pos;
    for (int32_t i = 0; i < max_sparks; i++) {
      sparks[i].pos = peak;
      sparks[i].vel =
          (static_cast<float>(hw_random8()) - 128.0f) / 128.0f * fabsf(gravity) * static_cast<float>(SEGLEN) * 0.4f;
      sparks[i].col = hw_random16(345u);
      sparks[i].col_index = hw_random8();
    }
    *dying_gravity = gravity * 0.8f;
    AUX0 = 3u;
  }

  if (AUX0 == 3u) {
    // Sparks in flight
    *dying_gravity *= 0.97f;
    bool all_dead = true;
    for (int32_t i = 0; i < max_sparks; i++) {
      sparks[i].pos += sparks[i].vel;
      sparks[i].vel += *dying_gravity;
      sparks[i].col = static_cast<uint16_t>(sparks[i].col > 4u ? sparks[i].col - 4u : 0u);
      if (sparks[i].col > 0u)
        all_dead = false;
      int32_t px = static_cast<int32_t>(sparks[i].pos);
      if (px >= 0 && px < SEGLEN) {
        uint32_t col;
        if (sparks[i].col > 300u)
          col = 0xFFFFFFu;
        else if (sparks[i].col > 45u)
          col = ctx.pal_color(sparks[i].col_index);
        else
          col = color_fade(ctx.pal_color(sparks[i].col_index), static_cast<uint8_t>(sparks[i].col * 5u));
        ctx.set_pixel(px, col);
      }
    }
    if (all_dead) {
      AUX0 = 4u;
      STEP = NOW;
    }
    return;
  }

  if (AUX0 >= 4u) {
    // Wait then reset
    if (NOW - STEP > 500u)
      AUX0 = 0u;
  }
}

// 118 — Popcorn (kernels pop up with random velocities)
// Source: WLED mode_popcorn (FX.cpp)
void fx_popcorn(EffectContext &ctx) {
  static constexpr int32_t MAX_POPCORN = 21;
  size_t data_size = static_cast<size_t>(MAX_POPCORN) * sizeof(Spark1D);
  if (!ctx.env->allocate_data(data_size)) {
    ctx.fill(COLOR0);
    return;
  }
  auto *kernels = reinterpret_cast<Spark1D *>(ctx.env->data);

  float gravity = -0.0001f - static_cast<float>(SPEED) / 200000.0f * static_cast<float>(SEGLEN);
  int32_t num_popcorn = std::max<int32_t>(1, static_cast<int32_t>(INTENSITY) * MAX_POPCORN / 255);

  // Background
  if (!ctx.params->check2) {
    uint32_t bg = COLOR1 ? COLOR1 : 0u;
    ctx.fill(bg);
  }

  for (int32_t i = 0; i < num_popcorn; i++) {
    // Inactive: maybe pop
    if (kernels[i].pos <= 0.0f && kernels[i].vel == 0.0f) {
      if (hw_random8(2u) == 0u) {
        float peak =
            0.4f + static_cast<float>(hw_random8(static_cast<uint8_t>(SEGLEN))) / static_cast<float>(SEGLEN) * 0.6f;
        kernels[i].vel = sqrtf(-2.0f * gravity * peak * static_cast<float>(SEGLEN));
        kernels[i].pos = 0.0f;
        kernels[i].col_index = hw_random8();
      }
      continue;
    }

    // Physics
    kernels[i].pos += kernels[i].vel;
    kernels[i].vel += gravity;
    if (kernels[i].pos <= 0.0f) {
      kernels[i].pos = 0.0f;
      kernels[i].vel = 0.0f;
    }

    int32_t px = static_cast<int32_t>(kernels[i].pos);
    if (px >= 0 && px < SEGLEN) {
      uint32_t col;
      if (ctx.params->palette_id != 0) {
        col = ctx.pal_color(kernels[i].col_index);
      } else {
        col = ctx.params->colors[static_cast<size_t>(i) % 3u];
      }
      ctx.set_pixel(px, col);
    }
  }
}

// 119 — Drip (drips fall from top, bounce at bottom)
// Source: WLED mode_drip (FX.cpp)
void fx_drip(EffectContext &ctx) {
  static constexpr int32_t MAX_DROPS = 4;
  size_t data_size = static_cast<size_t>(MAX_DROPS) * sizeof(Spark1D);
  if (!ctx.env->allocate_data(data_size)) {
    ctx.fill(COLOR0);
    return;
  }
  auto *drops = reinterpret_cast<Spark1D *>(ctx.env->data);

  float gravity =
      -0.0005f - static_cast<float>(SPEED) / 50000.0f * static_cast<float>(std::max<int32_t>(1, SEGLEN - 1));
  int32_t num_drops = std::min<int32_t>(MAX_DROPS, 1 + static_cast<int32_t>(INTENSITY >> 6u));

  if (!ctx.params->check2)
    ctx.fill(COLOR1);

  // Source glow at top
  static constexpr uint8_t SOURCE_DROP = 12u;
  ctx.set_pixel(SEGLEN - 1, color_fade(COLOR0, SOURCE_DROP));

  for (int32_t i = 0; i < num_drops; i++) {
    uint8_t state = drops[i].col_index;

    if (state == 0u) {
      // Init: start at top, forming
      drops[i].pos = static_cast<float>(SEGLEN - 1);
      drops[i].vel = 0.0f;
      drops[i].col_index = 1u;
    }

    if (drops[i].col_index == 1u) {
      // Forming: swell brightness, random drop trigger
      uint8_t bri = static_cast<uint8_t>(drops[i].col >> 8u);
      bri = static_cast<uint8_t>(std::min<uint16_t>(255u, static_cast<uint16_t>(bri) + 8u));
      drops[i].col = static_cast<uint16_t>(bri << 8u);
      int32_t px = static_cast<int32_t>(drops[i].pos);
      if (px >= 0 && px < SEGLEN)
        ctx.set_pixel(px, color_fade(COLOR0, bri));
      if (bri >= 255u && hw_random8(10u) == 0u) {
        drops[i].vel = 0.0f;
        drops[i].col_index = 2u;  // start falling
      }
      continue;
    }

    if (drops[i].col_index == 2u || drops[i].col_index == 5u) {
      // Falling / bouncing
      drops[i].pos += drops[i].vel;
      drops[i].vel += gravity;

      // Trail
      for (int32_t t = 1; t <= 3; t++) {
        int32_t trail_px = static_cast<int32_t>(drops[i].pos) + t;
        if (trail_px >= 0 && trail_px < SEGLEN)
          ctx.set_pixel(trail_px, color_fade(COLOR0, static_cast<uint8_t>(80u >> t)));
      }

      int32_t px = static_cast<int32_t>(drops[i].pos);
      if (px >= 0 && px < SEGLEN)
        ctx.set_pixel(px, COLOR0);

      // Bottom bounce
      if (drops[i].pos <= 0.0f) {
        drops[i].pos = 0.0f;
        if (drops[i].col_index == 2u) {
          drops[i].vel = -drops[i].vel / 4.0f;
          drops[i].col_index = 5u;
        } else {
          drops[i].vel = -drops[i].vel / 4.0f;
          if (fabsf(drops[i].vel) < 0.01f)
            drops[i].col_index = 0u;  // reset
        }
      }
    }
  }
}

// 120 — Dancing Shadows (moving spotlight blobs)
// Source: WLED mode_dancing_shadows (FX.cpp)
void fx_dancing_shadows(EffectContext &ctx) {
  struct SpotLight {
    float speed;
    uint8_t color_idx;
    int16_t position;
    uint32_t last_update;
    uint8_t width;
    uint8_t type;
  };

  int32_t num_spots = std::max<int32_t>(2, std::min<int32_t>(16, static_cast<int32_t>(INTENSITY) * 16 / 255 + 2));
  size_t data_size = static_cast<size_t>(num_spots) * sizeof(SpotLight);
  if (!ctx.env->allocate_data(data_size)) {
    ctx.fill_black();
    return;
  }
  auto *spots = reinterpret_cast<SpotLight *>(ctx.env->data);

  if (CALL == 0u) {
    for (int32_t i = 0; i < num_spots; i++) {
      spots[i].position = static_cast<int16_t>(hw_random16(static_cast<uint16_t>(SEGLEN)));
      spots[i].speed = (1.0f + static_cast<float>(hw_random8(20u))) / 20.0f;
      spots[i].color_idx = hw_random8();
      spots[i].width = 1u + hw_random8(static_cast<uint8_t>(std::max<int32_t>(1, SEGLEN / 8)));
      spots[i].type = hw_random8(6u);
      spots[i].last_update = NOW;
    }
  }

  ctx.fill_black();

  auto blend_pix = [&](int32_t i, uint32_t col, uint8_t alpha) {
    int32_t base = ctx.map_pixel(i);
    if (base < 0 || base >= static_cast<int32_t>(ctx.frame_len))
      return;
    ctx.frame_buf[base] = color_blend(ctx.frame_buf[base], col, alpha);
  };

  for (int32_t s = 0; s < num_spots; s++) {
    SpotLight &spot = spots[s];
    uint32_t dt = NOW - spot.last_update;
    spot.last_update = NOW;

    float move = spot.speed * static_cast<float>(SPEED + 1u) / 32.0f * static_cast<float>(dt) / 16.0f;
    spot.position = static_cast<int16_t>(static_cast<int32_t>(spot.position) + static_cast<int32_t>(move));

    // Wrap around
    if (spot.position >= SEGLEN) {
      spot.position = 0;
      spot.color_idx = hw_random8();
      spot.type = hw_random8(6u);
      spot.width = 1u + hw_random8(static_cast<uint8_t>(std::max<int32_t>(1, SEGLEN / 8)));
    } else if (spot.position < 0) {
      spot.position = static_cast<int16_t>(SEGLEN - 1);
    }

    uint32_t col = ctx.pal_color(spot.color_idx);
    int32_t center = static_cast<int32_t>(spot.position);
    int32_t half_w = static_cast<int32_t>(spot.width);

    for (int32_t d = -half_w; d <= half_w; d++) {
      int32_t px = center + d;
      if (px < 0 || px >= SEGLEN)
        continue;
      uint8_t alpha;
      switch (spot.type) {
        case 0:  // solid
          alpha = 200u;
          break;
        case 1:  // gradient
          alpha = cubicwave8(
              static_cast<uint8_t>(static_cast<uint32_t>(d + half_w) * 255u / static_cast<uint32_t>(2 * half_w + 1)));
          break;
        case 2:  // double gradient
          alpha = cubicwave8(
              static_cast<uint8_t>(static_cast<uint32_t>(abs(d)) * 255u / static_cast<uint32_t>(half_w + 1)));
          break;
        case 3:  // 2x dot
          alpha = (abs(d) <= 1) ? 200u : 40u;
          break;
        case 4:  // 3x dot
          alpha = (abs(d) == 0) ? 200u : (abs(d) <= 2 ? 80u : 20u);
          break;
        default:  // 4x dot
          alpha = (abs(d) == 0) ? 200u : (abs(d) <= 3 ? 60u : 15u);
          break;
      }
      blend_pix(px, col, alpha);
    }
  }
}

// 121 — TV Simulator (flickering screen-like color wash)
// Source: WLED mode_tv_simulator (FX.cpp)
void fx_tv_simulator(EffectContext &ctx) {
  struct TvState {
    uint32_t total_time;
    uint32_t fade_time;
    uint32_t start_time;
    uint32_t scene_start;
    uint32_t scene_dur;
    uint16_t scene_hue;
    uint16_t slider_vals;
    uint8_t scene_sat;
    uint8_t scene_bri;
    uint8_t r, g, b;
    uint16_t pr, pg, pb;
  };

  if (!ctx.env->allocate_data(sizeof(TvState))) {
    ctx.fill(COLOR0);
    return;
  }
  auto *tv = reinterpret_cast<TvState *>(ctx.env->data);

  uint8_t color_speed = static_cast<uint8_t>(1u + static_cast<uint32_t>(SPEED) * 20u / 255u);

  if (CALL == 0u) {
    tv->scene_hue = hw_random16();
    tv->scene_sat = hw_random8(150u) + 100u;
    tv->scene_bri = hw_random8(100u) + 100u;
    // scene_dur range: 60*250*cs .. 60*750*cs ms — may exceed uint16, use esp_random
    uint32_t dur_lo = 60u * 250u * color_speed;
    uint32_t dur_range = 60u * 500u * color_speed;
    tv->scene_dur = dur_lo + (esp_random() % (dur_range + 1u));
    tv->scene_start = NOW;
    tv->start_time = NOW;
    tv->fade_time = static_cast<uint32_t>(hw_random16(2000u) + 1000u);
    tv->total_time = static_cast<uint32_t>(hw_random16(4000u) + 1000u);
    tv->slider_vals = 0u;
    tv->r = 0u;
    tv->g = 0u;
    tv->b = 0u;
    tv->pr = 0u;
    tv->pg = 0u;
    tv->pb = 0u;
  }

  // New scene?
  if (NOW - tv->scene_start > tv->scene_dur) {
    tv->pr = static_cast<uint16_t>(tv->r);
    tv->pg = static_cast<uint16_t>(tv->g);
    tv->pb = static_cast<uint16_t>(tv->b);
    tv->scene_hue = static_cast<uint16_t>(tv->scene_hue + hw_random16(4000u) + 1000u);
    tv->scene_sat = static_cast<uint8_t>(hw_random8(150u) + 100u);
    tv->scene_bri = static_cast<uint8_t>(hw_random8(100u) + 100u);
    uint32_t dur_lo2 = 60u * 250u * color_speed;
    uint32_t dur_range2 = 60u * 500u * color_speed;
    tv->scene_dur = dur_lo2 + (esp_random() % (dur_range2 + 1u));
    tv->scene_start = NOW;
    tv->start_time = NOW;
    tv->fade_time = static_cast<uint32_t>(hw_random16(1500u) + 500u);
    tv->total_time = tv->fade_time + static_cast<uint32_t>(hw_random16(2500u) + 500u);
  }

  // Convert HSV scene colour to RGB (simple hue-based)
  uint16_t h = tv->scene_hue >> 8u;
  uint8_t region = static_cast<uint8_t>(h / 43u);
  uint8_t rem = static_cast<uint8_t>((h - region * 43u) * 6u);
  uint8_t p = static_cast<uint8_t>(static_cast<uint16_t>(tv->scene_bri) * (255u - tv->scene_sat) >> 8u);
  uint8_t q = static_cast<uint8_t>(
      static_cast<uint16_t>(tv->scene_bri) * (255u - (static_cast<uint16_t>(tv->scene_sat) * rem >> 8u)) >> 8u);
  uint8_t t_val = static_cast<uint8_t>(static_cast<uint16_t>(tv->scene_bri) *
                                           (255u - (static_cast<uint16_t>(tv->scene_sat) * (255u - rem) >> 8u)) >>
                                       8u);
  uint8_t target_r, target_g, target_b;
  switch (region % 6u) {
    case 0:
      target_r = tv->scene_bri;
      target_g = t_val;
      target_b = p;
      break;
    case 1:
      target_r = q;
      target_g = tv->scene_bri;
      target_b = p;
      break;
    case 2:
      target_r = p;
      target_g = tv->scene_bri;
      target_b = t_val;
      break;
    case 3:
      target_r = p;
      target_g = q;
      target_b = tv->scene_bri;
      break;
    case 4:
      target_r = t_val;
      target_g = p;
      target_b = tv->scene_bri;
      break;
    default:
      target_r = tv->scene_bri;
      target_g = p;
      target_b = q;
      break;
  }

  // Blend toward target during fade_time
  uint32_t elapsed = NOW - tv->start_time;
  if (elapsed < tv->fade_time && tv->fade_time > 0u) {
    uint8_t blend = static_cast<uint8_t>((elapsed * 255u) / tv->fade_time);
    tv->r = static_cast<uint8_t>(lerp8by8(static_cast<uint8_t>(tv->pr & 0xFFu), target_r, blend));
    tv->g = static_cast<uint8_t>(lerp8by8(static_cast<uint8_t>(tv->pg & 0xFFu), target_g, blend));
    tv->b = static_cast<uint8_t>(lerp8by8(static_cast<uint8_t>(tv->pb & 0xFFu), target_b, blend));
  } else {
    tv->r = target_r;
    tv->g = target_g;
    tv->b = target_b;
  }

  // Slight brightness flicker for realism
  uint8_t flicker = beatsin8(static_cast<uint8_t>(120u + INTENSITY / 4u), 200u, 255u, NOW);
  uint32_t draw_col = RGBW32(static_cast<uint8_t>(static_cast<uint16_t>(tv->r) * flicker >> 8u),
                             static_cast<uint8_t>(static_cast<uint16_t>(tv->g) * flicker >> 8u),
                             static_cast<uint8_t>(static_cast<uint16_t>(tv->b) * flicker >> 8u), 0u);
  ctx.fill(draw_col);
}

// Append new rows to the effect table
const EffectDescriptor WLED_EFFECTS[WLED_EFFECT_COUNT] = {
    /* 00 */ {"Solid", "Solid", fx_solid},
    /* 01 */ {"Blink", "Blink@!,Duty cycle;!,!;!;01", fx_blink},
    /* 02 */ {"Breathe", "Breathe@!,;!;!", fx_breathe},
    /* 03 */ {"Color Wipe", "Color Wipe@!,!;!,!;!", fx_color_wipe},
    /* 04 */ {"Color Wipe Random", "Color Wipe Random@!,!;;!", fx_color_wipe_random},
    /* 05 */ {"Random Color", "Random Color@!,;!;!", fx_random_color},
    /* 06 */ {"Color Sweep", "Color Sweep@!,!;!,!;!", fx_color_sweep},
    /* 07 */ {"Dynamic", "Dynamic@!,!;;!", fx_dynamic},
    /* 08 */ {"Colorloop", "Colorloop@!,Saturation;;!;01", fx_colorloop},
    /* 09 */ {"Rainbow", "Rainbow@!,Size;;!", fx_rainbow_cycle},
    /* 10 */ {"Scan", "Scan@!,# of dots,,,,,Overlay;!,!,!;!", fx_scan},
    /* 11 */ {"Scan Dual", "Scan Dual@!,# of dots,,,,,Overlay;!,!,!;!", fx_dual_scan},
    /* 12 */ {"Fade", "Fade@!,;!,!;!", fx_fade},
    /* 13 */ {"Theater Chase", "Theater Chase@!,Gap size;!,!;!", fx_theater_chase},
    /* 14 */ {"Theater Chase Rainbow", "Theater Chase Rainbow@!,Gap size;!,!;!", fx_theater_chase_rainbow},
    /* 15 */ {"Running", "Running@!,Wave width;!,!;!", fx_running_lights},
    /* 16 */ {"Saw", "Saw@!,Width;!,!;!", fx_saw},
    /* 17 */ {"Twinkle", "Twinkle@!,!;!;!", fx_twinkle},
    /* 18 */ {"Twinkle Random", "Twinkle Random@!,!;;!", fx_twinkle_random},
    /* 19 */ {"Strobe", "Strobe@!;!,!;!;01", fx_strobe},
    /* 20 */ {"Strobe Rainbow", "Strobe Rainbow@!;,!;!;01", fx_strobe_rainbow},
    /* 21 */ {"BPM", "BPM@!,;!;!", fx_bpm},
    /* 22 */ {"Larson Scanner", "Larson Scanner@!,Fade rate;!,!;!", fx_larson_scanner},
    /* 23 */ {"Comet", "Comet@!,Fade rate;!,!;!", fx_comet},
    /* 24 */ {"Fire 2012", "Fire 2012@Cooling,Sparking;!;01", fx_fire_2012},
    /* 25 */ {"Meteor", "Meteor@!,Trail length;!;!", fx_meteor},
    /* 26 */ {"Meteor Smooth", "Meteor Smooth@!,Trail length;!;!", fx_meteor_smooth},
    /* 27 */ {"Noise 1D", "Noise 1D@!,Scale;!;!", fx_noise1d},
    /* 28 */ {"Palette", "Palette@Shift,Size;!;!", fx_palette},
    /* 29 */ {"Ripple", "Ripple@!,;!;!", fx_ripple},
    /* 30 */ {"Juggle", "Juggle@!,Trail;!,!;!", fx_juggle},
    /* 31 */ {"Bouncing Balls", "Bouncing Balls@Gravity,# of balls;!,!,!;!;01", fx_bouncing_balls},
    /* 32 */ {"Fireworks", "Fireworks@Gravity,Firing side;!,!;!;01", fx_fireworks},
    /* 33 */ {"Police", "Police@!,Width;;!;01", fx_police},
    /* 34 */ {"Chase Flash", "Chase Flash@!,!;!,!;!", fx_chase_flash},
    /* 35 */ {"Heartbeat", "Heartbeat@!,!;!;!;01", fx_heartbeat},
    /* 36 */ {"Rain", "Rain@!,Spawning rate;!,!;!;01", fx_rain},
    /* 37 */ {"Sparkle", "Sparkle@!,;!;!;01", fx_sparkle},
    /* 38 */ {"Pride 2015", "Pride 2015@!,;;", fx_pride_2015},
    /* 39 */ {"Candle", "Candle@!,Flicker;!;0;01", fx_candle},
    /* 40 */ {"Fill Noise", "Fill Noise@!,Scale;!;!", fx_fill_noise},
    /* 41 */ {"Oscillate", "Oscillate@!,;,!;!", fx_oscillate},
    /* 42 */ {"Gradient", "Gradient@!,!;!,!;!", fx_gradient},
    /* 43 */ {"Pacifica", "Pacifica@!,;,,;!", fx_pacifica},
    /* 44 */ {"Sinelon", "Sinelon@!,Trail;!,!,!;!", fx_sinelon},
    /* 45 */ {"Dissolve", "Dissolve@Repeat speed,Dissolve speed,,,,Random,Complete;!,!;!", fx_dissolve},
    /* 46 */ {"Android", "Android@!,Width;!,!;!;;m12=1", fx_android},
    /* 47 */ {"Chase Rainbow", "Chase Rainbow@!,Width;!,!;!", fx_chase_rainbow},
    /* 48 */ {"Colorful", "Colorful@!,Saturation;1,2,3;!", fx_colorful},
    /* 49 */ {"Fire Flicker", "Fire Flicker@!,!;!;!;01", fx_fire_flicker},
    /* 50 */ {"Two Dots", "Two Dots@!,Dot size,,,,,Overlay;1,2,Bg;!", fx_two_dots},
    /* 51 */ {"Chase 3", "Chase 3@!,Size;1,2,3;!", fx_tricolor_chase},
    /* 52 */ {"ICU", "ICU@!,!,,,,,Overlay;!,!;!", fx_icu},
    /* 53 */ {"Lightning", "Lightning@!,!,,,,,Overlay;!,!;!", fx_lightning},
    /* 54 */ {"Glitter", "Glitter@!,!,,,,,Overlay;,,Glitter color;!;;pal=11,m12=0", fx_glitter},
    /* 55 */ {"Solid Glitter", "Solid Glitter@,!;Bg,,Glitter color;;;m12=0", fx_solid_glitter},
    /* 56 */ {"Spots", "Spots@Spread,Width,,,,,Overlay;!,!;!", fx_spots},
    /* 57 */ {"Percent", "Percent@!,% of fill,,,,One color;!,!;!", fx_percent},
    /* 58 */ {"Flow", "Flow@!,Zones;;!;;m12=1", fx_flow},
    /* 59 */ {"Phased", "Phased@!,!;!,!;!", fx_phased},
    /* 60 */ {"Sine", "Sine@!,Scale;;!", fx_sinewave},
    /* 61 */ {"Twinkleup", "Twinkleup@!,Intensity;!,!;!;;m12=0", fx_twinkleup},
    /* 62 */ {"Colorwaves", "Colorwaves@!,Hue;!;!;;pal=26", fx_colorwaves},
    /* 63 */ {"Chunchun", "Chunchun@!,Gap size;!,!;!", fx_chunchun},
    /* 64 */ {"Stream", "Stream@!,Zone size;;!", fx_stream},
    /* 65 */ {"Scanner Dual", "Scanner Dual@!,Trail,Delay,,,Dual,Bi-delay;!,!,!;!;;m12=0,c1=0", fx_scanner_dual},
    /* 66 */ {"Fairy", "Fairy@!,# of flashers;!,!;!", fx_fairy},
    /* 67 */ {"Fairytwinkle", "Fairytwinkle@!,!;!,!;!;;m12=0", fx_fairytwinkle},
    /* 68 */ {"Tri Wipe", "Tri Wipe@!;1,2,3;!", fx_tri_wipe},
    /* 69 */ {"Tri Fade", "Tri Fade@!;1,2,3;!", fx_tri_fade},
    /* 70 */ {"Multi Comet", "Multi Comet@!,Fade;!,!;!;1", fx_multi_comet},
    /* 71 */ {"Colortwinkles", "Colortwinkles@Fade speed,Spawn speed;;!;;m12=0", fx_colortwinkles},
    /* 72 */ {"Lake", "Lake@!;Fx;!", fx_lake},
    /* 73 */ {"Railway", "Railway@!,Smoothness;1,2;!;;pal=3", fx_railway},
    /* 74 */ {"Halloween Eyes", "Halloween Eyes@Eye off time,Eye on time,,,,,Overlay;!,!;!;12", fx_halloween_eyes},
    /* 75 */ {"Plasma", "Plasma@Phase,!;!;!", fx_plasma},
    /* 76 */ {"Sunrise", "Sunrise@Time [min],Width;;!;;pal=35,sx=60", fx_sunrise},
    /* 77 */ {"Washing Machine", "Washing Machine@!,!;;!", fx_washing_machine},
    /* 78 */ {"Blends", "Blends@Shift speed,Blend speed;;!", fx_blends},
    /* 79 */ {"Strobe Mega", "Strobe Mega@!,!;!,!;!;01", fx_strobe_mega},
    /* 80 */ {"Chase", "Chase@!,Width;!,!,!;!", fx_chase_color},
    /* 81 */ {"Chase Random", "Chase Random@!,Width;!,,!;!", fx_chase_color_random},
    /* 82 */ {"Solid Pattern Tri", "Solid Pattern Tri@,Size;1,2,3;;;pal=0", fx_solid_pattern_tri},
    /* 83 */ {"Solid Pattern", "Solid Pattern@Fg size,Bg size;Fg,!;!;;pal=0", fx_solid_pattern},
    /* 84 */ {"Blink Rainbow", "Blink Rainbow@Frequency,Blink duration;!,!;!;01", fx_blink_rainbow},
    /* 85 */ {"Dynamic Smooth", "Dynamic Smooth@!,!;;!", fx_dynamic_smooth},
    /* 86 */ {"Running Dual", "Running Dual@!,Wave width;L,!,R;!", fx_running_dual},
    /* 87 */ {"Traffic Light", "Traffic Light@!,US style;,!;!", fx_traffic_light},
    /* 88 */ {"Loading", "Loading@!,Fade;!,!;!;;ix=16", fx_loading},
    /* 89 */ {"Stream 2", "Stream 2@!;;", fx_stream2},
    /* 90 */ {"Spots Fade", "Spots Fade@Spread,Width,,,,,Overlay;!,!;!", fx_spots_fade},
    /* 91 */ {"Sinelon Dual", "Sinelon Dual@!,Trail;!,!,!;!", fx_sinelon_dual},
    /* 92 */ {"Sinelon Rainbow", "Sinelon Rainbow@!,Trail;,,!;!", fx_sinelon_rainbow},
    /* 93 */ {"Ripple Rainbow", "Ripple Rainbow@!,Wave #;;!;12", fx_ripple_rainbow},
    /* 94 */ {"Candle Multi", "Candle Multi@!,!;!,!;!;;sx=96,ix=224,pal=0", fx_candle_multi},
    /* 95 */ {"Noise Pal", "Noise Pal@!,Scale;;!", fx_noise_pal},
    /* 96 */ {"Twinklefox", "Twinklefox@!,Twinkle rate,,,,Cool;!,!;!", fx_twinklefox},
    /* 97 */ {"Flow Stripe", "Flow Stripe@Hue speed,Effect speed;;!;pal=11", fx_flow_stripe},
    /* 98 */ {"Twinklecat", "Twinklecat@!,Twinkle rate,,,,Cool,Reverse;!,!;!", fx_twinklecat},
    /* 99 */ {"Dissolve Rnd", "Dissolve Rnd@Repeat speed,Dissolve speed;,!;!", fx_dissolve_random},
    /* 100 */ {"Sweep Random", "Sweep Random@!;;!", fx_sweep_random},
    /* 101 */ {"Chase Flash Rnd", "Chase Flash Rnd@!;!,!;!", fx_chase_flash_random},
    /* 102 */ {"Sparkle Dark", "Sparkle Dark@!,!,,,,,Overlay;Bg,Fx;!;;m12=0", fx_sparkle_dark},
    /* 103 */ {"Sparkle+", "Sparkle+@!,!,,,,,Overlay;Bg,Fx;!;;m12=0", fx_sparkle_plus},
    /* 104 */ {"Chase 2", "Chase 2@!,Width;!,!;!", fx_chase2},
    /* 105 */ {"Rainbow Runner", "Rainbow Runner@!,Size;Bg;!", fx_rainbow_runner},
    /* 106 */ {"Noise 1", "Noise 1@!;!;!;;pal=20", fx_noise1},
    /* 107 */ {"Noise 2", "Noise 2@!;!;!;;pal=43", fx_noise2},
    /* 108 */ {"Noise 3", "Noise 3@!;!;!;;pal=35", fx_noise3},
    /* 109 */ {"Noise 4", "Noise 4@!;!;!;;pal=26", fx_noise4},
    /* 110 */ {"Phased Noise", "Phased Noise@!,!;!,!;!", fx_phased_noise},
    /* 111 */
    {"Wavesins", "Wavesins@!,Brightness variation,Starting color,Range of colors,Color variation;!;!", fx_wavesins},
    /* 112 */
    {"Shimmer",
     "Shimmer@Speed,Interval,Size,Granular,Flow,Zebra,Reverse,Sporadic;Fx,Bg,Cx;!;1;pal=15,sx=220,ix=10,c2=0,c3=0",
     fx_shimmer},
    /* 113 */ {"Aurora", "Aurora@!,!;1,2,3;!;;sx=24,pal=50", fx_aurora},
    /* 114 */ {"Tetrix", "Tetrix@!,Width,,,,One color;!,!;!;;sx=0,ix=0,pal=11,m12=1", fx_tetrix},
    /* 115 */
    {"Rolling Balls", "Rolling Balls@!,# of balls,,,,Collide,Overlay,Trails;!,!,!;!;1;m12=1", fx_rolling_balls},
    /* 116 */
    {"Fireworks Starburst", "Fireworks Starburst@Chance,Fragments,,,,,Overlay;,!;!;;pal=11,m12=0", fx_starburst},
    /* 117 */ {"Fireworks 1D", "Fireworks 1D@Gravity,Firing side;!,!;!;12;pal=11,ix=128", fx_fireworks_1d},
    /* 118 */ {"Popcorn", "Popcorn@!,!,,,,,Overlay;!,!,!;!;;m12=1", fx_popcorn},
    /* 119 */ {"Drip", "Drip@Gravity,# of drips,,,,,Overlay;!,!;!;;m12=1", fx_drip},
    /* 120 */ {"Dancing Shadows", "Dancing Shadows@!,# of shadows;!;!", fx_dancing_shadows},
    /* 121 */ {"TV Simulator", "TV Simulator@!,!;;!;01", fx_tv_simulator},
    /* 122 */ {"2D Matrix", "2D Matrix@!,Brightness;!,Spawn,Trail;!;;pal=11,m12=2", fx_2d_matrix},
    /* 123 */ {"2D Plasma", "2D Plasma@Phase,Scale;!;!;;m12=2", fx_2d_plasma},
    /* 124 */ {"2D Gameoflife", "2D Gameoflife@!;!;!;;m12=2", fx_2d_gameoflife},
    /* 125 */ {"2D Tartan", "2D Tartan@!,Scale;!;!;;m12=2", fx_2d_tartan},
    /* 126 */ {"2D Waverly", "2D Waverly@!,Amplitude;!;!;;m12=2", fx_2d_waverly},
    /* 127 */ {"2D Noise", "2D Noise@!,Scale;!;!;;m12=2", fx_2d_noise},
    /* 128 */ {"2D Sindots", "2D Sindots@!,# dots;!;!;;m12=2", fx_2d_sindots},
    /* 129 */ {"2D Lissajous", "2D Lissajous@!,Ratio;!;!;;m12=2", fx_2d_lissajous},
    /* 130 */ {"2D Frizzles", "2D Frizzles@!,;!;!;;m12=2", fx_2d_frizzles},
    /* 131 */ {"2D BlackHole", "2D BlackHole@!,Swirl;!;!;;m12=2", fx_2d_blackhole},
    /* 132 */ {"2D SineSin", "2D SineSin@!,Scale;!;!;;m12=2", fx_2d_sinesin},
    /* 133 */ {"2D Julia", "2D Julia@!,Iterations;!;!;;m12=2", fx_2d_julia},
    /* 134 */ {"2D Metaballs", "2D Metaballs@!,;!;!;;m12=2", fx_2d_metaballs},
    /* 135 */ {"2D Fire", "2D Fire@Cooling,Sparking;,!;!;;m12=2", fx_2d_fire},
    /* 136 */ {"2D Gravity", "2D Gravity@!,Spawn;!;!;;m12=2", fx_2d_gravity},
    /* 137 */ {"2D DNA", "2D DNA@!,;!;!;;m12=2", fx_2d_dna},
    /* 138 */ {"2D Pulser", "2D Pulser@!,Rings;!;!;;m12=2", fx_2d_pulser},
    /* 139 */ {"2D Drift", "2D Drift@!,Scale;!;!;;m12=2", fx_2d_drift},
    /* 140 */ {"2D NoiseFire", "2D NoiseFire@Cooling,Sparking;!;!;;m12=2", fx_2d_noisefire},
    /* 141 */ {"2D NoiseMove", "2D NoiseMove@!,Scale;!;!;;m12=2", fx_2d_noisemove},
    /* 142 */ {"2D ColoredBursts", "2D ColoredBursts@!,# rays;!;!;;m12=2", fx_2d_colored_bursts},
    /* 143 */ {"2D YinYang", "2D YinYang@!,;!,!;!;;m12=2", fx_2d_yinyang},
    /* 144 */ {"2D Fireworks", "2D Fireworks@!,Trails;!;!;;m12=2", fx_2d_fireworks2d},
    /* 145 */ {"2D Sprouts", "2D Sprouts@!,Spread;!;!;;m12=2", fx_2d_sprouts},
    /* 146 */ {"2D Zentangle", "2D Zentangle@!,Tile;!;!;;m12=2", fx_2d_zentangle},
    /* 147 */ {"2D Akemi", "2D Akemi@!,;!,!;!;;m12=2", fx_2d_akemi},
    /* 148 */ {"2D Hopalong", "2D Hopalong@!,;!;!;;m12=2", fx_2d_hopalong},
    /* 149 */ {"2D Magnetics", "2D Magnetics@!,# lines;!;!;;m12=2", fx_2d_magnetics},
    /* 150 */ {"2D Hypnotic", "2D Hypnotic@!,Spirals;!;!;;m12=2", fx_2d_hypnotic},
    /* 151 */ {"2D Bubbles", "2D Bubbles@!,;!;!;;m12=2", fx_2d_bubbles},
    /* 152 */ {"2D Magnifying", "2D Magnifying@!,Zoom;!;!;;m12=2", fx_2d_magnifying},
    /* 153 */ {"2D PopCorn", "2D PopCorn@!,!,,,,,Overlay;!,!,!;!;;m12=2", fx_2d_popcorn2d},
    /* 154 */ {"2D Sparkle", "2D Sparkle@!,!,,,,,Overlay;!;!;;m12=2", fx_2d_sparkle2d},
    /* 155 */ {"2D Pendulum", "2D Pendulum@!,Trail;!;!;;m12=2", fx_2d_pendulum},
    /* 156 */ {"2D Heatmap", "2D Heatmap@!,;!;!;;m12=2", fx_2d_heatmap},
    /* 157 */ {"2D SquaredSwirl", "2D SquaredSwirl@!,Swirl;!;!;;m12=2", fx_2d_squaredswirl},
    /* 158 */ {"2D SunRadiation", "2D SunRadiation@!,# rays;!;!;;m12=2", fx_2d_sunradiation},
    /* 159 */ {"2D PolarLights", "2D PolarLights@!,Scale;!;!;;m12=2", fx_2d_polarlights},
    /* 160 */ {"2D Swirl", "2D Swirl@!,Scale;!;!;;m12=2", fx_2d_swirl},
    /* 161 */ {"2D DNA Spiral", "2D DNA Spiral@!,Wavelength;!;!;;m12=2", fx_2d_dnaspiral},
    /* 162 */ {"2D Plasma Ball", "2D Plasma Ball@!,# arms;!;!;;m12=2", fx_2d_plasmaball},
    /* 163 */ {"2D Funky Plank", "2D Funky Plank@!,# cols;!;!;;m12=2", fx_2d_funkyplank},
    /* 164 */ {"2D PlasmaRotozoom", "2D PlasmaRotozoom@!,Scale;!;!;;m12=2", fx_2d_plasmarotozoom},
    /* 165 */ {"2D Distortion Waves", "2D Distortion Waves@!,Offset;!;!;;m12=2", fx_2d_distortionwaves},
    /* 166 */ {"2D Soap", "2D Soap@!,Scale;!;!;;m12=2", fx_2d_soap},
};

}  // namespace wled_bridge
}  // namespace esphome
