// WLED Effect Engine — 30 1D effects ported from WLED v16.0.0
// Algorithms are derived from WLED's FX.cpp (EUPL-1.2 original).
// This implementation is an independent rewrite under ESPHome's MIT license.
// Reference: WLED/wled00/FX.cpp, FX_fcn.cpp (tag v16.0.0)
#include "wled_effects.h"
#include "wled_fx_math.h"
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
// Effect table
// ============================================================
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
};

}  // namespace wled_bridge
}  // namespace esphome
