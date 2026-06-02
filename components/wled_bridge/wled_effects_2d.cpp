// 2D matrix effects for wled_bridge.
// All effects guard with ctx.is_2d() and fall back to a solid fill when the
// bridge is configured without matrix_width / matrix_height.
#include "wled_effects.h"
#include "wled_effect_context.h"
#include "wled_color.h"
#include "wled_fx_math.h"
#include "wled_types.h"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace esphome {
namespace wled_bridge {

// ============================================================
// 1. 2D Matrix — digital rain
// ============================================================
// env->data layout: 4 bytes per column: [head, speed_mask, hue, timer]
void fx_2d_matrix(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  size_t data_size = static_cast<size_t>(w) * 4;
  if (!ctx.env->allocate_data(data_size))
    return;

  uint8_t *head = ctx.env->data;
  uint8_t *smask = ctx.env->data + w;
  uint8_t *hue = ctx.env->data + 2 * w;
  uint8_t *timer = ctx.env->data + 3 * w;

  uint8_t speed = ctx.params->speed;
  uint8_t intensity = ctx.params->intensity;

  // Fade all pixels
  ctx.fade_2d(200 - scale8(intensity, 140));

  // Update and draw columns
  for (uint16_t x = 0; x < w; x++) {
    // Throttle column update by timer
    if (timer[x] > 0) {
      timer[x]--;
    } else {
      timer[x] = static_cast<uint8_t>(255 - scale8(speed, 200));
      // Advance head
      head[x] = static_cast<uint8_t>((head[x] + 1) % h);
      // Occasionally restart with new hue
      if (head[x] == 0) {
        hue[x] = hw_random8();
        smask[x] = hw_random8(2, 8);  // step size 2..7
      }
    }
    // Draw bright head
    uint8_t col_hue = hue[x];
    ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(head[x]),
                     RGBW32(scale8(cos8(col_hue), 128) + 127, 255, scale8(sin8(col_hue), 128), 0));
  }
}

// ============================================================
// 2. 2D Plasma — animated plasma
// ============================================================
void fx_2d_plasma(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.pal_color(static_cast<uint8_t>(ctx.now >> 3)));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint32_t t = ctx.now;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      // Three overlapping sine waves
      uint8_t v = sin8(static_cast<uint8_t>(x * scale8(ix, 20) + (t * scale8(spd, 3) >> 4)));
      v = qadd8(v, sin8(static_cast<uint8_t>(y * scale8(ix, 20) + (t * scale8(spd, 3) >> 4) + 85)));
      v = qadd8(v, sin8(static_cast<uint8_t>((x + y) * 10 + (t * scale8(spd, 3) >> 3) + 171)));
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(v));
    }
  }
}

// ============================================================
// 3. 2D Game of Life — Conway's cellular automaton
// ============================================================
// env->data: [w*h current] [w*h next]
void fx_2d_gameoflife(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint32_t cells = static_cast<uint32_t>(w) * h;
  if (!ctx.env->allocate_data(cells * 2))
    return;

  uint8_t *cur = ctx.env->data;
  uint8_t *nxt = ctx.env->data + cells;

  // Seed on first call
  if (ctx.env->call == 0) {
    for (uint32_t i = 0; i < cells; i++)
      cur[i] = hw_random8() < 100 ? 1 : 0;
  }

  // Throttle: only evolve every N frames
  uint8_t speed = ctx.params->speed;
  bool evolve = ctx.env->call % static_cast<uint32_t>(3 + (255 - speed) / 40) == 0;

  if (evolve) {
    for (uint16_t y = 0; y < h; y++) {
      for (uint16_t x = 0; x < w; x++) {
        uint8_t neighbors = 0;
        for (int8_t dy = -1; dy <= 1; dy++) {
          for (int8_t dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0)
              continue;
            uint16_t nx = static_cast<uint16_t>((x + w + dx) % w);
            uint16_t ny = static_cast<uint16_t>((y + h + dy) % h);
            neighbors += cur[ny * w + nx] ? 1 : 0;
          }
        }
        uint8_t alive = cur[y * w + x];
        nxt[y * w + x] = (alive && (neighbors == 2 || neighbors == 3)) || (!alive && neighbors == 3) ? 1 : 0;
      }
    }
    // Check if stuck (all dead or identical) — reseed
    uint32_t alive_count = 0;
    for (uint32_t i = 0; i < cells; i++)
      alive_count += nxt[i];
    if (alive_count == 0 || alive_count == cells)
      for (uint32_t i = 0; i < cells; i++)
        nxt[i] = hw_random8() < 80 ? 1 : 0;

    memcpy(cur, nxt, cells);
  }

  // Render
  uint8_t hue_shift = static_cast<uint8_t>(ctx.now >> 5);
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      if (cur[y * w + x]) {
        uint8_t hue = static_cast<uint8_t>(x * 255 / w + y * 255 / h + hue_shift);
        ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(hue));
      } else {
        ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), 0);
      }
    }
  }
}

// ============================================================
// 4. 2D Tartan — animated plaid
// ============================================================
void fx_2d_tartan(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;
  uint32_t t = ctx.now * scale8(spd, 4) >> 5;

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      uint8_t hx = sin8(static_cast<uint8_t>(x * scale8(ix, 16) + t));
      uint8_t hy = sin8(static_cast<uint8_t>(y * scale8(ix, 16) + t + 128));
      uint8_t col_idx = static_cast<uint8_t>((static_cast<uint16_t>(hx) + hy) >> 1);
      uint8_t bri = static_cast<uint8_t>(scale8(hx, 128) + scale8(hy, 128));
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(col_idx, bri));
    }
  }
}

// ============================================================
// 5. 2D Waverly — ripple from center
// ============================================================
void fx_2d_waverly(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  // Multiple wave sources (3)
  static const uint8_t SOURCES = 3;
  // Source positions drift slowly
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      uint32_t val = 0;
      for (uint8_t s = 0; s < SOURCES; s++) {
        uint32_t t = ctx.now * scale8(spd, 5) >> 4;
        int16_t sx = static_cast<int16_t>(w / 2 + (w / 3) * sin8(static_cast<uint8_t>(t + s * 85)) / 256);
        int16_t sy = static_cast<int16_t>(h / 2 + (h / 3) * cos8(static_cast<uint8_t>(t + s * 85)) / 256);
        int16_t dx = static_cast<int16_t>(x) - sx;
        int16_t dy = static_cast<int16_t>(y) - sy;
        uint16_t dist = static_cast<uint16_t>(sqrtf(static_cast<float>(dx * dx + dy * dy)) * scale8(ix, 20));
        val += sin8(static_cast<uint8_t>(dist - (ctx.now * scale8(spd, 3) >> 3)));
      }
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y),
                       ctx.pal_color(static_cast<uint8_t>(val / SOURCES)));
    }
  }
}

// ============================================================
// 6. 2D Noise — 2D Perlin noise
// ============================================================
void fx_2d_noise(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.pal_color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;
  uint32_t zt = ctx.now * scale8(spd, 3);
  uint16_t scale = static_cast<uint16_t>(256 - scale8(ix, 200));  // noise scale

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      uint8_t noise = inoise8(static_cast<uint16_t>(x * scale + zt), static_cast<uint16_t>(y * scale));
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(noise));
    }
  }
}

// ============================================================
// 7. 2D Sindots — sine-positioned dots
// ============================================================
void fx_2d_sindots(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(0);
    ctx.set_pixel(0, ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t num = static_cast<uint8_t>(2 + scale8(ctx.params->intensity, 10));

  ctx.fill_black();

  uint32_t t = ctx.now * scale8(spd, 4) >> 4;
  for (uint8_t i = 0; i < num; i++) {
    int16_t x = static_cast<int16_t>(scale8(sin8(static_cast<uint8_t>(t + i * 40)), static_cast<uint8_t>(w - 1)));
    int16_t y =
        static_cast<int16_t>(scale8(cos8(static_cast<uint8_t>(t * 2 + i * 65 + 42)), static_cast<uint8_t>(h - 1)));
    ctx.set_pixel_2d(x, y, ctx.pal_color(static_cast<uint8_t>(i * 40 + (t >> 2))));
    // Glow neighbors
    if (x > 0)
      ctx.set_pixel_2d(static_cast<int16_t>(x - 1), y,
                       color_fade(ctx.pal_color(static_cast<uint8_t>(i * 40 + (t >> 2))), 100));
    if (y > 0)
      ctx.set_pixel_2d(x, static_cast<int16_t>(y - 1),
                       color_fade(ctx.pal_color(static_cast<uint8_t>(i * 40 + (t >> 2))), 100));
  }
}

// ============================================================
// 8. 2D Lissajous — Lissajous figures
// ============================================================
void fx_2d_lissajous(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(0);
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  ctx.fade_2d(220 - scale8(ix, 80));

  // a/b ratio encoded in intensity; phase drifts over time
  uint8_t a_freq = 3;
  uint8_t b_freq = static_cast<uint8_t>(2 + scale8(ix, 4));
  uint32_t phase = ctx.now * scale8(spd, 3) >> 4;

  uint16_t steps = static_cast<uint16_t>(w + h) * 4;
  for (uint16_t i = 0; i < steps; i++) {
    uint8_t t = static_cast<uint8_t>((static_cast<uint32_t>(i) * 255) / steps);
    int16_t x = static_cast<int16_t>(((int32_t)sin8(static_cast<uint8_t>(a_freq * t + phase)) * (w - 1)) >> 8);
    int16_t y = static_cast<int16_t>(((int32_t)sin8(static_cast<uint8_t>(b_freq * t + phase / 2)) * (h - 1)) >> 8);
    ctx.set_pixel_2d(x, y, ctx.pal_color(t));
  }
}

// ============================================================
// 9. 2D Frizzles — colorful diagonal dashes
// ============================================================
void fx_2d_frizzles(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;

  ctx.fade_2d(170);

  uint32_t t = ctx.now * scale8(spd, 5) >> 4;
  uint8_t num_lines = 6;
  for (uint8_t i = 0; i < num_lines; i++) {
    int16_t x0 = static_cast<int16_t>(scale8(sin8(static_cast<uint8_t>(t + i * 43)), static_cast<uint8_t>(w - 1)));
    int16_t y0 = static_cast<int16_t>(scale8(cos8(static_cast<uint8_t>(t * 2 + i * 67)), static_cast<uint8_t>(h - 1)));
    int16_t x1 = static_cast<int16_t>(scale8(sin8(static_cast<uint8_t>(t + i * 43 + 80)), static_cast<uint8_t>(w - 1)));
    int16_t y1 =
        static_cast<int16_t>(scale8(cos8(static_cast<uint8_t>(t * 2 + i * 67 + 80)), static_cast<uint8_t>(h - 1)));
    uint32_t c = ctx.pal_color(static_cast<uint8_t>(i * 40 + (t >> 3)));
    // Bresenham line
    int16_t dx = static_cast<int16_t>(abs(x1 - x0));
    int16_t dy = static_cast<int16_t>(-abs(y1 - y0));
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy;
    for (;;) {
      ctx.set_pixel_2d(x0, y0, c);
      if (x0 == x1 && y0 == y1)
        break;
      int16_t e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x0 = static_cast<int16_t>(x0 + sx);
      }
      if (e2 <= dx) {
        err += dx;
        y0 = static_cast<int16_t>(y0 + sy);
      }
    }
  }
}

// ============================================================
// 10. 2D BlackHole — swirling vortex
// ============================================================
void fx_2d_blackhole(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(0);
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;
  uint32_t t = ctx.now * scale8(spd, 4) >> 4;

  float cx = (w - 1) * 0.5f;
  float cy = (h - 1) * 0.5f;
  float max_r = sqrtf(cx * cx + cy * cy);

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      float dx = static_cast<float>(x) - cx;
      float dy = static_cast<float>(y) - cy;
      float r = sqrtf(dx * dx + dy * dy);
      float angle = atan2f(dy, dx);
      // Spiral: angle increases toward center
      float swirl = static_cast<float>(scale8(ix, 4)) * (1.0f - r / max_r);
      uint8_t hue = static_cast<uint8_t>(static_cast<int32_t>((angle + swirl) * 40.0f + (t >> 2)) & 0xFF);
      uint8_t bri = static_cast<uint8_t>(static_cast<uint32_t>(r / max_r * 255.0f) & 0xFF);
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(hue, bri));
    }
  }
}

// ============================================================
// 11. 2D SineSin — two intersecting sine-scan lines
// ============================================================
void fx_2d_sinesin(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  ctx.fade_2d(200 - scale8(ix, 100));

  uint32_t t = ctx.now * scale8(spd, 4) >> 4;

  // Horizontal scan line
  for (uint16_t x = 0; x < w; x++) {
    uint8_t y_u8 = sin8(static_cast<uint8_t>(x * scale8(ix, 12) + t));
    int16_t y = static_cast<int16_t>(scale8(y_u8, static_cast<uint8_t>(h - 1)));
    ctx.set_pixel_2d(static_cast<int16_t>(x), y, ctx.pal_color(y_u8));
  }
  // Vertical scan line
  for (uint16_t y = 0; y < h; y++) {
    uint8_t x_u8 = sin8(static_cast<uint8_t>(y * scale8(ix, 12) + t + 128));
    int16_t x = static_cast<int16_t>(scale8(x_u8, static_cast<uint8_t>(w - 1)));
    ctx.set_pixel_2d(x, static_cast<int16_t>(y), ctx.pal_color(static_cast<uint8_t>(x_u8 + 85)));
  }
}

// ============================================================
// 12. 2D Julia — simplified animated Julia set
// ============================================================
void fx_2d_julia(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(0);
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  // Animate c on a small circle
  float ang = static_cast<float>(ctx.now) * scale8(spd, 3) * 0.0001f;
  float cr = 0.7885f * cosf(ang);
  float ci = 0.7885f * sinf(ang);

  uint8_t max_iter = static_cast<uint8_t>(8 + scale8(ix, 24));  // 8..32

  float zoom = 1.5f;
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      float zr = (static_cast<float>(x) / w - 0.5f) * 2.0f * zoom;
      float zi = (static_cast<float>(y) / h - 0.5f) * 2.0f * zoom;
      uint8_t iter = 0;
      while (iter < max_iter && (zr * zr + zi * zi) < 4.0f) {
        float tmp = zr * zr - zi * zi + cr;
        zi = 2.0f * zr * zi + ci;
        zr = tmp;
        iter++;
      }
      uint8_t col = (iter == max_iter) ? 0 : static_cast<uint8_t>(iter * 255 / max_iter);
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(col));
    }
  }
}

// ============================================================
// 13. 2D Metaballs — 3 moving blobs
// ============================================================
void fx_2d_metaballs(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;

  uint32_t t = ctx.now * scale8(spd, 4) >> 4;
  // 3 ball positions
  float bx[3], by[3];
  for (uint8_t i = 0; i < 3; i++) {
    bx[i] = (w - 1) * (0.5f + 0.4f * sinf(static_cast<float>(t + i * 85) * 0.04f));
    by[i] = (h - 1) * (0.5f + 0.4f * cosf(static_cast<float>(t * 2 + i * 120) * 0.03f));
  }
  float r2 = static_cast<float>(w < h ? w : h) * 0.3f;
  r2 = r2 * r2;

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      float field = 0;
      for (uint8_t i = 0; i < 3; i++) {
        float dx = static_cast<float>(x) - bx[i];
        float dy = static_cast<float>(y) - by[i];
        float d2 = dx * dx + dy * dy;
        field += r2 / (d2 + 0.001f);
      }
      // Threshold at 1.0 for "inside" the blob
      uint8_t col = field > 1.0f ? static_cast<uint8_t>(std::min(255.0f, (field - 1.0f) * 128.0f)) : 0;
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), col > 0 ? ctx.pal_color(col) : 0u);
    }
  }
}

// ============================================================
// 14. 2D Fire — heat-based fire simulation
// ============================================================
// env->data: heat array [w * h]
void fx_2d_fire(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint32_t cells = static_cast<uint32_t>(w) * h;
  if (!ctx.env->allocate_data(cells))
    return;

  uint8_t *heat = ctx.env->data;
  uint8_t cooling = static_cast<uint8_t>(30 + scale8(255 - ctx.params->speed, 80));
  uint8_t sparking = ctx.params->intensity;

  // Cool down
  for (uint32_t i = 0; i < cells; i++) {
    uint8_t cool = hw_random8(0, static_cast<uint8_t>(((cooling * 10) / h) + 2));
    heat[i] = heat[i] > cool ? heat[i] - cool : 0;
  }

  // Heat rises (bottom to top — row 0 is bottom)
  for (uint16_t y = h - 1; y >= 2; y--) {
    for (uint16_t x = 0; x < w; x++) {
      heat[y * w + x] = static_cast<uint8_t>((static_cast<uint16_t>(heat[(y - 1) * w + x]) +
                                              static_cast<uint16_t>(heat[(y - 2) * w + x]) * 2 +
                                              static_cast<uint16_t>(heat[(y - 2) * w + (x > 0 ? x - 1 : w - 1)])) /
                                             4);
    }
  }

  // Spark at bottom row
  if (hw_random8() < sparking) {
    uint16_t x = hw_random8(static_cast<uint8_t>(w));
    heat[x] = qadd8(heat[x], hw_random8(160, 255));
  }

  // Color mapping (heat 0-255 → palette)
  // Use Fire palette indices: 0=black, 64=red, 128=orange, 192=yellow, 255=white
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      uint8_t t = heat[y * w + x];
      uint32_t c;
      if (t < 85)
        c = RGBW32(scale8(t * 3, 255), 0, 0);
      else if (t < 170)
        c = RGBW32(255, scale8((t - 85) * 3, 255), 0);
      else
        c = RGBW32(255, 255, scale8((t - 170) * 3, 255));
      // Flip y so fire rises from bottom
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(h - 1 - y), c);
    }
  }
}

// ============================================================
// 15. 2D Gravity — pixels fall and stack
// ============================================================
// env->data: pixel stack — [w] stack heights + [w * h] color grid
void fx_2d_gravity(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint32_t data_size = static_cast<uint32_t>(w) * (h + 1);
  if (!ctx.env->allocate_data(data_size * 3))  // stack(w) + color_r(w*h) + color_g(w*h)
    return;

  // Throttle
  uint8_t spd = ctx.params->speed;
  if (!ctx.should_run(static_cast<uint32_t>(50 + (255 - spd) * 3)))
    return;

  // Spawn new pixels at top, random colors
  uint8_t num_spawn = static_cast<uint8_t>(1 + scale8(ctx.params->intensity, 4));
  for (uint8_t s = 0; s < num_spawn; s++) {
    // Just fill top row with palette colors
    uint16_t x = hw_random8(static_cast<uint8_t>(w));
    // Shift column down: move each row down by 1
    for (uint16_t y = h - 1; y > 0; y--)
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y),
                       ctx.get_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y - 1)));
    // New pixel at top
    ctx.set_pixel_2d(static_cast<int16_t>(x), 0, ctx.pal_color(hw_random8()));
  }

  // Fade non-spawned pixels very slowly (persistence)
  ctx.fade_2d(240);
}

}  // namespace wled_bridge
}  // namespace esphome
