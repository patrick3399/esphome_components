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

// ============================================================
// 16. 2D DNA — double helix
// ============================================================
void fx_2d_dna(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;

  ctx.fade_2d(220);

  uint32_t t = ctx.now * scale8(spd, 4) >> 4;
  for (uint16_t y = 0; y < h; y++) {
    // Two strand positions offset by π
    int16_t x1 = static_cast<int16_t>(scale8(sin8(static_cast<uint8_t>(y * 16 + t)), static_cast<uint8_t>(w - 1)));
    int16_t x2 =
        static_cast<int16_t>(scale8(sin8(static_cast<uint8_t>(y * 16 + t + 128)), static_cast<uint8_t>(w - 1)));
    ctx.set_pixel_2d(x1, static_cast<int16_t>(y), ctx.pal_color(static_cast<uint8_t>(y * 255 / h)));
    ctx.set_pixel_2d(x2, static_cast<int16_t>(y), ctx.pal_color(static_cast<uint8_t>(y * 255 / h + 128)));
    // Connect rungs when strands are near
    if (abs(x1 - x2) <= 3) {
      int16_t lo = std::min(x1, x2), hi = std::max(x1, x2);
      for (int16_t x = lo; x <= hi; x++)
        ctx.set_pixel_2d(x, static_cast<int16_t>(y),
                         color_blend(ctx.pal_color(static_cast<uint8_t>(y * 255 / h)),
                                     ctx.pal_color(static_cast<uint8_t>(y * 255 / h + 128)), 128));
    }
  }
}

// ============================================================
// 17. 2D Pulser — concentric expanding rings
// ============================================================
void fx_2d_pulser(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  uint32_t t = ctx.now * scale8(spd, 5) >> 5;
  float cx = (w - 1) * 0.5f, cy = (h - 1) * 0.5f;
  float max_r = sqrtf(cx * cx + cy * cy);
  uint8_t rings = static_cast<uint8_t>(2 + scale8(ix, 6));

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      float dx = static_cast<float>(x) - cx;
      float dy = static_cast<float>(y) - cy;
      float r = sqrtf(dx * dx + dy * dy);
      float norm = r / max_r;
      // Multiple ring harmonics
      uint8_t v = 0;
      for (uint8_t i = 1; i <= rings; i++)
        v = qadd8(v, sin8(static_cast<uint8_t>(static_cast<uint32_t>(norm * i * 255.0f) - t * i)));
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(v));
    }
  }
}

// ============================================================
// 18. 2D Drift — Perlin velocity field
// ============================================================
// env->data: [w*h*2] float-mapped particle positions encoded as uint8 pairs (x_frac, y_frac)
void fx_2d_drift(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.pal_color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint32_t n_parts = static_cast<uint32_t>(w) * 2;
  if (!ctx.env->allocate_data(n_parts * 2))
    return;

  uint8_t *px = ctx.env->data;
  uint8_t *py = ctx.env->data + n_parts;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  // Init on first call
  if (ctx.env->call == 0) {
    for (uint32_t i = 0; i < n_parts; i++) {
      px[i] = hw_random8();
      py[i] = hw_random8();
    }
  }

  ctx.fade_2d(200 - scale8(ix, 60));

  uint32_t zt = ctx.now * scale8(spd, 3);
  for (uint32_t i = 0; i < n_parts; i++) {
    // Velocity from noise
    int8_t vx =
        static_cast<int8_t>(inoise8(static_cast<uint16_t>(px[i] * 4 + zt), static_cast<uint16_t>(py[i] * 4)) - 128) /
        16;
    int8_t vy =
        static_cast<int8_t>(inoise8(static_cast<uint16_t>(px[i] * 4), static_cast<uint16_t>(py[i] * 4 + zt)) - 128) /
        16;
    px[i] = static_cast<uint8_t>(px[i] + vx);
    py[i] = static_cast<uint8_t>(py[i] + vy);
    int16_t x = static_cast<int16_t>(scale8(px[i], static_cast<uint8_t>(w - 1)));
    int16_t y = static_cast<int16_t>(scale8(py[i], static_cast<uint8_t>(h - 1)));
    ctx.set_pixel_2d(x, y, ctx.pal_color(px[i]));
  }
}

// ============================================================
// 19. 2D NoiseFire — fire with Perlin noise overlay
// ============================================================
void fx_2d_noisefire(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint32_t cells = static_cast<uint32_t>(w) * h;
  if (!ctx.env->allocate_data(cells))
    return;

  uint8_t *heat = ctx.env->data;
  uint8_t cool = static_cast<uint8_t>(15 + scale8(255 - ctx.params->speed, 40));
  uint8_t spk = ctx.params->intensity;
  uint32_t zt = ctx.now >> 3;

  for (uint32_t i = 0; i < cells; i++) {
    uint8_t c = hw_random8(0, static_cast<uint8_t>((cool * 10) / h + 2));
    heat[i] = heat[i] > c ? heat[i] - c : 0;
  }
  for (uint16_t y = h - 1; y >= 2; y--)
    for (uint16_t x = 0; x < w; x++)
      heat[y * w + x] = static_cast<uint8_t>((static_cast<uint16_t>(heat[(y - 1) * w + x]) +
                                              static_cast<uint16_t>(heat[(y - 2) * w + x]) * 2 +
                                              static_cast<uint16_t>(heat[(y - 2) * w + (x > 0 ? x - 1 : w - 1)])) /
                                             4);
  if (hw_random8() < spk) {
    uint16_t x = hw_random8(static_cast<uint8_t>(w));
    heat[x] = qadd8(heat[x], hw_random8(160, 255));
  }

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      uint8_t base = heat[y * w + x];
      // Perlin noise modulates hue
      uint8_t noise = inoise8(static_cast<uint16_t>(x * 30 + zt), static_cast<uint16_t>(y * 30));
      uint8_t hue = static_cast<uint8_t>(scale8(base, 64) + scale8(noise, 192));
      uint32_t c;
      if (base < 85)
        c = RGBW32(scale8(base * 3, 255), 0, 0);
      else if (base < 170)
        c = RGBW32(255, scale8((base - 85) * 3, 255), scale8(noise, 40));
      else
        c = ctx.pal_color(hue, base);
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(h - 1 - y), c);
    }
  }
}

// ============================================================
// 20. 2D NoiseMove — scrolling noise with velocity
// ============================================================
void fx_2d_noisemove(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.pal_color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  // Scroll offset drifts over time
  uint32_t ox = ctx.now * scale8(spd, 3);
  uint32_t oy = ctx.now * scale8(spd, 2) + 12345;
  uint16_t scale = static_cast<uint16_t>(64 + scale8(255 - ix, 192));

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      uint8_t n = inoise8(static_cast<uint16_t>(x * scale + ox), static_cast<uint16_t>(y * scale + oy));
      uint8_t n2 = inoise8(static_cast<uint16_t>(x * scale + oy), static_cast<uint16_t>(y * scale + ox + 30000));
      uint8_t col = qadd8(n, n2);
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(col, n));
    }
  }
}

// ============================================================
// 21. 2D ColoredBursts — color burst rays from center
// ============================================================
void fx_2d_colored_bursts(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(0);
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  ctx.fade_2d(180 - scale8(ix, 80));

  uint8_t num_rays = static_cast<uint8_t>(3 + scale8(ix, 10));
  uint32_t t = ctx.now * scale8(spd, 4) >> 4;

  float cx = (w - 1) * 0.5f;
  float cy = (h - 1) * 0.5f;
  float max_r = sqrtf(cx * cx + cy * cy);

  for (uint8_t i = 0; i < num_rays; i++) {
    float angle = static_cast<float>(i) * 6.2832f / num_rays + static_cast<float>(t) * 0.05f;
    float len = max_r * (0.6f + 0.4f * sinf(static_cast<float>(t) * 0.07f + i * 1.5f));
    uint32_t c = ctx.pal_color(static_cast<uint8_t>(i * 255 / num_rays + (t >> 3)));
    // Draw ray as line
    uint8_t steps = static_cast<uint8_t>(len + 0.5f);
    for (uint8_t s = 0; s < steps; s++) {
      float r = static_cast<float>(s) / steps * len;
      int16_t x = static_cast<int16_t>(cx + cosf(angle) * r + 0.5f);
      int16_t y = static_cast<int16_t>(cy + sinf(angle) * r + 0.5f);
      uint8_t bri = static_cast<uint8_t>(255u - static_cast<uint32_t>(s) * 255u / steps);
      ctx.set_pixel_2d(x, y, color_blend(c, 0, bri));
    }
  }
}

// ============================================================
// 22. 2D YinYang — rotating yin-yang symbol
// ============================================================
void fx_2d_yinyang(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;

  float angle = static_cast<float>(ctx.now) * scale8(spd, 3) * 0.0004f;
  float ca = cosf(angle), sa = sinf(angle);
  float cx = (w - 1) * 0.5f, cy = (h - 1) * 0.5f;
  float R = (cx < cy ? cx : cy) * 0.9f;

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      float dx = static_cast<float>(x) - cx;
      float dy = static_cast<float>(y) - cy;
      // Rotate
      float rx = dx * ca - dy * sa;
      float ry = dx * sa + dy * ca;
      float r = sqrtf(rx * rx + ry * ry);
      if (r > R) {
        ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), 0);
        continue;
      }
      // Yin-yang boundary: upper half = yin (dark), lower = yang (light)
      // Small circles in each half
      bool yang = ry > 0;
      // Small dot check
      float ty = ry - R * 0.5f * (yang ? -1 : 1);
      float dot_r = R * 0.2f;
      bool in_dot = sqrtf(rx * rx + ty * ty) < dot_r;
      bool cell = (yang ^ in_dot);
      uint32_t c0 = ctx.color(0), c1 = ctx.color(1);
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), cell ? c1 : c0);
    }
  }
}

// ============================================================
// 23. 2D Fireworks2D — particle fireworks
// ============================================================
// env->data: 3 shells × 16 bytes: [x8, y8, vx8, vy8, hue, age, active, pad, ...16 particles × 4 bytes]
struct Shell {
  uint8_t x, y, hue, age, active;
};
struct Particle {
  uint8_t x, y, vx_s, vy_s;  // vx/vy stored +128 offset
};
static constexpr uint8_t FW2D_SHELLS = 2;
static constexpr uint8_t FW2D_PARTS = 12;
static constexpr size_t FW2D_DATA = FW2D_SHELLS * (sizeof(Shell) + FW2D_PARTS * sizeof(Particle));

void fx_2d_fireworks2d(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(0);
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  if (!ctx.env->allocate_data(FW2D_DATA))
    return;

  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  ctx.fade_2d(200 - scale8(ix, 60));

  auto *shells = reinterpret_cast<Shell *>(ctx.env->data);
  auto *parts_base = reinterpret_cast<Particle *>(ctx.env->data + FW2D_SHELLS * sizeof(Shell));

  if (!ctx.should_run(static_cast<uint32_t>(40 + (255 - spd) * 2)))
    return;

  for (uint8_t s = 0; s < FW2D_SHELLS; s++) {
    Shell &sh = shells[s];
    Particle *parts = parts_base + s * FW2D_PARTS;

    if (!sh.active) {
      // Spawn new shell
      sh.x = hw_random8(static_cast<uint8_t>(w));
      sh.y = hw_random8(static_cast<uint8_t>(h));
      sh.hue = hw_random8();
      sh.age = 0;
      sh.active = 1;
      for (uint8_t p = 0; p < FW2D_PARTS; p++) {
        parts[p].x = sh.x;
        parts[p].y = sh.y;
        parts[p].vx_s = hw_random8();  // random velocity
        parts[p].vy_s = hw_random8();
      }
    }

    sh.age++;
    if (sh.age > 30)
      sh.active = 0;

    uint32_t c = ctx.pal_color(sh.hue, 255u - sh.age * 8u > 0 ? 255u - sh.age * 8u : 0);
    for (uint8_t p = 0; p < FW2D_PARTS; p++) {
      int16_t vx = static_cast<int16_t>(parts[p].vx_s) - 128;
      int16_t vy = static_cast<int16_t>(parts[p].vy_s) - 128;
      int16_t nx = static_cast<int16_t>(parts[p].x) + vx / 16;
      int16_t ny = static_cast<int16_t>(parts[p].y) + vy / 16;
      parts[p].x = static_cast<uint8_t>(nx & 0xFF);
      parts[p].y = static_cast<uint8_t>(ny & 0xFF);
      ctx.set_pixel_2d(static_cast<int16_t>(scale8(parts[p].x, static_cast<uint8_t>(w - 1))),
                       static_cast<int16_t>(scale8(parts[p].y, static_cast<uint8_t>(h - 1))), c);
    }
  }
}

// ============================================================
// 24. 2D Sprouts — growing blobs from random seeds
// ============================================================
void fx_2d_sprouts(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint32_t cells = static_cast<uint32_t>(w) * h;
  // data: cell age (0 = empty, >0 = growing)
  if (!ctx.env->allocate_data(cells))
    return;

  uint8_t *age = ctx.env->data;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  if (!ctx.should_run(static_cast<uint32_t>(50 + (255 - spd) * 3)))
    return;

  // Spawn new seeds
  if (hw_random8() < 40) {
    uint16_t x = hw_random8(static_cast<uint8_t>(w));
    uint16_t y = hw_random8(static_cast<uint8_t>(h));
    age[y * w + x] = 1;
  }

  // Age and spread
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      if (age[y * w + x] == 0)
        continue;
      age[y * w + x] = qadd8(age[y * w + x], 4);
      if (age[y * w + x] > 200)
        age[y * w + x] = 0;
      // Spread to neighbors occasionally
      if (hw_random8() < 40 + scale8(ix, 80)) {
        uint16_t nx = static_cast<uint16_t>((x + hw_random8(3) + w - 1) % w);
        uint16_t ny = static_cast<uint16_t>((y + hw_random8(3) + h - 1) % h);
        if (age[ny * w + nx] == 0)
          age[ny * w + nx] = 1;
      }
    }
  }

  uint8_t hue_base = static_cast<uint8_t>(ctx.now >> 5);
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      uint8_t a = age[y * w + x];
      if (a == 0) {
        ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), 0);
      } else {
        uint8_t bri = a < 128 ? a * 2 : 255 - (a - 128) * 2;
        ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y),
                         ctx.pal_color(static_cast<uint8_t>(hue_base + a), bri));
      }
    }
  }
}

// ============================================================
// 25. 2D Zentangle — geometric repeating pattern
// ============================================================
void fx_2d_zentangle(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;
  uint32_t t = ctx.now * scale8(spd, 3) >> 4;

  // Tile size derived from intensity
  uint8_t tile = static_cast<uint8_t>(2 + scale8(ix, 8));

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      uint8_t tx = static_cast<uint8_t>(x % tile);
      uint8_t ty = static_cast<uint8_t>(y % tile);
      // Alternating tile patterns based on tile index
      uint8_t ti = static_cast<uint8_t>((x / tile + y / tile) % 4);
      uint8_t v;
      switch (ti) {
        case 0:
          v = sin8(static_cast<uint8_t>(tx * 255 / tile + t));
          break;
        case 1:
          v = cos8(static_cast<uint8_t>(ty * 255 / tile + t));
          break;
        case 2:
          v = sin8(static_cast<uint8_t>((tx + ty) * 255 / (tile * 2) + t));
          break;
        default:
          v = sin8(static_cast<uint8_t>(tx * ty * 16 + t));
          break;
      }
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(v, scale8(v, 200) + 55));
    }
  }
}

// ============================================================
// 26. 2D Akemi — cat face (simplified static)
// ============================================================
void fx_2d_akemi(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t hue_base = static_cast<uint8_t>(ctx.now * scale8(spd, 2) >> 5);

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      float nx = (static_cast<float>(x) / w - 0.5f) * 2.0f;
      float ny = (static_cast<float>(y) / h - 0.5f) * 2.0f;
      float r = sqrtf(nx * nx + ny * ny);
      // Head outline
      uint32_t c = 0;
      if (r < 0.9f && r > 0.7f)
        c = ctx.pal_color(hue_base, 200);
      // Eyes
      float el = sqrtf((nx + 0.3f) * (nx + 0.3f) + (ny + 0.2f) * (ny + 0.2f));
      float er = sqrtf((nx - 0.3f) * (nx - 0.3f) + (ny + 0.2f) * (ny + 0.2f));
      if (el < 0.15f || er < 0.15f)
        c = ctx.pal_color(static_cast<uint8_t>(hue_base + 128), 255);
      // Nose
      if (sqrtf(nx * nx + (ny - 0.1f) * (ny - 0.1f)) < 0.07f)
        c = ctx.color(1);
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), c);
    }
  }
}

// ============================================================
// 27. 2D Hopalong — Hopalong attractor
// ============================================================
void fx_2d_hopalong(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(0);
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  ctx.fade_2d(200 - scale8(ix, 50));

  float a = 0.4f + 0.1f * sinf(static_cast<float>(ctx.now) * 0.001f);
  float b = 0.3f + 0.1f * cosf(static_cast<float>(ctx.now) * 0.00073f);
  float c_val = 0.2f + 0.1f * sinf(static_cast<float>(ctx.now) * 0.0013f);

  uint16_t iters = static_cast<uint16_t>(w * h / 2);
  float scale_f = static_cast<float>(std::min(w, h)) * 0.2f;
  float px = 0.0f, py = 0.0f;
  uint32_t t_shift = ctx.now * scale8(spd, 3) >> 8;

  for (uint16_t i = 0; i < iters; i++) {
    float sign_py = py >= 0 ? 1.0f : -1.0f;
    float nx = py - sign_py * sqrtf(fabsf(b * px - c_val));
    float ny = a - px;
    px = nx;
    py = ny;
    int16_t sx = static_cast<int16_t>(px * scale_f + w * 0.5f + 0.5f);
    int16_t sy = static_cast<int16_t>(py * scale_f + h * 0.5f + 0.5f);
    ctx.set_pixel_2d(sx, sy, ctx.pal_color(static_cast<uint8_t>(i + t_shift)));
  }
}

// ============================================================
// 28. 2D Magnetics — magnetic field lines
// ============================================================
void fx_2d_magnetics(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  // 2 magnetic poles drifting
  uint32_t t = ctx.now * scale8(spd, 4) >> 5;
  float p1x = (w - 1) * (0.3f + 0.2f * sinf(static_cast<float>(t) * 0.04f));
  float p1y = (h - 1) * (0.5f + 0.2f * cosf(static_cast<float>(t) * 0.03f));
  float p2x = (w - 1) * (0.7f + 0.2f * cosf(static_cast<float>(t) * 0.05f));
  float p2y = (h - 1) * (0.5f + 0.2f * sinf(static_cast<float>(t) * 0.04f));

  uint8_t num_lines = static_cast<uint8_t>(3 + scale8(ix, 9));

  ctx.fill_black();
  for (uint8_t li = 0; li < num_lines; li++) {
    float angle = static_cast<float>(li) * 3.14159f / num_lines;
    float fx = p1x + cosf(angle) * 2.0f;
    float fy = p1y + sinf(angle) * 2.0f;
    uint32_t c = ctx.pal_color(static_cast<uint8_t>(li * 255 / num_lines));

    for (uint16_t s = 0; s < 60; s++) {
      ctx.set_pixel_2d(static_cast<int16_t>(fx + 0.5f), static_cast<int16_t>(fy + 0.5f), c);
      // Gradient follows field
      float d1x = fx - p1x, d1y = fy - p1y;
      float d2x = fx - p2x, d2y = fy - p2y;
      float r12 = d1x * d1x + d1y * d1y + 0.01f;
      float r22 = d2x * d2x + d2y * d2y + 0.01f;
      float gx = d1x / r12 - d2x / r22;
      float gy = d1y / r12 - d2y / r22;
      float len = sqrtf(gx * gx + gy * gy) + 0.001f;
      fx += gx / len * 0.8f;
      fy += gy / len * 0.8f;
      if (fx < 0 || fy < 0 || fx >= w || fy >= h)
        break;
    }
  }
}

// ============================================================
// 29. 2D Hypnotic — rotating spiral rings
// ============================================================
void fx_2d_hypnotic(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.pal_color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  float cx = (w - 1) * 0.5f, cy = (h - 1) * 0.5f;
  uint32_t t = ctx.now * scale8(spd, 3) >> 4;
  float freq = 1.0f + scale8(ix, 4);

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      float dx = static_cast<float>(x) - cx;
      float dy = static_cast<float>(y) - cy;
      float r = sqrtf(dx * dx + dy * dy);
      float theta = atan2f(dy, dx);
      // Archimedean spiral
      float spiral = r - static_cast<float>(t) * 0.05f + theta * freq;
      uint8_t v = sin8(static_cast<uint8_t>(static_cast<uint32_t>(spiral * 20.0f) & 0xFF));
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(v, v));
    }
  }
}

// ============================================================
// 30. 2D Bubbles — rising bubbles
// ============================================================
// env->data: 8 bubbles × 4 bytes [x, y_fixed, radius, hue]
static constexpr uint8_t BUBBLES_N = 8;

void fx_2d_bubbles(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(0);
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  if (!ctx.env->allocate_data(BUBBLES_N * 4))
    return;

  uint8_t *bx = ctx.env->data;
  uint8_t *by = ctx.env->data + BUBBLES_N;
  uint8_t *brad = ctx.env->data + BUBBLES_N * 2;
  uint8_t *bhue = ctx.env->data + BUBBLES_N * 3;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  // Init
  if (ctx.env->call == 0) {
    for (uint8_t i = 0; i < BUBBLES_N; i++) {
      bx[i] = hw_random8(static_cast<uint8_t>(w));
      by[i] = hw_random8(static_cast<uint8_t>(h));
      brad[i] = static_cast<uint8_t>(1 + hw_random8(3));
      bhue[i] = hw_random8();
    }
  }

  if (!ctx.should_run(static_cast<uint32_t>(30 + (255 - spd) * 2)))
    return;

  ctx.fade_2d(230 - scale8(ix, 40));

  for (uint8_t i = 0; i < BUBBLES_N; i++) {
    // Rise
    if (by[i] == 0) {
      bx[i] = hw_random8(static_cast<uint8_t>(w));
      by[i] = static_cast<uint8_t>(h - 1);
      brad[i] = static_cast<uint8_t>(1 + hw_random8(3));
      bhue[i] = hw_random8();
    } else {
      by[i]--;
    }
    // Draw circle
    uint8_t r = brad[i];
    uint32_t c = ctx.pal_color(bhue[i]);
    for (int16_t dy = -static_cast<int16_t>(r); dy <= static_cast<int16_t>(r); dy++) {
      for (int16_t dx = -static_cast<int16_t>(r); dx <= static_cast<int16_t>(r); dx++) {
        if (dx * dx + dy * dy <= r * r) {
          ctx.set_pixel_2d(static_cast<int16_t>(bx[i]) + dx, static_cast<int16_t>(by[i]) + dy, c);
        }
      }
    }
  }
}

// ============================================================
// 31. 2D Magnifying — magnifying glass panning
// ============================================================
void fx_2d_magnifying(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.pal_color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;
  uint32_t t = ctx.now * scale8(spd, 4) >> 5;

  // Lens center drifts
  float lcx = (w - 1) * (0.5f + 0.4f * sinf(static_cast<float>(t) * 0.03f));
  float lcy = (h - 1) * (0.5f + 0.4f * cosf(static_cast<float>(t) * 0.04f));
  float lens_r = static_cast<float>(std::min(w, h)) * 0.25f;
  float zoom = 0.4f;

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      float dx = static_cast<float>(x) - lcx;
      float dy = static_cast<float>(y) - lcy;
      float r = sqrtf(dx * dx + dy * dy);
      float sx, sy;
      if (r < lens_r && r > 0) {
        float scale = zoom + (1.0f - zoom) * (r / lens_r);
        sx = lcx + dx / scale;
        sy = lcy + dy / scale;
      } else {
        sx = static_cast<float>(x);
        sy = static_cast<float>(y);
      }
      uint16_t xi = static_cast<uint16_t>(static_cast<int16_t>(sx + 0.5f) & (w - 1));
      uint16_t yi = static_cast<uint16_t>(static_cast<int16_t>(sy + 0.5f) & (h - 1));
      uint8_t nc = inoise8(static_cast<uint16_t>(xi * 40 + t * 2), static_cast<uint16_t>(yi * 40));
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(nc, 200));
    }
  }
}

// ============================================================
// 32. 2D Popcorn2D — 2D popcorn bursts
// ============================================================
static constexpr uint8_t POPN2D_KERNELS = 10;
void fx_2d_popcorn2d(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(0);
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  if (!ctx.env->allocate_data(POPN2D_KERNELS * 5))
    return;

  uint8_t *kx = ctx.env->data;
  uint8_t *ky = ctx.env->data + POPN2D_KERNELS;
  uint8_t *vx = ctx.env->data + POPN2D_KERNELS * 2;  // +128 offset
  uint8_t *vy_d = ctx.env->data + POPN2D_KERNELS * 3;
  uint8_t *khue = ctx.env->data + POPN2D_KERNELS * 4;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  ctx.fade_2d(220 - scale8(ix, 50));

  if (!ctx.should_run(static_cast<uint32_t>(20 + (255 - spd) * 2)))
    return;

  for (uint8_t i = 0; i < POPN2D_KERNELS; i++) {
    // Spawn if at bottom
    if (ky[i] >= h - 1 || (kx[i] == 0 && ky[i] == 0)) {
      kx[i] = hw_random8(static_cast<uint8_t>(w));
      ky[i] = static_cast<uint8_t>(h - 1);
      vx[i] = static_cast<uint8_t>(hw_random8(220, 255));  // upward
      vy_d[i] = hw_random8();
      khue[i] = hw_random8();
    }
    // Apply gravity (decelerate upward = increase vy toward gravity)
    int16_t vel = static_cast<int16_t>(vx[i]) - 128 - 2;
    if (vel < -100)
      vel = -100;
    vx[i] = static_cast<uint8_t>(vel + 128);
    int16_t nx = static_cast<int16_t>(ky[i]) + vel / 8;
    if (nx >= h)
      nx = h - 1;
    if (nx < 0)
      nx = 0;
    ky[i] = static_cast<uint8_t>(nx);
    int16_t hx = static_cast<int16_t>(kx[i]) + (static_cast<int16_t>(vy_d[i]) - 128) / 16;
    kx[i] = static_cast<uint8_t>(hx & 0xFF);
    ctx.set_pixel_2d(static_cast<int16_t>(scale8(kx[i], static_cast<uint8_t>(w - 1))), static_cast<int16_t>(ky[i]),
                     ctx.pal_color(khue[i]));
  }
}

// ============================================================
// 33. 2D Sparkle2D — random sparkles on canvas
// ============================================================
void fx_2d_sparkle2d(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(0);
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  ctx.fade_2d(200 - scale8(spd, 60));

  uint8_t num_sparks = static_cast<uint8_t>(1 + scale8(ix, 12));
  for (uint8_t i = 0; i < num_sparks; i++) {
    int16_t x = static_cast<int16_t>(hw_random8(static_cast<uint8_t>(w)));
    int16_t y = static_cast<int16_t>(hw_random8(static_cast<uint8_t>(h)));
    ctx.set_pixel_2d(x, y, ctx.pal_color(hw_random8(), 255));
    // Soft glow
    if (x > 0)
      ctx.set_pixel_2d(static_cast<int16_t>(x - 1), y, color_fade(ctx.pal_color(hw_random8()), 120));
    if (y > 0)
      ctx.set_pixel_2d(x, static_cast<int16_t>(y - 1), color_fade(ctx.pal_color(hw_random8()), 120));
  }
}

// ============================================================
// 34. 2D Pendulum — oscillating pendulum sweep
// ============================================================
void fx_2d_pendulum(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(0);
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;
  uint8_t ix = ctx.params->intensity;

  ctx.fade_2d(200 - scale8(ix, 80));

  // Pendulum angle
  float ang = 1.2f * sinf(static_cast<float>(ctx.now) * scale8(spd, 3) * 0.0003f);
  float cx = (w - 1) * 0.5f;
  float arm_len = h * 0.8f;

  // Draw pendulum path (several points along the sweep trail)
  uint8_t trail = static_cast<uint8_t>(3 + scale8(ix, 6));
  for (uint8_t t = 0; t <= trail; t++) {
    float a = ang * (1.0f - static_cast<float>(t) / trail * 0.5f);
    float bx = cx + arm_len * sinf(a);
    float by = static_cast<float>(t > 0 ? t - 1 : 0) * h / trail;
    uint8_t bri = static_cast<uint8_t>(255u - t * 255u / (trail + 1));
    ctx.set_pixel_2d(static_cast<int16_t>(bx + 0.5f), static_cast<int16_t>(by),
                     ctx.pal_color(static_cast<uint8_t>(t * 40), bri));
  }
  // Bob at end of arm
  int16_t bx = static_cast<int16_t>(cx + arm_len * sinf(ang) + 0.5f);
  int16_t by = static_cast<int16_t>(h * 0.8f + 0.5f);
  ctx.set_pixel_2d(bx, by, ctx.color(0));
  if (bx > 0)
    ctx.set_pixel_2d(static_cast<int16_t>(bx - 1), by, ctx.color(0));
  if (bx < static_cast<int16_t>(w - 1))
    ctx.set_pixel_2d(static_cast<int16_t>(bx + 1), by, ctx.color(0));
}

// ============================================================
// 35. 2D Heatmap — animated heat flow
// ============================================================
void fx_2d_heatmap(EffectContext &ctx) {
  if (!ctx.is_2d()) {
    ctx.fill(ctx.pal_color(0));
    return;
  }
  uint16_t w = ctx.matrix_w, h = ctx.matrix_h;
  uint8_t spd = ctx.params->speed;

  uint32_t t = ctx.now * scale8(spd, 3);

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      // Combine multiple noise octaves for heat-like appearance
      uint8_t v = inoise8(static_cast<uint16_t>(x * 50 + t), static_cast<uint16_t>(y * 50));
      uint8_t v2 = inoise8(static_cast<uint16_t>(x * 100 + t * 2), static_cast<uint16_t>(y * 100 + t));
      uint8_t heat = static_cast<uint8_t>((static_cast<uint16_t>(v) + v2) >> 1);
      // Map to heat colors (cold=blue, warm=red/yellow)
      uint32_t c;
      if (heat < 85)
        c = RGBW32(0, 0, scale8(heat * 3, 255));
      else if (heat < 170)
        c = RGBW32(scale8((heat - 85) * 3, 255), 0, scale8(255 - (heat - 85) * 3, 255));
      else
        c = RGBW32(255, scale8((heat - 170) * 3, 255), 0);
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), c);
    }
  }
}

}  // namespace wled_bridge
}  // namespace esphome
