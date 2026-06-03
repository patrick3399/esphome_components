#include "esphome/core/defines.h"
#ifdef WLED_BRIDGE_AUDIO

#include "wled_effect_context.h"
#include "wled_fx_math.h"

#include <cmath>
#include <cstring>

namespace esphome {
namespace wled_bridge {

// ============================================================
//  Volume-reactive effects (no FFT required)
// ============================================================

// Gravimeter — VU meter with gravity and perlin noise.
void fx_gravimeter(EffectContext &ctx) {
  uint8_t SPEED = ctx.params->speed;
  uint8_t INTENSITY = ctx.params->intensity;
  float vol = ctx.volume();

  if (ctx.env->call == 0)
    ctx.env->step = 0;

  ctx.fade_to_black(240);

  float gravity = 0.015f + static_cast<float>(SPEED) * 0.0002f;
  float peak_pos = static_cast<float>(ctx.env->step) / 256.0f;

  float target = vol / 255.0f;
  if (target > peak_pos) {
    peak_pos = target;
    ctx.env->aux0 = 0;  // velocity reset
  } else {
    float vel = static_cast<float>(ctx.env->aux0) / 256.0f;
    vel += gravity;
    peak_pos -= vel;
    if (peak_pos < 0)
      peak_pos = 0;
    ctx.env->aux0 = static_cast<uint16_t>(vel * 256.0f);
  }

  ctx.env->step = static_cast<uint32_t>(peak_pos * 256.0f);

  int32_t bar_len = static_cast<int32_t>(peak_pos * static_cast<float>(ctx.len));
  for (int32_t i = 0; i < bar_len && i < ctx.len; i++) {
    uint8_t idx = static_cast<uint8_t>((static_cast<uint32_t>(i) * 255) / static_cast<uint32_t>(ctx.len));
    ctx.set_pixel(i, ctx.pal_color(idx));
  }

  if (bar_len < ctx.len)
    ctx.set_pixel(bar_len, RGBW32(255, 255, 255, 0));
}

// Puddles — random coloured puddles based on volume.
void fx_puddles(EffectContext &ctx) {
  uint8_t INTENSITY = ctx.params->intensity;
  float vol = ctx.volume();

  ctx.fade_to_black(224);

  if (vol > 1.0f) {
    int32_t pos = hw_random16(static_cast<uint16_t>(ctx.len));
    uint8_t bri = static_cast<uint8_t>(vol);
    uint8_t pal_idx = hw_random8();
    uint32_t col = ctx.pal_color(pal_idx, bri);

    int32_t width = 1 + (INTENSITY >> 5);
    for (int32_t w = -width; w <= width; w++) {
      int32_t px = pos + w;
      if (px >= 0 && px < ctx.len)
        ctx.set_pixel(px, col);
    }
  }
}

// Puddlepeak — coloured puddles on beat.
void fx_puddlepeak(EffectContext &ctx) {
  uint8_t INTENSITY = ctx.params->intensity;

  ctx.fade_to_black(224);

  if (ctx.beat()) {
    int32_t pos = hw_random16(static_cast<uint16_t>(ctx.len));
    uint8_t pal_idx = hw_random8();
    uint32_t col = ctx.pal_color(pal_idx);

    int32_t width = 2 + (INTENSITY >> 5);
    for (int32_t w = -width; w <= width; w++) {
      int32_t px = pos + w;
      if (px >= 0 && px < ctx.len)
        ctx.set_pixel(px, col);
    }
  }
}

// Pixels — random pixels based on volume.
void fx_pixels_vol(EffectContext &ctx) {
  float vol = ctx.volume();

  ctx.fade_to_black(200);

  if (vol < 1.0f)
    return;

  uint8_t num = 1 + static_cast<uint8_t>(vol / 32.0f);
  for (uint8_t n = 0; n < num; n++) {
    int32_t pos = hw_random16(static_cast<uint16_t>(ctx.len));
    uint8_t idx = hw_random8();
    uint8_t bri = static_cast<uint8_t>(vol);
    ctx.set_pixel(pos, ctx.pal_color(idx, bri));
  }
}

// Pixelwave — pixels emanating from center.
void fx_pixelwave(EffectContext &ctx) {
  uint8_t SPEED = ctx.params->speed;
  float vol = ctx.volume();

  uint32_t shift = (ctx.now * static_cast<uint32_t>(SPEED)) >> 10;
  ctx.fade_to_black(220);

  if (vol > 2.0f) {
    int32_t center = ctx.len / 2;
    uint8_t bri = static_cast<uint8_t>(vol);
    uint8_t idx = static_cast<uint8_t>(shift);
    uint32_t col = ctx.pal_color(idx, bri);
    ctx.set_pixel(center, col);
    if (center > 0)
      ctx.set_pixel(center - 1, col);
    if (center < ctx.len - 1)
      ctx.set_pixel(center + 1, col);
  }

  // Shift outward.
  if (ctx.env->call % 2 == 0) {
    int32_t half = ctx.len / 2;
    for (int32_t i = 0; i < half - 1; i++) {
      int32_t abs_l = ctx.map_pixel(half - 1 - i);
      int32_t src_l = ctx.map_pixel(half - i);
      if (abs_l >= 0 && src_l >= 0 && abs_l < static_cast<int32_t>(ctx.frame_len) &&
          src_l < static_cast<int32_t>(ctx.frame_len))
        ctx.frame_buf[abs_l] = ctx.frame_buf[src_l];
    }
    for (int32_t i = ctx.len - 1; i > half; i--) {
      int32_t abs_r = ctx.map_pixel(i);
      int32_t src_r = ctx.map_pixel(i - 1);
      if (abs_r >= 0 && src_r >= 0 && abs_r < static_cast<int32_t>(ctx.frame_len) &&
          src_r < static_cast<int32_t>(ctx.frame_len))
        ctx.frame_buf[abs_r] = ctx.frame_buf[src_r];
    }
  }
}

// Noisemeter — VU meter with noise texture.
void fx_noisemeter(EffectContext &ctx) {
  float vol = ctx.volume();

  ctx.fade_to_black(240);

  int32_t bar_len = static_cast<int32_t>((vol / 255.0f) * static_cast<float>(ctx.len));
  if (bar_len > ctx.len)
    bar_len = ctx.len;

  for (int32_t i = 0; i < bar_len; i++) {
    uint8_t noise = inoise8(static_cast<uint16_t>(i * 40 + ctx.now / 3));
    uint8_t idx = static_cast<uint8_t>(noise);
    ctx.set_pixel(i, ctx.pal_color(idx));
  }
}

// Noisefire — perlin noise volume-reactive fire.
void fx_noisefire(EffectContext &ctx) {
  float vol = ctx.volume();

  for (int32_t i = 0; i < ctx.len; i++) {
    uint8_t noise = inoise8(static_cast<uint16_t>(i * 50 + ctx.now / 4));
    uint8_t heat = scale8(noise, static_cast<uint8_t>(vol));
    ctx.set_pixel(i, ctx.pal_color(heat));
  }
}

// Matripix — matrix-style pixel rain driven by volume.
void fx_matripix(EffectContext &ctx) {
  float vol = ctx.volume();

  // Shift all pixels down by 1.
  for (int32_t i = 0; i < ctx.len - 1; i++) {
    int32_t dst = ctx.map_pixel(i);
    int32_t src = ctx.map_pixel(i + 1);
    if (dst >= 0 && src >= 0 && dst < static_cast<int32_t>(ctx.frame_len) && src < static_cast<int32_t>(ctx.frame_len))
      ctx.frame_buf[dst] = ctx.frame_buf[src];
  }

  // Add a new pixel at the top based on volume.
  if (vol > 2.0f) {
    uint8_t pal_idx = static_cast<uint8_t>(vol);
    ctx.set_pixel(ctx.len - 1, ctx.pal_color(pal_idx));
  } else {
    ctx.set_pixel(ctx.len - 1, 0);
  }
}

// Ripple Peak — ripples triggered by beat detection.
void fx_ripple_peak(EffectContext &ctx) {
  uint8_t SPEED = ctx.params->speed;
  uint8_t INTENSITY = ctx.params->intensity;

  ctx.fade_to_black(240);

  // Use env->data to store ripple state: [pos(2), age(1), color(1)] × max_ripples.
  constexpr size_t MAX_RIPPLES = 8;
  constexpr size_t RIPPLE_SIZE = 4;
  if (!ctx.env->allocate_data(MAX_RIPPLES * RIPPLE_SIZE))
    return;

  uint8_t *ripples = ctx.env->data;

  // Spawn new ripple on beat.
  if (ctx.beat()) {
    for (size_t r = 0; r < MAX_RIPPLES; r++) {
      uint8_t *rp = ripples + r * RIPPLE_SIZE;
      if (rp[2] == 0) {
        uint16_t pos = hw_random16(static_cast<uint16_t>(ctx.len));
        rp[0] = pos & 0xFF;
        rp[1] = pos >> 8;
        rp[2] = 1;  // age
        rp[3] = hw_random8();  // palette index
        break;
      }
    }
  }

  // Animate ripples.
  for (size_t r = 0; r < MAX_RIPPLES; r++) {
    uint8_t *rp = ripples + r * RIPPLE_SIZE;
    uint8_t age = rp[2];
    if (age == 0)
      continue;

    int32_t center = static_cast<int32_t>(rp[0]) | (static_cast<int32_t>(rp[1]) << 8);
    int32_t radius = age >> 1;
    uint8_t bri = 255 - age * 3;
    if (bri < 10) {
      rp[2] = 0;
      continue;
    }

    uint32_t col = ctx.pal_color(rp[3], bri);
    if (center - radius >= 0 && center - radius < ctx.len)
      ctx.set_pixel(center - radius, col);
    if (center + radius >= 0 && center + radius < ctx.len)
      ctx.set_pixel(center + radius, col);
    if (center >= 0 && center < ctx.len)
      ctx.set_pixel(center, ctx.pal_color(rp[3]));

    rp[2] = age + 1 + (SPEED >> 6);
  }
}

// Juggles — juggling balls with volume-driven brightness.
void fx_juggles_vol(EffectContext &ctx) {
  float vol = ctx.volume();

  ctx.fade_to_black(224);

  uint8_t num_dots = 3 + (ctx.params->intensity >> 5);
  if (num_dots > 8)
    num_dots = 8;

  uint8_t bri = static_cast<uint8_t>(vol);
  if (bri < 32)
    bri = 32;

  for (uint8_t d = 0; d < num_dots; d++) {
    uint16_t pos16 = beatsin16(10 + d * 3, 0, static_cast<uint16_t>(ctx.len - 1) * 256, ctx.now, d * 8192);
    int32_t pos = pos16 >> 8;
    uint8_t pal_idx = static_cast<uint8_t>(d * 32 + (ctx.now >> 4));
    ctx.set_pixel(pos, ctx.pal_color(pal_idx, bri));
  }
}

// Plasmoid — volume-driven sine wave plasma.
void fx_plasmoid(EffectContext &ctx) {
  uint8_t SPEED = ctx.params->speed;
  float vol = ctx.volume();

  uint8_t bri = static_cast<uint8_t>(vol);
  if (bri < 16)
    bri = 16;

  for (int32_t i = 0; i < ctx.len; i++) {
    uint16_t phase = static_cast<uint16_t>(i * 16 + (ctx.now * static_cast<uint32_t>(SPEED)) / 256);
    uint8_t val = sin8(phase >> 8);
    uint8_t idx = val + static_cast<uint8_t>(ctx.now >> 4);
    ctx.set_pixel(i, ctx.pal_color(idx, scale8(bri, val)));
  }
}

// ============================================================
//  FFT-based effects (require WLED_BRIDGE_FFT)
// ============================================================

// GEQ — 16-band graphic equalizer mapped across the strip.
void fx_geq(EffectContext &ctx) {
  ctx.fill_black();

  int32_t band_width = ctx.len / AUDIO_GEQ_CHANNELS;
  if (band_width < 1)
    band_width = 1;

  for (uint8_t ch = 0; ch < AUDIO_GEQ_CHANNELS; ch++) {
    uint8_t val = ctx.fft_bin(ch);
    int32_t bar_height = (static_cast<int32_t>(val) * band_width) / 255;
    int32_t start = static_cast<int32_t>(ch) * band_width;

    uint8_t pal_idx = static_cast<uint8_t>(ch * 16);
    for (int32_t j = 0; j < bar_height && start + j < ctx.len; j++) {
      uint8_t bri = static_cast<uint8_t>((static_cast<uint32_t>(j + 1) * 255) / static_cast<uint32_t>(band_width));
      ctx.set_pixel(start + j, ctx.pal_color(pal_idx, bri));
    }
  }
}

// Freqmap — map the loudest frequency across the strip.
void fx_freqmap(EffectContext &ctx) {
  float peak = ctx.major_peak();
  float vol = ctx.volume();

  ctx.fade_to_black(230);

  if (vol < 1.0f)
    return;

  float frac = peak / 5000.0f;
  if (frac > 1.0f)
    frac = 1.0f;
  int32_t pos = static_cast<int32_t>(frac * static_cast<float>(ctx.len - 1));
  uint8_t bri = static_cast<uint8_t>(vol);
  uint8_t hue = static_cast<uint8_t>(frac * 255.0f);

  ctx.set_pixel(pos, ctx.pal_color(hue, bri));
  if (pos > 0)
    ctx.set_pixel(pos - 1, ctx.pal_color(hue, bri / 2));
  if (pos < ctx.len - 1)
    ctx.set_pixel(pos + 1, ctx.pal_color(hue, bri / 2));
}

// Blurz — flash an FFT bin per frame and blur/fade.
void fx_blurz(EffectContext &ctx) {
  ctx.fade_to_black(220);

  for (uint8_t ch = 0; ch < AUDIO_GEQ_CHANNELS; ch++) {
    uint8_t val = ctx.fft_bin(ch);
    if (val < 16)
      continue;
    int32_t pos = (static_cast<int32_t>(ch) * ctx.len) / AUDIO_GEQ_CHANNELS;
    uint8_t pal_idx = static_cast<uint8_t>(ch * 16 + (ctx.now >> 4));
    ctx.set_pixel(pos, ctx.pal_color(pal_idx, val));
  }
}

// DJLight — effect emanating from center to edges based on FFT.
void fx_djlight(EffectContext &ctx) {
  ctx.fade_to_black(230);

  int32_t center = ctx.len / 2;

  for (uint8_t ch = 0; ch < AUDIO_GEQ_CHANNELS; ch++) {
    uint8_t val = ctx.fft_bin(ch);
    if (val < 10)
      continue;

    int32_t offset = (static_cast<int32_t>(ch) * center) / AUDIO_GEQ_CHANNELS;
    uint8_t pal_idx = static_cast<uint8_t>(ch * 16);
    uint32_t col = ctx.pal_color(pal_idx, val);

    if (center + offset < ctx.len)
      ctx.set_pixel(center + offset, col);
    if (center - offset >= 0)
      ctx.set_pixel(center - offset, col);
  }
}

// Freqpixels — random pixels coloured by frequency.
void fx_freqpixels(EffectContext &ctx) {
  ctx.fade_to_black(210);

  for (uint8_t ch = 0; ch < AUDIO_GEQ_CHANNELS; ch++) {
    uint8_t val = ctx.fft_bin(ch);
    if (val < 32)
      continue;

    int32_t pos = hw_random16(static_cast<uint16_t>(ctx.len));
    uint8_t hue = static_cast<uint8_t>(ch * 16);
    ctx.set_pixel(pos, ctx.pal_color(hue, val));
  }
}

// Freqwave — frequency to HSV colour mapping as a running wave.
void fx_freqwave(EffectContext &ctx) {
  float peak = ctx.major_peak();
  float vol = ctx.volume();

  // Shift pixels to the right.
  for (int32_t i = ctx.len - 1; i > 0; i--) {
    int32_t dst = ctx.map_pixel(i);
    int32_t src = ctx.map_pixel(i - 1);
    if (dst >= 0 && src >= 0 && dst < static_cast<int32_t>(ctx.frame_len) && src < static_cast<int32_t>(ctx.frame_len))
      ctx.frame_buf[dst] = ctx.frame_buf[src];
  }

  if (vol < 1.0f) {
    ctx.set_pixel(0, 0);
    return;
  }

  float frac = peak / 5120.0f;
  if (frac > 1.0f)
    frac = 1.0f;
  uint8_t hue = static_cast<uint8_t>(frac * 255.0f);
  uint8_t bri = static_cast<uint8_t>(vol);
  ctx.set_pixel(0, ctx.pal_color(hue, bri));
}

// Gravfreq — VU meter using major peak frequency for color.
void fx_gravfreq(EffectContext &ctx) {
  float vol = ctx.volume();
  float peak = ctx.major_peak();

  ctx.fade_to_black(240);

  float gravity = 0.015f + static_cast<float>(ctx.params->speed) * 0.0002f;
  float peak_pos = static_cast<float>(ctx.env->step) / 256.0f;

  float target = vol / 255.0f;
  if (target > peak_pos) {
    peak_pos = target;
    ctx.env->aux0 = 0;
  } else {
    float vel = static_cast<float>(ctx.env->aux0) / 256.0f;
    vel += gravity;
    peak_pos -= vel;
    if (peak_pos < 0)
      peak_pos = 0;
    ctx.env->aux0 = static_cast<uint16_t>(vel * 256.0f);
  }
  ctx.env->step = static_cast<uint32_t>(peak_pos * 256.0f);

  float frac = peak / 5000.0f;
  if (frac > 1.0f)
    frac = 1.0f;
  uint8_t hue = static_cast<uint8_t>(frac * 255.0f);

  int32_t bar_len = static_cast<int32_t>(peak_pos * static_cast<float>(ctx.len));
  for (int32_t i = 0; i < bar_len && i < ctx.len; i++)
    ctx.set_pixel(i, ctx.pal_color(hue));

  if (bar_len < ctx.len)
    ctx.set_pixel(bar_len, RGBW32(255, 255, 255, 0));
}

// Noisemove — perlin noise as movement for different frequency bins.
void fx_noisemove(EffectContext &ctx) {
  for (int32_t i = 0; i < ctx.len; i++) {
    uint8_t noise = inoise8(static_cast<uint16_t>(i * 40), ctx.now >> 3);
    ctx.set_pixel(i, ctx.pal_color(noise, noise >> 1));
  }

  for (uint8_t ch = 0; ch < AUDIO_GEQ_CHANNELS; ch++) {
    uint8_t val = ctx.fft_bin(ch);
    if (val < 16)
      continue;
    int32_t pos = inoise8(static_cast<uint16_t>(ch * 1000 + ctx.now / 4));
    pos = (pos * ctx.len) >> 8;
    if (pos >= ctx.len)
      pos = ctx.len - 1;
    ctx.set_pixel(pos, ctx.pal_color(static_cast<uint8_t>(ch * 16), val));
  }
}

// Waterfall — frequency data scrolling down the strip.
void fx_waterfall(EffectContext &ctx) {
  // Shift pixels down.
  for (int32_t i = 0; i < ctx.len - 1; i++) {
    int32_t dst = ctx.map_pixel(i);
    int32_t src = ctx.map_pixel(i + 1);
    if (dst >= 0 && src >= 0 && dst < static_cast<int32_t>(ctx.frame_len) && src < static_cast<int32_t>(ctx.frame_len))
      ctx.frame_buf[dst] = ctx.frame_buf[src];
  }

  // Add new row from FFT mid-band.
  uint8_t mid = 0;
  for (uint8_t ch = 3; ch < 12; ch++)
    mid = mid > ctx.fft_bin(ch) ? mid : ctx.fft_bin(ch);

  uint8_t hue = static_cast<uint8_t>(ctx.major_peak() / 20.0f);
  ctx.set_pixel(ctx.len - 1, ctx.pal_color(hue, mid));
}

// Rocktaves — musical octaves mapped to strip sections.
void fx_rocktaves(EffectContext &ctx) {
  ctx.fade_to_black(230);

  float peak = ctx.major_peak();
  if (peak < 20.0f)
    return;

  float octave = log2f(peak / 55.0f);
  if (octave < 0)
    octave = 0;
  if (octave > 7.0f)
    octave = 7.0f;

  float frac = octave / 7.0f;
  int32_t pos = static_cast<int32_t>(frac * static_cast<float>(ctx.len - 1));
  uint8_t hue = static_cast<uint8_t>(frac * 255.0f);
  uint8_t bri = static_cast<uint8_t>(ctx.volume());
  if (bri < 32)
    bri = 32;

  for (int32_t w = -2; w <= 2; w++) {
    int32_t px = pos + w;
    if (px >= 0 && px < ctx.len)
      ctx.set_pixel(px, ctx.pal_color(hue, bri));
  }
}

#ifdef WLED_BRIDGE_2D

// 2D GEQ — graphic equalizer on a matrix.
void fx_2d_geq(EffectContext &ctx) {
  if (!ctx.is_2d())
    return;

  uint16_t w = ctx.matrix_w;
  uint16_t h = ctx.matrix_h;

  ctx.fill_black();

  uint8_t bands = (w < AUDIO_GEQ_CHANNELS) ? w : AUDIO_GEQ_CHANNELS;
  uint16_t col_width = w / bands;

  for (uint8_t ch = 0; ch < bands; ch++) {
    uint8_t val = ctx.fft_bin(ch);
    int32_t bar_height = (static_cast<int32_t>(val) * h) / 255;

    uint8_t pal_idx = static_cast<uint8_t>(ch * (256 / bands));
    for (int32_t y = 0; y < bar_height && y < static_cast<int32_t>(h); y++) {
      uint8_t bri = static_cast<uint8_t>((static_cast<uint32_t>(y + 1) * 255) / static_cast<uint32_t>(h));
      uint32_t col = ctx.pal_color(pal_idx, bri);
      for (uint16_t x = ch * col_width; x < (ch + 1) * col_width && x < w; x++)
        ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(h - 1 - y), col);
    }
  }
}

// 2D Waverly — noise waves modulated by volume.
void fx_2d_waverly_vol(EffectContext &ctx) {
  if (!ctx.is_2d())
    return;

  uint16_t w = ctx.matrix_w;
  uint16_t h = ctx.matrix_h;
  float vol = ctx.volume();

  ctx.fade_2d(230);

  int32_t mid_y = h / 2;
  float amp = (vol / 255.0f) * static_cast<float>(mid_y);

  for (uint16_t x = 0; x < w; x++) {
    uint8_t noise = inoise8(static_cast<uint16_t>(x * 30 + ctx.now / 5), ctx.now >> 3);
    int32_t offset = static_cast<int32_t>((static_cast<float>(noise) / 255.0f - 0.5f) * amp * 2.0f);
    int32_t y = mid_y + offset;
    if (y >= 0 && y < static_cast<int32_t>(h)) {
      uint8_t pal_idx = noise;
      ctx.set_pixel_2d(static_cast<int16_t>(x), static_cast<int16_t>(y), ctx.pal_color(pal_idx));
    }
  }
}

#endif  // WLED_BRIDGE_2D

}  // namespace wled_bridge
}  // namespace esphome

#endif  // WLED_BRIDGE_AUDIO
