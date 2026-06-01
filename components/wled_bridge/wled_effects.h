#pragma once
#include <stddef.h>
#include "wled_effect_context.h"

namespace esphome {
namespace wled_bridge {

using EffectFn = void (*)(EffectContext &);

struct EffectDescriptor {
  const char *name;  // display name (also WLED effect name)
  const char *meta;  // WLED metadata string "Name@speed_label,intensity_label;col0,col1,col2;pal;flags"
  EffectFn fn;
};

static constexpr size_t WLED_EFFECT_COUNT = 84;
extern const EffectDescriptor WLED_EFFECTS[WLED_EFFECT_COUNT];

// ---- forward declarations ----
void fx_solid(EffectContext &);
void fx_blink(EffectContext &);
void fx_breathe(EffectContext &);
void fx_color_wipe(EffectContext &);
void fx_color_wipe_random(EffectContext &);
void fx_random_color(EffectContext &);
void fx_color_sweep(EffectContext &);
void fx_dynamic(EffectContext &);
void fx_colorloop(EffectContext &);
void fx_rainbow_cycle(EffectContext &);
void fx_scan(EffectContext &);
void fx_dual_scan(EffectContext &);
void fx_fade(EffectContext &);
void fx_theater_chase(EffectContext &);
void fx_theater_chase_rainbow(EffectContext &);
void fx_running_lights(EffectContext &);
void fx_saw(EffectContext &);
void fx_twinkle(EffectContext &);
void fx_twinkle_random(EffectContext &);
void fx_strobe(EffectContext &);
void fx_strobe_rainbow(EffectContext &);
void fx_bpm(EffectContext &);
void fx_larson_scanner(EffectContext &);
void fx_comet(EffectContext &);
void fx_fire_2012(EffectContext &);
void fx_meteor(EffectContext &);
void fx_meteor_smooth(EffectContext &);
void fx_noise1d(EffectContext &);
void fx_palette(EffectContext &);
void fx_ripple(EffectContext &);
// --- batch 2 ---
void fx_juggle(EffectContext &);
void fx_bouncing_balls(EffectContext &);
void fx_fireworks(EffectContext &);
void fx_police(EffectContext &);
void fx_chase_flash(EffectContext &);
void fx_heartbeat(EffectContext &);
void fx_rain(EffectContext &);
void fx_sparkle(EffectContext &);
void fx_pride_2015(EffectContext &);
void fx_candle(EffectContext &);
void fx_fill_noise(EffectContext &);
void fx_oscillate(EffectContext &);
void fx_gradient(EffectContext &);
void fx_pacifica(EffectContext &);

// --- batch 3 ---
void fx_sinelon(EffectContext &);
void fx_dissolve(EffectContext &);
void fx_android(EffectContext &);
void fx_chase_rainbow(EffectContext &);
void fx_colorful(EffectContext &);
void fx_fire_flicker(EffectContext &);
void fx_two_dots(EffectContext &);
void fx_tricolor_chase(EffectContext &);
void fx_icu(EffectContext &);
void fx_lightning(EffectContext &);
void fx_glitter(EffectContext &);
void fx_solid_glitter(EffectContext &);
void fx_spots(EffectContext &);
void fx_percent(EffectContext &);
void fx_flow(EffectContext &);
void fx_phased(EffectContext &);
void fx_sinewave(EffectContext &);
void fx_twinkleup(EffectContext &);
void fx_colorwaves(EffectContext &);
void fx_chunchun(EffectContext &);

// --- batch 4 ---
void fx_stream(EffectContext &);
void fx_scanner_dual(EffectContext &);
void fx_fairy(EffectContext &);
void fx_fairytwinkle(EffectContext &);
void fx_tri_wipe(EffectContext &);
void fx_tri_fade(EffectContext &);
void fx_multi_comet(EffectContext &);
void fx_colortwinkles(EffectContext &);
void fx_lake(EffectContext &);
void fx_railway(EffectContext &);
void fx_halloween_eyes(EffectContext &);
void fx_plasma(EffectContext &);
void fx_sunrise(EffectContext &);
void fx_washing_machine(EffectContext &);
void fx_blends(EffectContext &);
void fx_strobe_mega(EffectContext &);
void fx_chase_color(EffectContext &);
void fx_chase_color_random(EffectContext &);
void fx_solid_pattern_tri(EffectContext &);
void fx_solid_pattern(EffectContext &);

}  // namespace wled_bridge
}  // namespace esphome
