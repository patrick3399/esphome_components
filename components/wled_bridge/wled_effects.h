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

static constexpr size_t WLED_EFFECT_COUNT = 167;
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

// --- batch 5 ---
void fx_blink_rainbow(EffectContext &);
void fx_dynamic_smooth(EffectContext &);
void fx_running_dual(EffectContext &);
void fx_traffic_light(EffectContext &);
void fx_loading(EffectContext &);
void fx_stream2(EffectContext &);
void fx_spots_fade(EffectContext &);
void fx_sinelon_dual(EffectContext &);
void fx_sinelon_rainbow(EffectContext &);
void fx_ripple_rainbow(EffectContext &);
void fx_candle_multi(EffectContext &);
void fx_noise_pal(EffectContext &);
void fx_twinklefox(EffectContext &);
void fx_flow_stripe(EffectContext &);
void fx_twinklecat(EffectContext &);
void fx_dissolve_random(EffectContext &);
void fx_sweep_random(EffectContext &);
void fx_chase_flash_random(EffectContext &);

// --- batch 6 ---
void fx_sparkle_dark(EffectContext &);
void fx_sparkle_plus(EffectContext &);
void fx_chase2(EffectContext &);
void fx_rainbow_runner(EffectContext &);
void fx_noise1(EffectContext &);
void fx_noise2(EffectContext &);
void fx_noise3(EffectContext &);
void fx_noise4(EffectContext &);
void fx_phased_noise(EffectContext &);
void fx_wavesins(EffectContext &);
// --- batch 7 ---
void fx_shimmer(EffectContext &);
void fx_aurora(EffectContext &);
void fx_tetrix(EffectContext &);
void fx_rolling_balls(EffectContext &);
void fx_starburst(EffectContext &);
void fx_fireworks_1d(EffectContext &);
void fx_popcorn(EffectContext &);
void fx_drip(EffectContext &);
void fx_dancing_shadows(EffectContext &);
void fx_tv_simulator(EffectContext &);

// --- 2D effects batch 1 ---
void fx_2d_matrix(EffectContext &);
void fx_2d_plasma(EffectContext &);
void fx_2d_gameoflife(EffectContext &);
void fx_2d_tartan(EffectContext &);
void fx_2d_waverly(EffectContext &);
void fx_2d_noise(EffectContext &);
void fx_2d_sindots(EffectContext &);
void fx_2d_lissajous(EffectContext &);
void fx_2d_frizzles(EffectContext &);
void fx_2d_blackhole(EffectContext &);
void fx_2d_sinesin(EffectContext &);
void fx_2d_julia(EffectContext &);
void fx_2d_metaballs(EffectContext &);
void fx_2d_fire(EffectContext &);
void fx_2d_gravity(EffectContext &);
// --- 2D effects batch 2 ---
void fx_2d_dna(EffectContext &);
void fx_2d_pulser(EffectContext &);
void fx_2d_drift(EffectContext &);
void fx_2d_noisefire(EffectContext &);
void fx_2d_noisemove(EffectContext &);
void fx_2d_colored_bursts(EffectContext &);
void fx_2d_yinyang(EffectContext &);
void fx_2d_fireworks2d(EffectContext &);
void fx_2d_sprouts(EffectContext &);
void fx_2d_zentangle(EffectContext &);
// --- 2D effects batch 3 ---
void fx_2d_akemi(EffectContext &);
void fx_2d_hopalong(EffectContext &);
void fx_2d_magnetics(EffectContext &);
void fx_2d_hypnotic(EffectContext &);
void fx_2d_bubbles(EffectContext &);
void fx_2d_magnifying(EffectContext &);
void fx_2d_popcorn2d(EffectContext &);
void fx_2d_sparkle2d(EffectContext &);
void fx_2d_pendulum(EffectContext &);
void fx_2d_heatmap(EffectContext &);
// --- 2D effects batch 4 ---
void fx_2d_squaredswirl(EffectContext &);
void fx_2d_sunradiation(EffectContext &);
void fx_2d_polarlights(EffectContext &);
void fx_2d_swirl(EffectContext &);
void fx_2d_dnaspiral(EffectContext &);
void fx_2d_plasmaball(EffectContext &);
void fx_2d_funkyplank(EffectContext &);
void fx_2d_plasmarotozoom(EffectContext &);
void fx_2d_distortionwaves(EffectContext &);
void fx_2d_soap(EffectContext &);

}  // namespace wled_bridge
}  // namespace esphome
