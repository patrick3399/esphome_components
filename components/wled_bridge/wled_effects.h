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

static constexpr size_t WLED_EFFECT_COUNT = 30;
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

}  // namespace wled_bridge
}  // namespace esphome
