#include "wled_bridge.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/color.h"
#include "esphome/core/helpers.h"
#include "wled_color.h"

#include "wled_json.h"
#include "wled_ui_data.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include <algorithm>
#include <stdio.h>
#include <string.h>

namespace esphome {
namespace wled_bridge {

// ============================================================
// WLEDProxyEffect
// ============================================================
void WLEDProxyEffect::start() {
  light::AddressableLightEffect::start();  // sets effect_active = true
  comp_->set_effect(effect_id_);
}

// ============================================================
// WLEDBridgeComponent
// ============================================================
static const char *const TAG = "wled_bridge";
static constexpr uint32_t WLED_PRESET_MAGIC = 0x574C5033;  // WLP3
static constexpr uint32_t WLED_STATE_MAGIC = 0x574C5332;  // WLS2
static constexpr uint32_t WLED_STATE_SAVE_DELAY_MS = 2000;

static uint8_t scale8_linear(uint8_t value, uint8_t scale) {
  return static_cast<uint8_t>((static_cast<uint16_t>(value) * scale) / 255u);
}

static uint8_t unit_to_u8(float value) {
  if (value <= 0.0f)
    return 0;
  if (value >= 1.0f)
    return 255;
  return static_cast<uint8_t>(value * 255.0f + 0.5f);
}

// Derive a white channel from RGB for RGBW strips (WLED-style auto white modes).
static uint32_t apply_auto_white(uint32_t c, uint8_t mode) {
  if (mode == 0)
    return c;
  uint8_t r = R(c), g = G(c), b = B(c), w = W(c);
  uint8_t lo = std::min<uint8_t>(r, std::min<uint8_t>(g, b));
  switch (mode) {
    case 1:  // brighter: keep RGB, add white from the common floor
      w = qadd8(w, lo);
      break;
    case 2:  // accurate: pull the white component out of RGB
      r = static_cast<uint8_t>(r - lo);
      g = static_cast<uint8_t>(g - lo);
      b = static_cast<uint8_t>(b - lo);
      w = qadd8(w, lo);
      break;
    case 4: {  // max: white tracks the brightest channel
      uint8_t hi = std::max<uint8_t>(r, std::max<uint8_t>(g, b));
      w = qadd8(w, hi);
      break;
    }
    default:
      w = qadd8(w, lo);
      break;
  }
  return RGBW32(r, g, b, w);
}

// ============================================================
// add_bus — called from generated code once per bus
// ============================================================
void WLEDBridgeComponent::add_bus(light::LightState *state, uint32_t max_ma, uint32_t led_ma) {
  VirtualBus bus;
  bus.light_state = state;
  bus.strip = nullptr;  // resolved in setup() after strips are initialized
  bus.start = 0;  // calculated in setup()
  bus.len = 0;  // calculated in setup()
  bus.max_ma = max_ma;
  bus.led_ma = led_ma;
  this->buses_.push_back(bus);

  if (this->light_state_ == nullptr) {
    // First bus → primary: used for HA sync and effect registration
    this->light_state_ = state;
  }
}

// ============================================================
// setup
// ============================================================
void WLEDBridgeComponent::setup() {
  if (this->buses_.empty()) {
    ESP_LOGE(TAG, "No buses configured — add light_id or buses: in YAML");
    this->mark_failed();
    return;
  }

  // Resolve strips and build virtual LED space
  uint32_t virtual_pos = 0;
  for (auto &bus : this->buses_) {
    bus.strip = static_cast<light::AddressableLight *>(bus.light_state->get_output());
    bus.start = virtual_pos;
    bus.len = static_cast<uint32_t>(bus.strip->size());
    virtual_pos += bus.len;

    // Each bus: disable ESPHome colour correction (bridge handles gamma itself)
    // and claim pixel ownership.
    bus.strip->set_correction(1.0f, 1.0f, 1.0f);
    bus.strip->set_effect_active(true);
  }
  this->total_leds_ = virtual_pos;
  this->light_ = this->buses_[0].strip;  // primary alias for WLEDProxyEffect

  if (this->total_leds_ == 0) {
    ESP_LOGE(TAG, "All buses report 0 LEDs");
    this->mark_failed();
    return;
  }

  this->segment_start_ = 0;
  this->segment_stop_ = this->total_leds_;

  // Default effect parameters
  this->params_.speed = 128;
  this->params_.intensity = 128;
  this->params_.colors[0] = 0xFFAA00;
  this->params_.colors[1] = 0x000000;
  this->params_.colors[2] = 0x000000;
  this->params_.palette_id = 0;
  this->load_presets_();
  this->load_state_();

  this->light_state_->add_remote_values_listener(this);
  if (this->state_loaded_) {
    this->publish_light_state();
  } else {
    this->sync_from_light_state_(false);
  }

  // Allocate virtual frame buffers (PSRAM-preferred, internal heap fallback)
  size_t buf_bytes = this->total_leds_ * sizeof(uint32_t);
  this->full_frame_ = static_cast<uint32_t *>(heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (this->full_frame_ == nullptr)
    this->full_frame_ = static_cast<uint32_t *>(malloc(buf_bytes));
  if (this->full_frame_ == nullptr) {
    ESP_LOGE(TAG, "Unable to allocate full-frame buffer (%u LEDs)", this->total_leds_);
    this->mark_failed();
    return;
  }
  memset(this->full_frame_, 0, buf_bytes);

  this->transition_frame_ = static_cast<uint32_t *>(heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (this->transition_frame_ == nullptr)
    this->transition_frame_ = static_cast<uint32_t *>(malloc(buf_bytes));
  if (this->transition_frame_ == nullptr) {
    ESP_LOGE(TAG, "Unable to allocate transition buffer (%u LEDs)", this->total_leds_);
    this->mark_failed();
    return;
  }
  memset(this->transition_frame_, 0, buf_bytes);

  // Per-LED opacity map (segment brightness / on-off). Internal RAM is fine —
  // one byte per LED, accessed every flush.
  this->pixel_opacity_ = static_cast<uint8_t *>(malloc(this->total_leds_));
  if (this->pixel_opacity_ == nullptr) {
    ESP_LOGE(TAG, "Unable to allocate opacity map (%u LEDs)", this->total_leds_);
    this->mark_failed();
    return;
  }
  memset(this->pixel_opacity_, 255, this->total_leds_);
  this->opacity_map_dirty_ = true;

  // Zero all strips
  for (auto &bus : this->buses_) {
    for (int32_t i = 0; i < static_cast<int32_t>(bus.len); i++)
      (*bus.strip)[i].set(esphome::Color::BLACK);
    bus.strip->schedule_show();
  }

  // Register web handlers
  auto *wsb = web_server_base::global_web_server_base;
  if (wsb != nullptr) {
    this->json_handler_ = new WLEDJsonHandler(this);  // NOLINT
    this->sse_handler_ = new WLEDSseHandler(this);  // NOLINT
    this->ui_handler_ = new WLEDUiHandler();  // NOLINT
    wsb->get_server()->addHandler(this->ui_handler_);
    wsb->get_server()->addHandler(this->json_handler_);
    wsb->get_server()->addHandler(this->sse_handler_);
    ESP_LOGD(TAG, "WLED JSON API registered");
  } else {
    ESP_LOGW(TAG, "web_server_base not available — JSON API disabled");
  }

  if (this->use_task_) {
    xTaskCreatePinnedToCore(render_task_fn_, "wled_render", 8192, this, 5, &this->render_task_, APP_CPU_NUM);
    ESP_LOGD(TAG, "Render task started on core 1");
  }

  // Register every WLED effect as an ESPHome proxy effect (primary bus only)
  for (size_t i = 0; i < WLED_EFFECT_COUNT; i++) {
    auto *proxy = new WLEDProxyEffect(WLED_EFFECTS[i].name, static_cast<uint8_t>(i), this);  // NOLINT
    proxy->init_internal(this->light_state_);
    this->light_state_->add_effects({proxy});
  }

  ESP_LOGCONFIG(TAG, "WLED Bridge ready: %u virtual LEDs across %zu bus(es), %zu effects", this->total_leds_,
                this->buses_.size(), WLED_EFFECT_COUNT);
}

void WLEDBridgeComponent::load_presets_() {
  this->preset_store_.magic = WLED_PRESET_MAGIC;
  uint32_t key = fnv1a_hash("wled_bridge_presets");
  this->preset_pref_ = global_preferences->make_preference<WLEDPresetStore>(key, true);
  WLEDPresetStore recovered;
  if (this->preset_pref_.load(&recovered) && recovered.magic == WLED_PRESET_MAGIC) {
    this->preset_store_ = recovered;
    ESP_LOGD(TAG, "Loaded WLED bridge presets");
  }
}

void WLEDBridgeComponent::persist_presets_() {
  this->preset_store_.magic = WLED_PRESET_MAGIC;
  this->preset_pref_.save(&this->preset_store_);
  global_preferences->sync();
}

void WLEDBridgeComponent::load_state_() {
  uint32_t key = fnv1a_hash("wled_bridge_state");
  this->state_pref_ = global_preferences->make_preference<WLEDStoredState>(key, true);
  this->state_pref_ready_ = true;

  WLEDStoredState recovered;
  if (!this->state_pref_.load(&recovered) || recovered.magic != WLED_STATE_MAGIC || recovered.state.valid == 0)
    return;
  if (recovered.state.effect >= WLED_EFFECT_COUNT)
    return;

  this->apply_preset_(recovered.state);
  this->state_loaded_ = true;
  ESP_LOGD(TAG, "Restored WLED bridge state");
}

void WLEDBridgeComponent::persist_state_() {
  if (!this->state_pref_ready_)
    return;
  WLEDStoredState state;
  state.magic = WLED_STATE_MAGIC;
  state.state = this->current_as_preset_();
  this->state_pref_.save(&state);
  global_preferences->sync();
  ESP_LOGV(TAG, "Persisted WLED bridge state");
}

void WLEDBridgeComponent::schedule_state_save_() {
  if (!this->state_pref_ready_)
    return;
  this->state_save_due_ms_ = millis() + WLED_STATE_SAVE_DELAY_MS;
}

WLEDPresetRecord WLEDBridgeComponent::current_as_preset_() const {
  WLEDPresetRecord preset;
  preset.valid = 1;
  preset.on = this->is_on_ ? 1 : 0;
  preset.brightness = this->global_bri_;
  preset.effect = this->active_fx_;
  preset.speed = this->params_.speed;
  preset.intensity = this->params_.intensity;
  preset.custom1 = this->params_.custom1;
  preset.custom2 = this->params_.custom2;
  preset.custom3 = this->params_.custom3;
  preset.check1 = this->params_.check1 ? 1 : 0;
  preset.check2 = this->params_.check2 ? 1 : 0;
  preset.check3 = this->params_.check3 ? 1 : 0;
  preset.palette = this->params_.palette_id;
  preset.segment_start = static_cast<uint16_t>(this->segment_start_);
  preset.segment_stop = static_cast<uint16_t>(this->segment_stop_);
  preset.reverse = this->segment_reverse_ ? 1 : 0;
  preset.mirror = this->segment_mirror_ ? 1 : 0;
  preset.transition_ms = this->transition_ms_;
  preset.colors[0] = this->params_.colors[0];
  preset.colors[1] = this->params_.colors[1];
  preset.colors[2] = this->params_.colors[2];
  return preset;
}

void WLEDBridgeComponent::copy_preset_name_(WLEDPresetRecord *preset, const char *name) const {
  if (preset == nullptr || name == nullptr || name[0] == '\0')
    return;
  snprintf(preset->name, sizeof(preset->name), "%s", name);
}

void WLEDBridgeComponent::set_default_preset_name_(WLEDPresetRecord *preset, uint8_t preset_id) const {
  if (preset == nullptr || preset->name[0] != '\0')
    return;
  snprintf(preset->name, sizeof(preset->name), "Preset %u", preset_id);
}

void WLEDBridgeComponent::apply_preset_(const WLEDPresetRecord &preset) {
  bool effect_changed = preset.effect != this->active_fx_;
  bool segment_changed = preset.segment_start != this->segment_start_ || preset.segment_stop != this->segment_stop_ ||
                         (preset.reverse != 0) != this->segment_reverse_ ||
                         (preset.mirror != 0) != this->segment_mirror_;
  this->is_on_ = preset.on != 0;
  this->global_bri_ = preset.brightness;
  this->active_fx_ = preset.effect;
  this->params_.speed = preset.speed;
  this->params_.intensity = preset.intensity;
  this->params_.custom1 = preset.custom1;
  this->params_.custom2 = preset.custom2;
  this->params_.custom3 = preset.custom3;
  this->params_.check1 = preset.check1 != 0;
  this->params_.check2 = preset.check2 != 0;
  this->params_.check3 = preset.check3 != 0;
  this->params_.palette_id = preset.palette;
  this->segment_start_ = std::min<uint32_t>(preset.segment_start, this->total_leds_ > 0 ? this->total_leds_ - 1 : 0);
  this->segment_stop_ = std::min<uint32_t>(preset.segment_stop, this->total_leds_);
  if (this->segment_stop_ <= this->segment_start_)
    this->segment_stop_ = this->total_leds_;
  this->segment_reverse_ = preset.reverse != 0;
  this->segment_mirror_ = preset.mirror != 0;
  this->transition_ms_ = preset.transition_ms;
  this->params_.colors[0] = preset.colors[0];
  this->params_.colors[1] = preset.colors[1];
  this->params_.colors[2] = preset.colors[2];
  if (effect_changed || segment_changed)
    this->reset_segment_state_();
}

// ============================================================
// on_light_remote_values_update
// ============================================================
void WLEDBridgeComponent::on_light_remote_values_update() {
  if (this->suppress_light_sync_)
    return;
  this->sync_from_light_state_(true);
}

// ============================================================
// loop
// ============================================================
void WLEDBridgeComponent::loop() {
  // Re-assert effect ownership on all buses — ESPHome effect lifecycle may
  // clear effect_active when HA changes or removes an effect.
  for (auto &bus : this->buses_) {
    if (bus.strip != nullptr && !bus.strip->is_effect_active())
      bus.strip->set_effect_active(true);
  }

  if (!this->use_task_) {
    uint32_t now = millis();
    if (now - this->last_frame_ms_ >= FRAMETIME_MS) {
      this->render_frame_();
      this->last_frame_ms_ = now;
    }
  }

  // Broadcast state change via SSE
  if (this->state_dirty_ && this->sse_handler_ != nullptr) {
    this->sse_handler_->broadcast_state();
    this->state_dirty_ = false;
  }
  if (this->state_save_due_ms_ != 0 && static_cast<int32_t>(millis() - this->state_save_due_ms_) >= 0) {
    this->persist_state_();
    this->state_save_due_ms_ = 0;
  }
}

void WLEDBridgeComponent::begin_transition_() {
  if (this->transition_ms_ == 0 || this->transition_frame_ == nullptr || this->full_frame_ == nullptr ||
      this->transition_active_ || this->total_leds_ == 0)
    return;

  // Snapshot the current unscaled rendered frame as transition source.
  memcpy(this->transition_frame_, this->full_frame_, this->total_leds_ * sizeof(uint32_t));
  this->transition_start_ms_ = millis();
  this->transition_duration_ms_ = this->transition_ms_;
  this->transition_active_ = true;
}

// ============================================================
// sync_from_light_state_ — inbound Home Assistant / ESPHome light state
// ============================================================
void WLEDBridgeComponent::sync_from_light_state_(bool publish) {
  if (this->light_state_ == nullptr)
    return;

  const auto &values = this->light_state_->remote_values;
  bool changed = false;

  bool on = values.is_on();
  if (this->is_on_ != on) {
    this->is_on_ = on;
    changed = true;
  }

  uint8_t bri = unit_to_u8(values.get_brightness());
  if (this->global_bri_ != bri) {
    this->global_bri_ = bri;
    changed = true;
  }

  uint8_t color_bri = unit_to_u8(values.get_color_brightness());
  uint8_t r = scale8_linear(unit_to_u8(values.get_red()), color_bri);
  uint8_t g = scale8_linear(unit_to_u8(values.get_green()), color_bri);
  uint8_t b = scale8_linear(unit_to_u8(values.get_blue()), color_bri);
  uint8_t w = unit_to_u8(values.get_white());
  uint32_t rgbw = RGBW32(r, g, b, w);
  if (this->params_.colors[0] != rgbw) {
    this->params_.colors[0] = rgbw;
    changed = true;
  }

  if (changed && publish)
    this->mark_dirty_();
}

// ============================================================
// publish_light_state — outbound WLED UI / JSON state to ESPHome
// ============================================================
void WLEDBridgeComponent::publish_light_state() {
  if (this->light_state_ == nullptr)
    return;

  const uint32_t c = this->params_.colors[0];
  auto call = this->light_state_->make_call();
  call.set_state(this->is_on_);
  call.set_brightness(static_cast<float>(this->global_bri_) / 255.0f);
  call.set_color_brightness(1.0f);
  call.set_rgbw(static_cast<float>(R(c)) / 255.0f, static_cast<float>(G(c)) / 255.0f, static_cast<float>(B(c)) / 255.0f,
                static_cast<float>(W(c)) / 255.0f);
  call.set_transition_length(0);

  this->suppress_light_sync_ = true;
  call.perform();
  this->suppress_light_sync_ = false;
  this->reset_output_correction_();
}

// ============================================================
// render_frame_ — frame-buffer pipeline (multi-bus aware)
//
// Pipeline:
//   full_frame_[]  ← unscaled, persists between frames (enables fade effects)
//        │
//   effect writes into full_frame_[] via EffectContext
//        │
//   apply_transition_blend_() blends on full_frame_[] (still unscaled)
//        │
//   compute_output_scale_()  → final_scale (brightness * ABL)
//        │
//   reset_output_correction_()  → prevent ESPHome from re-scaling our output
//        │
//   flush_frame_to_buses_()  → scale(full_frame_) → each bus strip → schedule_show
// ============================================================
void WLEDBridgeComponent::render_frame_() {
  if (this->buses_.empty() || this->full_frame_ == nullptr)
    return;

  uint32_t now = millis();

  if (!this->is_on_) {
    memset(this->full_frame_, 0, this->total_leds_ * sizeof(uint32_t));
    this->apply_transition_blend_(now);
    this->reset_output_correction_();
    this->flush_frame_to_buses_(255u);
    return;
  }

  // full_frame_[] already holds the unscaled result from the last frame.
  // Effects that fade (comet, larson…) read previous values and write faded
  // versions — no separate "restore" step needed.

  // Main segment (id 0) — state lives in scalar members.
  SegmentView main_view{static_cast<int32_t>(this->segment_start_),
                        static_cast<int32_t>(this->segment_stop_),
                        this->segment_reverse_,
                        this->segment_mirror_,
                        this->main_grouping_,
                        this->main_spacing_,
                        &this->params_,
                        &this->env_,
                        this->active_fx_};
  this->render_segment_(main_view, now);

  // Extra segments (id 1..)
  for (uint8_t i = 0; i < this->extra_count_; i++) {
    ExtraSegment &seg = this->extra_segments_[i];
    if (!seg.on || seg.stop <= seg.start)
      continue;  // off / empty segments are masked black by the opacity map
    SegmentView view{static_cast<int32_t>(seg.start),
                     static_cast<int32_t>(seg.stop),
                     seg.reverse,
                     seg.mirror,
                     seg.grouping,
                     seg.spacing,
                     &seg.params,
                     &seg.env,
                     seg.mode};
    this->render_segment_(view, now);
  }

  if (this->opacity_map_dirty_)
    this->rebuild_opacity_map_();

  this->apply_transition_blend_(now);
  uint8_t final_scale = this->compute_output_scale_();
  this->reset_output_correction_();
  this->flush_frame_to_buses_(final_scale);
}

// Render a single segment into full_frame_ over its physical range.
void WLEDBridgeComponent::render_segment_(const SegmentView &view, uint32_t now) {
  int32_t phys_len = view.stop - view.start;
  if (phys_len <= 0 || view.start < 0 || view.stop > static_cast<int32_t>(this->total_leds_))
    return;
  int32_t group = view.grouping < 1 ? 1 : view.grouping;
  int32_t step = group + view.spacing;
  int32_t vlen = (phys_len + step - 1) / step;

  EffectContext ctx{this->full_frame_, this->total_leds_, view.params,   view.env,    now, view.start, view.stop, vlen,
                    view.reverse,      view.mirror,       view.grouping, view.spacing};

  if (view.mode < WLED_EFFECT_COUNT)
    WLED_EFFECTS[view.mode].fn(ctx);

  view.env->call++;
}

// Rebuild per-LED opacity from segment coverage. Pixels not covered by any
// segment stay black (opacity 0), matching WLED's behavior.
void WLEDBridgeComponent::rebuild_opacity_map_() {
  if (this->pixel_opacity_ == nullptr || this->total_leds_ == 0)
    return;
  memset(this->pixel_opacity_, 0, this->total_leds_);

  // Main segment (its on/off is the global power state).
  uint8_t main_op = this->main_opacity_;
  for (uint32_t i = this->segment_start_; i < this->segment_stop_ && i < this->total_leds_; i++)
    this->pixel_opacity_[i] = main_op;

  // Extra segments paint over the main where they overlap (later wins).
  for (uint8_t s = 0; s < this->extra_count_; s++) {
    const ExtraSegment &seg = this->extra_segments_[s];
    uint8_t op = seg.on ? seg.opacity : 0;
    for (uint32_t i = seg.start; i < seg.stop && i < this->total_leds_; i++)
      this->pixel_opacity_[i] = op;
  }
  this->opacity_map_dirty_ = false;
}

ExtraSegment *WLEDBridgeComponent::resolve_extra_(uint8_t id) {
  if (id == 0 || id > this->extra_count_)
    return nullptr;
  return &this->extra_segments_[id - 1];
}

// ============================================================
// reset_output_correction_ — prevent ESPHome from re-scaling our output
// ============================================================
void WLEDBridgeComponent::reset_output_correction_() {
  for (auto &bus : this->buses_) {
    if (bus.light_state == nullptr || bus.strip == nullptr)
      continue;
    bus.light_state->current_values.set_state(true);
    bus.light_state->current_values.set_brightness(1.0f);
    bus.strip->update_state(bus.light_state);
  }
}

// ============================================================
// compute_output_scale_ — brightness + ABL → single 0-255 multiplier
// Does NOT modify full_frame_[] so unscaled values persist for next frame.
// ============================================================
uint8_t WLEDBridgeComponent::compute_output_scale_() {
  if (this->total_leds_ == 0)
    return this->global_bri_;

  // Estimate total current after applying segment opacity + global brightness
  uint64_t channel_sum = 0;
  for (uint32_t i = 0; i < this->total_leds_; i++) {
    uint32_t c = this->full_frame_[i];
    uint8_t op = this->pixel_opacity_ != nullptr ? this->pixel_opacity_[i] : 255;
    uint8_t bri = scale8_linear(op, this->global_bri_);
    channel_sum += scale8_linear(R(c), bri);
    channel_sum += scale8_linear(G(c), bri);
    channel_sum += scale8_linear(B(c), bri);
    channel_sum += scale8_linear(W(c), bri);
  }

  // Aggregate per-bus budget (0 = unlimited)
  uint32_t total_max_ma = 0;
  uint32_t primary_led_ma = 0;
  for (const auto &bus : this->buses_) {
    total_max_ma += bus.max_ma;
    if (primary_led_ma == 0)
      primary_led_ma = bus.led_ma;
  }

  if (primary_led_ma == 0) {
    this->current_ma_ = 0;
    return this->global_bri_;
  }

  uint32_t estimated_ma = static_cast<uint32_t>((channel_sum * primary_led_ma) / (255u * 3u));
  this->current_ma_ = estimated_ma;

  if (total_max_ma == 0 || estimated_ma <= total_max_ma)
    return this->global_bri_;

  // ABL: scale down to stay within budget
  uint8_t abl_scale = static_cast<uint8_t>((static_cast<uint64_t>(total_max_ma) * 255u) / estimated_ma);
  uint8_t final_scale = scale8_linear(this->global_bri_, abl_scale);
  // Re-estimate after ABL
  this->current_ma_ = static_cast<uint32_t>((static_cast<uint64_t>(estimated_ma) * final_scale) / 255u);
  return final_scale;
}

// ============================================================
// flush_frame_to_buses_ — write full_frame_[] (scaled on-the-fly) to strips
// ============================================================
void WLEDBridgeComponent::flush_frame_to_buses_(uint8_t final_scale) {
  for (const auto &bus : this->buses_) {
    if (bus.strip == nullptr)
      continue;
    for (uint32_t i = 0; i < bus.len; i++) {
      uint32_t idx = bus.start + i;
      uint32_t c = this->full_frame_[idx];
      if (this->auto_white_mode_ != 0)
        c = apply_auto_white(c, this->auto_white_mode_);
      uint8_t op = this->pixel_opacity_ != nullptr ? this->pixel_opacity_[idx] : 255;
      uint8_t scale = scale8_linear(op, final_scale);
      write_pixel((*bus.strip)[static_cast<int32_t>(i)],
                  RGBW32(scale8_linear(R(c), scale), scale8_linear(G(c), scale), scale8_linear(B(c), scale),
                         scale8_linear(W(c), scale)));
    }
    bus.strip->schedule_show();
  }
}

void WLEDBridgeComponent::apply_transition_blend_(uint32_t now) {
  if (!this->transition_active_ || this->transition_frame_ == nullptr || this->transition_duration_ms_ == 0)
    return;

  uint32_t elapsed = now - this->transition_start_ms_;
  if (elapsed >= this->transition_duration_ms_) {
    this->transition_active_ = false;
    return;
  }

  // Blend on unscaled full_frame_[] so the next compute_output_scale_() sees
  // correct values.
  uint8_t amount = static_cast<uint8_t>((elapsed * 255u) / this->transition_duration_ms_);
  for (uint32_t i = 0; i < this->total_leds_; i++) {
    this->full_frame_[i] = color_blend(this->transition_frame_[i], this->full_frame_[i], amount);
  }
}

// ============================================================
// State mutators
// ============================================================
void WLEDBridgeComponent::set_on(bool on) {
  this->is_on_ = on;
  this->mark_dirty_();
}

void WLEDBridgeComponent::set_brightness(uint8_t bri) {
  this->global_bri_ = bri;
  this->mark_dirty_();
}

void WLEDBridgeComponent::set_effect(uint8_t fx_index) {
  if (fx_index >= WLED_EFFECT_COUNT)
    return;
  if (fx_index != this->active_fx_) {
    this->begin_transition_();
    this->active_fx_ = fx_index;
    this->reset_segment_state_();
    this->mark_dirty_();
  }
}

bool WLEDBridgeComponent::is_preset_valid(uint8_t preset_id) const {
  if (preset_id == 0 || preset_id > WLED_PRESET_COUNT)
    return false;
  return this->preset_store_.slots[preset_id - 1].valid != 0;
}

bool WLEDBridgeComponent::save_preset(uint8_t preset_id, const char *name) {
  if (preset_id == 0 || preset_id > WLED_PRESET_COUNT)
    return false;
  const auto &previous = this->preset_store_.slots[preset_id - 1];
  WLEDPresetRecord preset = this->current_as_preset_();
  if (name != nullptr && name[0] != '\0') {
    this->copy_preset_name_(&preset, name);
  } else if (previous.valid != 0 && previous.name[0] != '\0') {
    this->copy_preset_name_(&preset, previous.name);
  }
  this->set_default_preset_name_(&preset, preset_id);
  this->preset_store_.slots[preset_id - 1] = preset;
  this->active_preset_ = preset_id;
  this->persist_presets_();
  this->mark_dirty_(false);
  return true;
}

bool WLEDBridgeComponent::load_preset(uint8_t preset_id) {
  if (!this->is_preset_valid(preset_id))
    return false;
  const auto &preset = this->preset_store_.slots[preset_id - 1];
  if (preset.effect >= WLED_EFFECT_COUNT)
    return false;
  this->begin_transition_();
  this->apply_preset_(preset);
  this->active_preset_ = preset_id;
  this->mark_dirty_(false);
  return true;
}

bool WLEDBridgeComponent::delete_preset(uint8_t preset_id) {
  if (preset_id == 0 || preset_id > WLED_PRESET_COUNT)
    return false;
  this->preset_store_.slots[preset_id - 1] = WLEDPresetRecord{};
  if (this->active_preset_ == preset_id)
    this->active_preset_ = 0;
  this->persist_presets_();
  this->mark_dirty_(false);
  return true;
}

void WLEDBridgeComponent::set_segment_bounds(uint32_t start, uint32_t stop) {
  if (this->total_leds_ == 0)
    return;
  start = std::min<uint32_t>(start, this->total_leds_ - 1);
  stop = std::min<uint32_t>(stop, this->total_leds_);
  if (stop <= start)
    stop = start + 1;
  if (this->segment_start_ == start && this->segment_stop_ == stop)
    return;
  this->begin_transition_();
  this->segment_start_ = start;
  this->segment_stop_ = stop;
  this->opacity_map_dirty_ = true;
  this->reset_segment_state_();
  this->mark_dirty_();
}

void WLEDBridgeComponent::set_segment_reverse(bool reverse) {
  if (this->segment_reverse_ == reverse)
    return;
  this->begin_transition_();
  this->segment_reverse_ = reverse;
  this->reset_segment_state_();
  this->mark_dirty_();
}

void WLEDBridgeComponent::set_segment_mirror(bool mirror) {
  if (this->segment_mirror_ == mirror)
    return;
  this->begin_transition_();
  this->segment_mirror_ = mirror;
  this->reset_segment_state_();
  this->mark_dirty_();
}

// ============================================================
// Multi-segment read view
// ============================================================
bool WLEDBridgeComponent::get_segment_view(uint8_t id, SegmentReadView &out) const {
  if (id == 0) {
    out.start = static_cast<uint16_t>(this->segment_start_);
    out.stop = static_cast<uint16_t>(this->segment_stop_);
    out.grouping = this->main_grouping_;
    out.spacing = this->main_spacing_;
    out.on = this->is_on_;
    out.reverse = this->segment_reverse_;
    out.mirror = this->segment_mirror_;
    out.selected = true;
    out.opacity = this->main_opacity_;
    out.mode = this->active_fx_;
    out.speed = this->params_.speed;
    out.intensity = this->params_.intensity;
    out.palette = this->params_.palette_id;
    out.custom1 = this->params_.custom1;
    out.custom2 = this->params_.custom2;
    out.custom3 = this->params_.custom3;
    out.check1 = this->params_.check1;
    out.check2 = this->params_.check2;
    out.check3 = this->params_.check3;
    out.colors[0] = this->params_.colors[0];
    out.colors[1] = this->params_.colors[1];
    out.colors[2] = this->params_.colors[2];
    return true;
  }
  if (id > this->extra_count_)
    return false;
  const ExtraSegment &seg = this->extra_segments_[id - 1];
  out.start = seg.start;
  out.stop = seg.stop;
  out.grouping = seg.grouping;
  out.spacing = seg.spacing;
  out.on = seg.on;
  out.reverse = seg.reverse;
  out.mirror = seg.mirror;
  out.selected = false;
  out.opacity = seg.opacity;
  out.mode = seg.mode;
  out.speed = seg.params.speed;
  out.intensity = seg.params.intensity;
  out.palette = seg.params.palette_id;
  out.custom1 = seg.params.custom1;
  out.custom2 = seg.params.custom2;
  out.custom3 = seg.params.custom3;
  out.check1 = seg.params.check1;
  out.check2 = seg.params.check2;
  out.check3 = seg.params.check3;
  out.colors[0] = seg.params.colors[0];
  out.colors[1] = seg.params.colors[1];
  out.colors[2] = seg.params.colors[2];
  return true;
}

// ============================================================
// Multi-segment mutators (id 0 = main, delegates to existing setters)
// ============================================================
void WLEDBridgeComponent::segment_set_bounds(uint8_t id, uint32_t start, uint32_t stop) {
  if (id == 0) {
    this->set_segment_bounds(start, stop);
    return;
  }
  if (this->total_leds_ == 0)
    return;
  // Removing an existing extra segment (empty bounds).
  if (stop <= start) {
    ExtraSegment *seg = this->resolve_extra_(id);
    if (seg == nullptr)
      return;
    // Compact: shift later segments down by swapping field-wise (non-copyable).
    for (uint8_t i = id; i < this->extra_count_; i++) {
      ExtraSegment &dst = this->extra_segments_[i - 1];
      ExtraSegment &src = this->extra_segments_[i];
      dst.start = src.start;
      dst.stop = src.stop;
      dst.grouping = src.grouping;
      dst.spacing = src.spacing;
      dst.reverse = src.reverse;
      dst.mirror = src.mirror;
      dst.on = src.on;
      dst.opacity = src.opacity;
      dst.mode = src.mode;
      dst.params = src.params;
      dst.env.free_data();
      dst.env.step = dst.env.call = 0;
      dst.env.aux0 = dst.env.aux1 = 0;
    }
    this->extra_count_--;
    this->extra_segments_[this->extra_count_].env.free_data();
    this->extra_segments_[this->extra_count_].stop = 0;
    this->opacity_map_dirty_ = true;
    this->mark_dirty_();
    return;
  }
  start = std::min<uint32_t>(start, this->total_leds_ - 1);
  stop = std::min<uint32_t>(stop, this->total_leds_);
  ExtraSegment *seg = this->resolve_extra_(id);
  if (seg == nullptr) {
    // Append a new extra segment when id is the next free slot.
    if (id != this->extra_count_ + 1 || this->extra_count_ + 1u >= WLED_MAX_SEGMENTS)
      return;
    seg = &this->extra_segments_[this->extra_count_];
    seg->env.free_data();
    seg->grouping = 1;
    seg->spacing = 0;
    seg->reverse = false;
    seg->mirror = false;
    seg->on = true;
    seg->opacity = 255;
    seg->mode = 0;
    seg->params = EffectParams{};
    this->extra_count_++;
  }
  seg->start = static_cast<uint16_t>(start);
  seg->stop = static_cast<uint16_t>(stop);
  seg->env.free_data();
  seg->env.step = seg->env.call = 0;
  seg->env.aux0 = seg->env.aux1 = 0;
  this->opacity_map_dirty_ = true;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_grouping(uint8_t id, uint16_t grouping, uint16_t spacing) {
  if (grouping < 1)
    grouping = 1;
  if (id == 0) {
    this->main_grouping_ = grouping;
    this->main_spacing_ = spacing;
  } else {
    ExtraSegment *seg = this->resolve_extra_(id);
    if (seg == nullptr)
      return;
    seg->grouping = grouping;
    seg->spacing = spacing;
  }
  this->opacity_map_dirty_ = true;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_on(uint8_t id, bool on) {
  if (id == 0) {
    this->set_on(on);
    return;
  }
  ExtraSegment *seg = this->resolve_extra_(id);
  if (seg == nullptr || seg->on == on)
    return;
  seg->on = on;
  this->opacity_map_dirty_ = true;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_opacity(uint8_t id, uint8_t opacity) {
  if (id == 0) {
    this->main_opacity_ = opacity;
  } else {
    ExtraSegment *seg = this->resolve_extra_(id);
    if (seg == nullptr)
      return;
    seg->opacity = opacity;
  }
  this->opacity_map_dirty_ = true;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_effect(uint8_t id, uint8_t fx) {
  if (fx >= WLED_EFFECT_COUNT)
    return;
  if (id == 0) {
    this->set_effect(fx);
    return;
  }
  ExtraSegment *seg = this->resolve_extra_(id);
  if (seg == nullptr || seg->mode == fx)
    return;
  seg->mode = fx;
  seg->env.free_data();
  seg->env.step = seg->env.call = 0;
  seg->env.aux0 = seg->env.aux1 = 0;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_speed(uint8_t id, uint8_t v) {
  if (id == 0) {
    this->set_speed(v);
    return;
  }
  ExtraSegment *seg = this->resolve_extra_(id);
  if (seg == nullptr)
    return;
  seg->params.speed = v;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_intensity(uint8_t id, uint8_t v) {
  if (id == 0) {
    this->set_intensity(v);
    return;
  }
  ExtraSegment *seg = this->resolve_extra_(id);
  if (seg == nullptr)
    return;
  seg->params.intensity = v;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_palette(uint8_t id, uint8_t v) {
  if (id == 0) {
    this->set_palette(v);
    return;
  }
  ExtraSegment *seg = this->resolve_extra_(id);
  if (seg == nullptr)
    return;
  seg->params.palette_id = v;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_custom(uint8_t id, uint8_t which, uint8_t v) {
  EffectParams *p = nullptr;
  if (id == 0) {
    p = &this->params_;
  } else {
    ExtraSegment *seg = this->resolve_extra_(id);
    if (seg == nullptr)
      return;
    p = &seg->params;
  }
  if (which == 1)
    p->custom1 = v;
  else if (which == 2)
    p->custom2 = v;
  else if (which == 3)
    p->custom3 = v;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_check(uint8_t id, uint8_t which, bool v) {
  EffectParams *p = nullptr;
  if (id == 0) {
    p = &this->params_;
  } else {
    ExtraSegment *seg = this->resolve_extra_(id);
    if (seg == nullptr)
      return;
    p = &seg->params;
  }
  if (which == 1)
    p->check1 = v;
  else if (which == 2)
    p->check2 = v;
  else if (which == 3)
    p->check3 = v;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_color(uint8_t id, uint8_t slot, uint32_t rgb) {
  if (slot >= 3)
    return;
  if (id == 0) {
    this->set_color(slot, rgb);
    return;
  }
  ExtraSegment *seg = this->resolve_extra_(id);
  if (seg == nullptr)
    return;
  seg->params.colors[slot] = rgb;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_reverse(uint8_t id, bool reverse) {
  if (id == 0) {
    this->set_segment_reverse(reverse);
    return;
  }
  ExtraSegment *seg = this->resolve_extra_(id);
  if (seg == nullptr || seg->reverse == reverse)
    return;
  seg->reverse = reverse;
  this->mark_dirty_();
}

void WLEDBridgeComponent::segment_set_mirror(uint8_t id, bool mirror) {
  if (id == 0) {
    this->set_segment_mirror(mirror);
    return;
  }
  ExtraSegment *seg = this->resolve_extra_(id);
  if (seg == nullptr || seg->mirror == mirror)
    return;
  seg->mirror = mirror;
  this->mark_dirty_();
}

void WLEDBridgeComponent::reset_segment_state_() {
  this->env_.step = 0;
  this->env_.call = 0;
  this->env_.aux0 = 0;
  this->env_.aux1 = 0;
  this->env_.free_data();
  if (this->full_frame_ != nullptr && this->total_leds_ > 0)
    memset(this->full_frame_, 0, this->total_leds_ * sizeof(uint32_t));
}

// ============================================================
// dump_config
// ============================================================
void WLEDBridgeComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "WLED Bridge:");
  ESP_LOGCONFIG(TAG, "  Buses: %zu", this->buses_.size());
  for (size_t i = 0; i < this->buses_.size(); i++) {
    const auto &bus = this->buses_[i];
    ESP_LOGCONFIG(TAG, "    Bus %zu: virtual [%u-%u) (%u LEDs), max %u mA, %u mA/LED", i, bus.start,
                  bus.start + bus.len, bus.len, bus.max_ma, bus.led_ma);
  }
  ESP_LOGCONFIG(TAG, "  Virtual LEDs: %u", this->total_leds_);
  ESP_LOGCONFIG(TAG, "  FPS target: %u", WLED_FPS);
  ESP_LOGCONFIG(TAG, "  Effects: %zu", WLED_EFFECT_COUNT);
  ESP_LOGCONFIG(TAG, "  Palettes: %zu", WLED_PALETTE_COUNT);
  ESP_LOGCONFIG(TAG, "  Presets: %u slots", WLED_PRESET_COUNT);
  ESP_LOGCONFIG(TAG, "  Segment: %u-%u", this->segment_start_, this->segment_stop_);
  ESP_LOGCONFIG(TAG, "  Use task: %s", YESNO(this->use_task_));
}

// ============================================================
// FreeRTOS render task (optional)
// ============================================================
void WLEDBridgeComponent::render_task_fn_(void *arg) {
  auto *self = static_cast<WLEDBridgeComponent *>(arg);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(FRAMETIME_MS);

  while (true) {
    self->render_frame_();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

}  // namespace wled_bridge
}  // namespace esphome
