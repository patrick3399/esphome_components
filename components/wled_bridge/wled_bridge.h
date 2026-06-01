#pragma once
#include <array>
#include <vector>
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/light/addressable_light.h"
#include "esphome/components/light/addressable_light_effect.h"
#include "esphome/components/light/light_state.h"
#include "wled_types.h"
#include "wled_effects.h"

#include "wled_json.h"

namespace esphome {
namespace wled_bridge {

class WLEDBridgeComponent;  // forward

// Thin ESPHome effect proxy so HA can select WLED effects by name.
// Delegates rendering entirely to WLEDBridgeComponent::loop().
class WLEDProxyEffect : public light::AddressableLightEffect {
 public:
  WLEDProxyEffect(const char *name, uint8_t effect_id, WLEDBridgeComponent *comp)
      : light::AddressableLightEffect(name), effect_id_(effect_id), comp_(comp) {}

  void start() override;

  void stop() override {
    // Do NOT call parent::stop() — that calls set_effect_active(false).
    // Bridge owns pixel writes unconditionally; reassert here.
    if (this->state_ != nullptr) {
      auto *al = static_cast<light::AddressableLight *>(
          static_cast<light::AddressableLightState *>(this->state_)->get_output());
      al->set_effect_active(true);
    }
  }

  void apply(light::AddressableLight &it, const Color & /*current_color*/) override {
    it.set_effect_active(true);  // keep asserted while this proxy is "active"
  }

 protected:
  uint8_t effect_id_;
  WLEDBridgeComponent *comp_;
};

static constexpr uint8_t WLED_PRESET_COUNT = 16;
static constexpr size_t WLED_PRESET_NAME_SIZE = 24;

struct WLEDPresetRecord {
  uint8_t valid{0};
  uint8_t on{1};
  uint8_t brightness{128};
  uint8_t effect{0};
  uint8_t speed{128};
  uint8_t intensity{128};
  uint8_t custom1{128};
  uint8_t custom2{128};
  uint8_t custom3{16};
  uint8_t check1{0};
  uint8_t check2{0};
  uint8_t check3{0};
  uint8_t palette{0};
  uint16_t segment_start{0};
  uint16_t segment_stop{0};
  uint8_t reverse{0};
  uint8_t mirror{0};
  uint16_t transition_ms{0};
  uint32_t colors[3]{0xFFAA00, 0x000000, 0x000000};
  char name[WLED_PRESET_NAME_SIZE]{};
};

struct WLEDPresetStore {
  uint32_t magic{0};
  WLEDPresetRecord slots[WLED_PRESET_COUNT]{};
};

struct WLEDStoredState {
  uint32_t magic{0};
  WLEDPresetRecord state{};
};

class WLEDBridgeComponent : public Component, public light::LightRemoteValuesListener {
 public:
  // ---- ESPHome lifecycle ----
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_light_remote_values_update() override;
  float get_setup_priority() const override {
    return setup_priority::LATE;
  }

  // ---- Setters called from generated code ----
  // add_bus() is called once per bus in declaration order.
  // The first call establishes the primary bus (HA sync / effect registration).
  void add_bus(light::LightState *state, uint32_t max_ma, uint32_t led_ma);

  void set_use_task(bool use_task) {
    this->use_task_ = use_task;
  }

  // Auto-white: derive the W channel from RGB for RGBW strips.
  // 0 = none, 1 = brighter (keep RGB, add W), 2 = accurate (subtract W from RGB),
  // 4 = max (W = brightest channel). Only enable on RGBW hardware.
  void set_auto_white_mode(uint8_t mode) {
    this->auto_white_mode_ = mode;
  }
  uint8_t get_auto_white_mode() const {
    return this->auto_white_mode_;
  }

  // ---- State accessors (used by JSON layer) ----
  bool is_on() const {
    return this->is_on_;
  }
  uint8_t get_brightness() const {
    return this->global_bri_;
  }
  uint8_t get_effect_index() const {
    return this->active_fx_;
  }
  const EffectParams &get_params() const {
    return this->params_;
  }
  uint32_t get_led_count() const {
    return this->total_leds_;
  }
  uint32_t get_max_ma() const {
    uint32_t total = 0;
    for (const auto &bus : this->buses_)
      total += bus.max_ma;
    return total;
  }
  uint32_t get_current_ma() const {
    return this->current_ma_;
  }
  uint32_t get_state_version() const {
    return this->state_version_;
  }
  uint16_t get_transition_ms() const {
    return this->transition_ms_;
  }
  uint8_t get_active_preset() const {
    return this->active_preset_;
  }
  uint32_t get_segment_start() const {
    return this->segment_start_;
  }
  uint32_t get_segment_stop() const {
    return this->segment_stop_;
  }
  uint32_t get_segment_length() const {
    return this->segment_stop_ > this->segment_start_ ? this->segment_stop_ - this->segment_start_ : 0;
  }
  bool is_segment_reversed() const {
    return this->segment_reverse_;
  }
  bool is_segment_mirrored() const {
    return this->segment_mirror_;
  }
  const WLEDPresetRecord *get_preset(uint8_t preset_id) const {
    if (preset_id == 0 || preset_id > WLED_PRESET_COUNT)
      return nullptr;
    return &this->preset_store_.slots[preset_id - 1];
  }
  bool is_preset_valid(uint8_t preset_id) const;
  void publish_light_state();

  // ---- Multi-segment read API (id 0 = main segment) ----
  uint8_t get_segment_count() const {
    return static_cast<uint8_t>(1 + this->extra_count_);
  }
  static constexpr uint8_t get_max_segments() {
    return WLED_MAX_SEGMENTS;
  }
  uint8_t get_main_segment() const {
    return 0;
  }
  struct SegmentReadView {
    uint16_t start, stop, grouping, spacing;
    bool on, reverse, mirror, selected;
    uint8_t opacity, mode, speed, intensity, palette;
    uint8_t custom1, custom2, custom3;
    bool check1, check2, check3;
    uint32_t colors[3];
  };
  bool get_segment_view(uint8_t id, SegmentReadView &out) const;

  // ---- Multi-segment mutation API (id 0 = main segment) ----
  // Setting bounds with stop <= start on an extra segment removes it.
  void segment_set_bounds(uint8_t id, uint32_t start, uint32_t stop);
  void segment_set_grouping(uint8_t id, uint16_t grouping, uint16_t spacing);
  void segment_set_on(uint8_t id, bool on);
  void segment_set_opacity(uint8_t id, uint8_t opacity);
  void segment_set_effect(uint8_t id, uint8_t fx);
  void segment_set_speed(uint8_t id, uint8_t v);
  void segment_set_intensity(uint8_t id, uint8_t v);
  void segment_set_palette(uint8_t id, uint8_t v);
  void segment_set_custom(uint8_t id, uint8_t which, uint8_t v);  // which 1..3
  void segment_set_check(uint8_t id, uint8_t which, bool v);  // which 1..3
  void segment_set_color(uint8_t id, uint8_t slot, uint32_t rgb);
  void segment_set_reverse(uint8_t id, bool reverse);
  void segment_set_mirror(uint8_t id, bool mirror);

  // ---- State mutators (used by JSON POST handler) ----
  void set_on(bool on);
  void set_brightness(uint8_t bri);
  void set_effect(uint8_t fx_index);
  void set_speed(uint8_t sx) {
    this->params_.speed = sx;
    this->mark_dirty_();
  }
  void set_intensity(uint8_t ix) {
    this->params_.intensity = ix;
    this->mark_dirty_();
  }
  void set_palette(uint8_t pal) {
    this->params_.palette_id = pal;
    this->mark_dirty_();
  }
  void set_custom1(uint8_t c1) {
    this->params_.custom1 = c1;
    this->mark_dirty_();
  }
  void set_custom2(uint8_t c2) {
    this->params_.custom2 = c2;
    this->mark_dirty_();
  }
  void set_custom3(uint8_t c3) {
    this->params_.custom3 = c3;
    this->mark_dirty_();
  }
  void set_check1(bool o1) {
    this->params_.check1 = o1;
    this->mark_dirty_();
  }
  void set_check2(bool o2) {
    this->params_.check2 = o2;
    this->mark_dirty_();
  }
  void set_check3(bool o3) {
    this->params_.check3 = o3;
    this->mark_dirty_();
  }
  void set_color(uint8_t slot, uint32_t rgb) {
    if (slot < 3) {
      this->params_.colors[slot] = rgb;
      this->mark_dirty_();
    }
  }
  void set_custom(uint8_t c1, uint8_t c2, uint8_t c3) {
    this->params_.custom1 = c1;
    this->params_.custom2 = c2;
    this->params_.custom3 = c3;
    this->mark_dirty_();
  }
  void set_transition(uint16_t ms) {
    this->transition_ms_ = ms;
    this->mark_dirty_();
  }
  bool save_preset(uint8_t preset_id, const char *name = nullptr);
  bool load_preset(uint8_t preset_id);
  bool delete_preset(uint8_t preset_id);
  void set_segment_bounds(uint32_t start, uint32_t stop);
  void set_segment_reverse(bool reverse);
  void set_segment_mirror(bool mirror);

  // SSE dirty flag — checked in loop() to broadcast state change
  bool is_state_dirty() const {
    return this->state_dirty_;
  }
  void clear_dirty() {
    this->state_dirty_ = false;
  }

 protected:
  // One physical LED strip with its position in the virtual LED space.
  struct VirtualBus {
    light::LightState *light_state;  // ESPHome LightState (for correction/effect_active)
    light::AddressableLight *strip;  // addressable output (populated in setup)
    uint32_t start;  // virtual start index (sum of preceding bus lengths)
    uint32_t len;  // LED count of this strip
    uint32_t max_ma;  // ABL budget for this bus (0 = unlimited)
    uint32_t led_ma;  // estimated mA per LED at full white
  };

  // Lightweight render view shared by the main segment (scalar members) and
  // each ExtraSegment, so one routine renders both.
  struct SegmentView {
    int32_t start;
    int32_t stop;
    bool reverse;
    bool mirror;
    uint16_t grouping;
    uint16_t spacing;
    EffectParams *params;
    SegmentState *env;
    uint8_t mode;
  };

  void sync_from_light_state_(bool publish);
  void render_frame_();
  void render_segment_(const SegmentView &view, uint32_t now);
  void rebuild_opacity_map_();
  ExtraSegment *resolve_extra_(uint8_t id);  // id>=1 → &extra_segments_[id-1] if valid, else nullptr
  void reset_output_correction_();
  uint8_t compute_output_scale_();
  void flush_frame_to_buses_(uint8_t final_scale);
  void apply_transition_blend_(uint32_t now);
  void reset_segment_state_();
  void begin_transition_();
  void load_presets_();
  void persist_presets_();
  void load_state_();
  void persist_state_();
  void schedule_state_save_();
  void apply_preset_(const WLEDPresetRecord &preset);
  WLEDPresetRecord current_as_preset_() const;
  void set_default_preset_name_(WLEDPresetRecord *preset, uint8_t preset_id) const;
  void copy_preset_name_(WLEDPresetRecord *preset, const char *name) const;
  void mark_dirty_(bool clear_active_preset = true) {
    if (clear_active_preset)
      this->active_preset_ = 0;
    this->begin_transition_();
    this->state_version_++;
    this->state_dirty_ = true;
    this->schedule_state_save_();
  }

  static void render_task_fn_(void *arg);
  TaskHandle_t render_task_{nullptr};

  // Bus registry — populated by add_bus() before setup().
  std::vector<VirtualBus> buses_{};
  // Primary bus aliases (first bus): used for HA sync and effect registration.
  light::LightState *light_state_{nullptr};
  light::AddressableLight *light_{nullptr};  // kept for WLEDProxyEffect::stop()

  EffectParams params_{};
  SegmentState env_{};

  uint8_t active_fx_{0};
  uint8_t global_bri_{128};
  bool is_on_{true};
  uint32_t total_leds_{0};  // virtual LED space size (sum of all bus lengths)
  // ---- main segment (id 0) ----
  uint32_t segment_start_{0};
  uint32_t segment_stop_{0};
  bool segment_reverse_{false};
  bool segment_mirror_{false};
  uint16_t main_grouping_{1};
  uint16_t main_spacing_{0};
  uint8_t main_opacity_{255};
  // ---- extra segments (id 1..) ----
  std::array<ExtraSegment, WLED_MAX_SEGMENTS - 1> extra_segments_{};
  uint8_t extra_count_{0};
  uint8_t *pixel_opacity_{nullptr};  // per-LED opacity (segment bri / on-off mask)
  bool opacity_map_dirty_{true};
  uint32_t current_ma_{0};
  uint8_t auto_white_mode_{0};
  bool use_task_{false};
  uint16_t transition_ms_{0};
  uint32_t *full_frame_{nullptr};  // unscaled rendered pixel values
  uint32_t *transition_frame_{nullptr};  // snapshot taken at transition start
  uint32_t transition_start_ms_{0};
  uint32_t transition_duration_ms_{0};
  bool transition_active_{false};
  uint8_t active_preset_{0};
  WLEDPresetStore preset_store_{};
  ESPPreferenceObject preset_pref_{};
  ESPPreferenceObject state_pref_{};

  uint32_t last_frame_ms_{0};
  uint32_t state_version_{0};
  uint32_t state_save_due_ms_{0};
  bool state_dirty_{false};
  bool state_loaded_{false};
  bool state_pref_ready_{false};
  bool suppress_light_sync_{false};

  WLEDJsonHandler *json_handler_{nullptr};
  WLEDSseHandler *sse_handler_{nullptr};
  WLEDUiHandler *ui_handler_{nullptr};
};

}  // namespace wled_bridge
}  // namespace esphome
