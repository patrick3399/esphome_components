#pragma once
#include <array>
#include <vector>
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/light/addressable_light.h"
#include "esphome/components/light/addressable_light_effect.h"
#include "esphome/components/light/light_state.h"
#include "wled_types.h"
#include "wled_effects.h"
#include "wled_json.h"
#include "wled_udp.h"
#ifdef USE_ESP32
#include "wled_ws.h"
#endif

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
static constexpr uint8_t WLED_PLAYLIST_MAX_ENTRIES = 16;

// Compact persisted form of one extra (non-main) segment.
struct WLEDExtraSegRecord {
  uint16_t start{0};
  uint16_t stop{0};
  uint16_t grouping{1};
  uint16_t spacing{0};
  uint8_t reverse{0};
  uint8_t mirror{0};
  uint8_t on{1};
  uint8_t opacity{255};
  uint8_t mode{0};
  uint8_t speed{128};
  uint8_t intensity{128};
  uint8_t custom1{128};
  uint8_t custom2{128};
  uint8_t custom3{16};
  uint8_t check1{0};
  uint8_t check2{0};
  uint8_t check3{0};
  uint8_t palette{0};
  uint32_t colors[3]{0xFFAA00, 0x000000, 0x000000};
};

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
  uint8_t main_segment{0};
  uint8_t main_grouping{1};
  uint8_t main_spacing{0};
  uint8_t main_opacity{255};
  uint8_t extra_count{0};
  uint8_t playlist_count{0};
  uint8_t playlist_repeat{0};
  uint16_t transition_ms{0};
  uint32_t colors[3]{0xFFAA00, 0x000000, 0x000000};
  WLEDExtraSegRecord extras[WLED_MAX_SEGMENTS - 1]{};
  uint8_t playlist_presets[WLED_PLAYLIST_MAX_ENTRIES]{};
  uint16_t playlist_durations[WLED_PLAYLIST_MAX_ENTRIES]{};
  uint16_t playlist_transitions[WLED_PLAYLIST_MAX_ENTRIES]{};
  char name[WLED_PRESET_NAME_SIZE]{};
};

struct WLEDPresetStore {
  uint32_t magic{0};
  WLEDPresetRecord slots[WLED_PRESET_COUNT]{};
};

struct WLEDStoredState {
  uint32_t magic{0};
  WLEDPresetRecord state{};  // main segment + globals
  uint8_t main_grouping{1};
  uint8_t main_spacing{0};
  uint8_t main_opacity{255};
  uint8_t main_segment{0};
  uint8_t extra_count{0};
  WLEDExtraSegRecord extras[WLED_MAX_SEGMENTS - 1]{};
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

  // 2D matrix geometry
  void set_matrix_width(uint16_t w) {
    this->matrix_width_ = w;
  }
  void set_matrix_height(uint16_t h) {
    this->matrix_height_ = h;
  }
  void set_matrix_serpentine(bool s) {
    this->matrix_serpentine_ = s;
  }
  uint16_t get_matrix_width() const {
    return this->matrix_width_;
  }
  uint16_t get_matrix_height() const {
    return this->matrix_height_;
  }
  bool get_matrix_serpentine() const {
    return this->matrix_serpentine_;
  }
  bool is_2d() const {
    return this->matrix_width_ > 0 && this->matrix_height_ > 0;
  }

  // UDP sync
  void set_udp_port(uint16_t port) {
    this->udp_port_ = port;
#ifdef USE_ESP32
    this->udp_sync_.set_ports(this->udp_port_, this->udp_port2_);
#endif
  }
  void set_udp_port2(uint16_t port) {
    this->udp_port2_ = port;
#ifdef USE_ESP32
    this->udp_sync_.set_ports(this->udp_port_, this->udp_port2_);
#endif
  }
  void set_udp_send(bool v) {
    this->udp_send_ = v;
  }
  void set_udp_receive(bool v) {
    this->udp_receive_ = v;
  }
  void set_udp_send_enabled(bool v) {
    this->udp_send_ = v;
#ifdef USE_ESP32
    this->udp_sync_.set_send_enabled(v);
#endif
    this->mark_dirty_(false);
  }
  void set_udp_receive_enabled(bool v) {
    this->udp_receive_ = v;
#ifdef USE_ESP32
    this->udp_sync_.set_receive_enabled(v);
#endif
    this->mark_dirty_(false);
  }
  uint16_t get_udp_port() const {
    return this->udp_port_;
  }
  uint16_t get_udp_port2() const {
    return this->udp_port2_;
  }
  bool get_udp_send() const {
    return this->udp_send_;
  }
  bool get_udp_receive() const {
    return this->udp_receive_;
  }
  void set_udp_sync_groups(uint8_t groups) {
    this->udp_sync_groups_ = groups;
    this->mark_dirty_(false);
  }
  void set_udp_receive_groups(uint8_t groups) {
    this->udp_receive_groups_ = groups;
    this->mark_dirty_(false);
  }
  void set_udp_receive_brightness(bool v) {
    this->udp_receive_brightness_ = v;
    this->mark_dirty_(false);
  }
  void set_udp_receive_color(bool v) {
    this->udp_receive_color_ = v;
    this->mark_dirty_(false);
  }
  void set_udp_receive_effects(bool v) {
    this->udp_receive_effects_ = v;
    this->mark_dirty_(false);
  }
  void set_udp_receive_palette(bool v) {
    this->udp_receive_palette_ = v;
    this->mark_dirty_(false);
  }
  void set_udp_receive_segments(bool v) {
    this->udp_receive_segments_ = v;
    this->mark_dirty_(false);
  }
  void set_udp_receive_segment_options(bool v) {
    this->udp_receive_segment_options_ = v;
    this->mark_dirty_(false);
  }
  void set_udp_notify_direct(bool v) {
    this->udp_notify_direct_ = v;
    this->mark_dirty_(false);
  }
  void set_udp_notify_button(bool v) {
    this->udp_notify_button_ = v;
    this->mark_dirty_(false);
  }
  void set_udp_notify_alexa(bool v) {
    this->udp_notify_alexa_ = v;
    this->mark_dirty_(false);
  }
  void set_udp_notify_hue(bool v) {
    this->udp_notify_hue_ = v;
    this->mark_dirty_(false);
  }
  void set_udp_retries(uint8_t retries) {
    this->udp_retries_ = retries;
    this->mark_dirty_(false);
  }
  uint8_t get_udp_sync_groups() const {
    return this->udp_sync_groups_;
  }
  uint8_t get_udp_receive_groups() const {
    return this->udp_receive_groups_;
  }
  bool get_udp_receive_brightness() const {
    return this->udp_receive_brightness_;
  }
  bool get_udp_receive_color() const {
    return this->udp_receive_color_;
  }
  bool get_udp_receive_effects() const {
    return this->udp_receive_effects_;
  }
  bool get_udp_receive_palette() const {
    return this->udp_receive_palette_;
  }
  bool get_udp_receive_segments() const {
    return this->udp_receive_segments_;
  }
  bool get_udp_receive_segment_options() const {
    return this->udp_receive_segment_options_;
  }
  bool get_udp_notify_direct() const {
    return this->udp_notify_direct_;
  }
  bool get_udp_notify_button() const {
    return this->udp_notify_button_;
  }
  bool get_udp_notify_alexa() const {
    return this->udp_notify_alexa_;
  }
  bool get_udp_notify_hue() const {
    return this->udp_notify_hue_;
  }
  uint8_t get_udp_retries() const {
    return this->udp_retries_;
  }
#ifdef USE_ESP32
  int get_ws_client_count() const {
    return this->ws_handler_ != nullptr ? static_cast<int>(this->ws_handler_->client_count()) : 0;
  }
#endif

  // Boot preset (0 = use NVS last state, 1-16 = load specific preset on boot)
  void set_boot_preset(uint8_t preset) {
    this->boot_preset_ = preset;
  }
  uint8_t get_boot_preset() const {
    return this->boot_preset_;
  }

  // Brightness factor (0-100%, caps maximum brightness)
  void set_brightness_factor(uint8_t percent) {
    this->brightness_factor_ = percent > 100 ? 100 : percent;
  }
  uint8_t get_brightness_factor() const {
    return this->brightness_factor_;
  }

  // DDP realtime receiver
  void set_ddp_enabled(bool v) {
    this->ddp_enabled_ = v;
  }
  bool get_ddp_enabled() const {
    return this->ddp_enabled_;
  }

  // E1.31 (sACN) realtime receiver
  void set_e131_enabled(bool v) {
    this->e131_enabled_ = v;
  }
  void set_e131_universe(uint16_t uni) {
    this->e131_universe_ = uni;
  }
  void set_e131_universe_count(uint8_t count) {
    this->e131_universe_count_ = count;
  }
  bool get_e131_enabled() const {
    return this->e131_enabled_;
  }
  uint16_t get_e131_universe() const {
    return this->e131_universe_;
  }
  uint8_t get_e131_universe_count() const {
    return this->e131_universe_count_;
  }

#ifdef USE_ESP32
  bool is_ddp_receiving() const {
    return this->ddp_receiver_.is_receiving();
  }
  bool is_e131_receiving() const {
    return this->e131_receiver_.is_receiving();
  }
#else
  bool is_ddp_receiving() const {
    return false;
  }
  bool is_e131_receiving() const {
    return false;
  }
#endif

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
  size_t get_bus_count() const {
    return this->buses_.size();
  }
  uint32_t get_bus_start(size_t bus_index) const {
    return bus_index < this->buses_.size() ? this->buses_[bus_index].start : 0;
  }
  uint32_t get_bus_len(size_t bus_index) const {
    return bus_index < this->buses_.size() ? this->buses_[bus_index].len : 0;
  }
  uint32_t get_bus_max_ma(size_t bus_index) const {
    return bus_index < this->buses_.size() ? this->buses_[bus_index].max_ma : 0;
  }
  uint32_t get_bus_led_ma(size_t bus_index) const {
    return bus_index < this->buses_.size() ? this->buses_[bus_index].led_ma : 0;
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
  uint32_t get_live_pixel_color(uint32_t index) const;
  bool set_pixel_override(uint8_t segment_id, uint32_t segment_offset, uint32_t color);
  void clear_pixel_overrides();
  uint32_t get_state_version() const {
    return this->state_version_;
  }
  uint16_t get_transition_ms() const {
    return this->transition_ms_;
  }
  bool is_nightlight_active() const {
    return this->nightlight_active_;
  }
  uint16_t get_nightlight_duration_s() const {
    return this->nightlight_duration_s_;
  }
  uint16_t get_nightlight_duration_min() const {
    return static_cast<uint16_t>((static_cast<uint32_t>(this->nightlight_duration_s_) + 59u) / 60u);
  }
  uint8_t get_nightlight_mode() const {
    return this->nightlight_mode_;
  }
  uint8_t get_nightlight_target_brightness() const {
    return this->nightlight_target_bri_;
  }
  int32_t get_nightlight_remaining_s() const;
  uint8_t get_active_preset() const {
    return this->active_preset_;
  }
  bool is_playlist_active() const {
    return this->playlist_active_;
  }
  int16_t get_active_playlist() const {
    return this->playlist_active_ ? static_cast<int16_t>(this->active_playlist_preset_) : -1;
  }
  uint8_t get_playlist_index() const {
    return this->playlist_index_;
  }
  uint8_t get_playlist_count() const {
    return this->playlist_count_;
  }
  uint8_t get_playlist_repeat_remaining() const {
    return this->playlist_repeat_remaining_;
  }
  uint8_t get_playlist_preset(uint8_t index) const {
    return index < this->playlist_count_ ? this->playlist_presets_[index] : 0;
  }
  uint16_t get_playlist_duration(uint8_t index) const {
    return index < this->playlist_count_ ? this->playlist_durations_[index] : 0;
  }
  uint16_t get_playlist_transition(uint8_t index) const {
    return index < this->playlist_count_ ? this->playlist_transitions_[index] : 0;
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
    return this->main_segment_ < this->get_segment_count() ? this->main_segment_ : 0;
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
  void set_main_segment(uint8_t id);

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
  bool set_preset(uint8_t preset_id, const WLEDPresetRecord &preset);
  bool load_preset(uint8_t preset_id);
  bool delete_preset(uint8_t preset_id);
  bool start_playlist(const uint8_t *presets, const uint16_t *durations, const uint16_t *transitions, uint8_t count,
                      uint8_t repeat);
  void stop_playlist();
  void configure_nightlight(uint16_t duration_s, uint8_t target_bri, uint8_t mode);
  void start_nightlight(uint16_t duration_s, uint8_t target_bri, uint8_t mode);
  void stop_nightlight();
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
  void capture_extras_(WLEDStoredState *state) const;
  void apply_stored_extras_(const WLEDStoredState &state);
  void schedule_state_save_();
  void process_playlist_();
  void advance_playlist_();
  void process_nightlight_();
  void apply_nightlight_brightness_(uint8_t bri, bool finished);
  void apply_pixel_overrides_();
  void apply_preset_(const WLEDPresetRecord &preset);
  WLEDPresetRecord current_as_preset_() const;
  void set_default_preset_name_(WLEDPresetRecord *preset, uint8_t preset_id) const;
  void copy_preset_name_(WLEDPresetRecord *preset, const char *name) const;
  void mark_dirty_(bool clear_active_preset = true) {
    if (clear_active_preset) {
      this->active_preset_ = 0;
      if (!this->playlist_applying_)
        this->stop_playlist();
    }
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
  uint8_t main_segment_{0};
  // ---- extra segments (id 1..) ----
  std::array<ExtraSegment, WLED_MAX_SEGMENTS - 1> extra_segments_{};
  uint8_t extra_count_{0};
  uint8_t *pixel_opacity_{nullptr};  // per-LED opacity (segment bri / on-off mask)
  bool opacity_map_dirty_{true};
  uint32_t current_ma_{0};
  uint8_t last_output_scale_{255};
  uint8_t auto_white_mode_{0};
  bool use_task_{false};
  uint16_t transition_ms_{0};
  bool nightlight_active_{false};
  uint16_t nightlight_duration_s_{60};
  uint8_t nightlight_mode_{1};
  uint8_t nightlight_target_bri_{0};
  uint8_t nightlight_start_bri_{0};
  uint32_t nightlight_start_ms_{0};
  uint32_t nightlight_next_update_ms_{0};
  uint32_t *full_frame_{nullptr};  // unscaled rendered pixel values
  uint32_t *pixel_override_colors_{nullptr};
  uint8_t *pixel_override_mask_{nullptr};
  bool pixel_overrides_active_{false};
  uint32_t *transition_frame_{nullptr};  // snapshot taken at transition start
  uint32_t transition_start_ms_{0};
  uint32_t transition_duration_ms_{0};
  bool transition_active_{false};
  uint8_t active_preset_{0};
  uint8_t active_playlist_preset_{0};
  bool playlist_applying_{false};
  bool playlist_active_{false};
  uint8_t playlist_count_{0};
  uint8_t playlist_index_{0};
  uint8_t playlist_repeat_remaining_{0};  // 0 = forever
  uint32_t playlist_next_ms_{0};
  uint8_t playlist_presets_[WLED_PLAYLIST_MAX_ENTRIES]{};
  uint16_t playlist_durations_[WLED_PLAYLIST_MAX_ENTRIES]{};  // seconds
  uint16_t playlist_transitions_[WLED_PLAYLIST_MAX_ENTRIES]{};  // tenths of seconds
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
#ifdef USE_ESP32
  WLEDWsHandler *ws_handler_{nullptr};
#endif
  WLEDUdpSync udp_sync_{};
#ifdef USE_ESP32
  WLEDDdpReceiver ddp_receiver_{};
  WLEDE131Receiver e131_receiver_{};
#endif

  // 2D matrix geometry (0 = no 2D)
  uint16_t matrix_width_{0};
  uint16_t matrix_height_{0};
  bool matrix_serpentine_{false};

  // UDP sync config
  uint16_t udp_port_{21324};
  uint16_t udp_port2_{65506};
  bool udp_send_{false};
  bool udp_receive_{false};
  uint8_t udp_sync_groups_{0x01};
  uint8_t udp_receive_groups_{0x01};
  bool udp_receive_brightness_{true};
  bool udp_receive_color_{true};
  bool udp_receive_effects_{true};
  bool udp_receive_palette_{true};
  bool udp_receive_segments_{true};
  bool udp_receive_segment_options_{true};
  bool udp_notify_direct_{true};
  bool udp_notify_button_{true};
  bool udp_notify_alexa_{false};
  bool udp_notify_hue_{false};
  uint8_t udp_retries_{0};

  // Boot preset
  uint8_t boot_preset_{0};
  uint8_t brightness_factor_{100};

  // DDP realtime receiver
  bool ddp_enabled_{false};

  // E1.31 (sACN) realtime receiver
  bool e131_enabled_{false};
  uint16_t e131_universe_{1};
  uint8_t e131_universe_count_{1};
};

// ---- ESPHome automation actions ----

template <typename... Ts>
class LoadPresetAction : public Action<Ts...>, public Parented<WLEDBridgeComponent> {
 public:
  TEMPLATABLE_VALUE(uint8_t, preset)
  void play(const Ts &...x) override {
    this->parent_->load_preset(this->preset_.value(x...));
  }
};

template <typename... Ts>
class SavePresetAction : public Action<Ts...>, public Parented<WLEDBridgeComponent> {
 public:
  TEMPLATABLE_VALUE(uint8_t, preset)
  void play(const Ts &...x) override {
    this->parent_->save_preset(this->preset_.value(x...));
  }
};

template <typename... Ts>
class SetEffectAction : public Action<Ts...>, public Parented<WLEDBridgeComponent> {
 public:
  TEMPLATABLE_VALUE(uint8_t, effect)
  void play(const Ts &...x) override {
    this->parent_->set_effect(this->effect_.value(x...));
    this->parent_->publish_light_state();
  }
};

template <typename... Ts>
class SetPaletteAction : public Action<Ts...>, public Parented<WLEDBridgeComponent> {
 public:
  TEMPLATABLE_VALUE(uint8_t, palette)
  void play(const Ts &...x) override {
    this->parent_->set_palette(this->palette_.value(x...));
    this->parent_->publish_light_state();
  }
};

template <typename... Ts>
class SetBrightnessAction : public Action<Ts...>, public Parented<WLEDBridgeComponent> {
 public:
  TEMPLATABLE_VALUE(uint8_t, brightness)
  void play(const Ts &...x) override {
    this->parent_->set_brightness(this->brightness_.value(x...));
    this->parent_->publish_light_state();
  }
};

template <typename... Ts>
class SetColorAction : public Action<Ts...>, public Parented<WLEDBridgeComponent> {
 public:
  TEMPLATABLE_VALUE(uint8_t, red)
  TEMPLATABLE_VALUE(uint8_t, green)
  TEMPLATABLE_VALUE(uint8_t, blue)
  TEMPLATABLE_VALUE(uint8_t, white)
  void play(const Ts &...x) override {
    uint8_t r = this->red_.value(x...);
    uint8_t g = this->green_.value(x...);
    uint8_t b = this->blue_.value(x...);
    uint8_t w = this->white_.value(x...);
    this->parent_->set_color(0, RGBW32(r, g, b, w));
    this->parent_->publish_light_state();
  }
};

template <typename... Ts>
class PowerAction : public Action<Ts...>, public Parented<WLEDBridgeComponent> {
 public:
  void set_state(bool state) {
    this->state_ = state;
  }
  void set_toggle(bool toggle) {
    this->toggle_ = toggle;
  }
  void play(const Ts &...x) override {
    if (this->toggle_)
      this->parent_->set_on(!this->parent_->is_on());
    else
      this->parent_->set_on(this->state_);
    this->parent_->publish_light_state();
  }

 protected:
  bool state_{true};
  bool toggle_{false};
};

template <typename... Ts>
class StopPlaylistAction : public Action<Ts...>, public Parented<WLEDBridgeComponent> {
 public:
  void play(const Ts &...x) override {
    this->parent_->stop_playlist();
  }
};

}  // namespace wled_bridge
}  // namespace esphome
