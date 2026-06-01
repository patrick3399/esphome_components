#pragma once
#include "esphome/core/component.h"
#include "esphome/components/light/addressable_light.h"
#include "esphome/components/light/light_state.h"
#include "wled_types.h"
#include "wled_effects.h"

#include "wled_json.h"

namespace esphome {
namespace wled_bridge {

class WLEDBridgeComponent : public Component {
 public:
  // ---- ESPHome lifecycle ----
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override {
    return setup_priority::LATE;
  }

  // ---- Setters called from generated code ----
  void set_light_state(light::LightState *state) {
    this->light_state_ = state;
    this->light_ = static_cast<light::AddressableLight *>(state->get_output());
  }
  void set_max_ma(uint32_t ma) {
    this->max_ma_ = ma;
  }
  void set_led_ma(uint32_t ma) {
    this->led_ma_ = ma;
  }
  void set_use_task(bool use_task) {
    this->use_task_ = use_task;
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
    return this->led_count_;
  }
  uint32_t get_max_ma() const {
    return this->max_ma_;
  }
  uint32_t get_current_ma() const {
    return this->current_ma_;
  }

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
  }

  // SSE dirty flag — checked in loop() to broadcast state change
  bool is_state_dirty() const {
    return this->state_dirty_;
  }
  void clear_dirty() {
    this->state_dirty_ = false;
  }

 protected:
  void render_frame_();
  void apply_abl_();
  void reset_segment_state_();
  void mark_dirty_() {
    this->state_dirty_ = true;
  }

  static void render_task_fn_(void *arg);
  TaskHandle_t render_task_{nullptr};

  light::LightState *light_state_{nullptr};
  light::AddressableLight *light_{nullptr};

  EffectParams params_{};
  SegmentState env_{};

  uint8_t active_fx_{0};
  uint8_t global_bri_{128};
  bool is_on_{true};
  uint32_t led_count_{1};
  uint32_t max_ma_{2000};
  uint32_t led_ma_{20};
  uint32_t current_ma_{0};
  bool use_task_{false};
  uint16_t transition_ms_{0};

  uint32_t last_frame_ms_{0};
  bool state_dirty_{false};

  WLEDJsonHandler *json_handler_{nullptr};
  WLEDSseHandler *sse_handler_{nullptr};
  WLEDUiHandler *ui_handler_{nullptr};
};

}  // namespace wled_bridge
}  // namespace esphome
