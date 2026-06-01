#include "wled_bridge.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/color.h"
#include "wled_color.h"

#include "wled_json.h"
#include "wled_ui_data.h"
#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace wled_bridge {

static const char *const TAG = "wled_bridge";

// ============================================================
// setup
// ============================================================
void WLEDBridgeComponent::setup() {
  if (this->light_ == nullptr) {
    ESP_LOGE(TAG, "Light not set — check light_id in YAML");
    this->mark_failed();
    return;
  }

  this->led_count_ = static_cast<uint32_t>(this->light_->size());
  if (this->led_count_ == 0) {
    ESP_LOGE(TAG, "Light reports 0 LEDs");
    this->mark_failed();
    return;
  }

  // Tell ESPHome's light subsystem that an effect owns the pixels.
  // update_state() will still apply local_brightness (used for ABL) but
  // won't overwrite our pixel data.
  this->light_->set_effect_active(true);

  // Disable ESPHome colour correction — WLED bridge handles gamma itself.
  this->light_->set_correction(1.0f, 1.0f, 1.0f);

  // Default effect parameters
  this->params_.speed = 128;
  this->params_.intensity = 128;
  this->params_.colors[0] = 0xFFAA00;
  this->params_.colors[1] = 0x000000;
  this->params_.colors[2] = 0x000000;
  this->params_.palette_id = 0;

  // Zero the frame
  for (int32_t i = 0; i < this->light_->size(); i++)
    (*this->light_)[i].set(esphome::Color::BLACK);
  this->light_->schedule_show();

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
    xTaskCreatePinnedToCore(render_task_fn_, "wled_render",
                            8192,  // stack in bytes
                            this,  // parameter
                            5,  // priority (above idle, below WiFi)
                            &this->render_task_,
                            APP_CPU_NUM  // pin to core 1
    );
    ESP_LOGD(TAG, "Render task started on core 1");
  }

  ESP_LOGCONFIG(TAG, "WLED Bridge ready: %u LEDs, max %u mA, %u mA/LED", this->led_count_, this->max_ma_,
                this->led_ma_);
}

// ============================================================
// loop
// ============================================================
void WLEDBridgeComponent::loop() {
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
}

// ============================================================
// render_frame_
// ============================================================
void WLEDBridgeComponent::render_frame_() {
  if (this->light_ == nullptr)
    return;

  if (!this->is_on_) {
    for (int32_t i = 0; i < this->light_->size(); i++)
      (*this->light_)[i].set(esphome::Color::BLACK);
    this->light_->schedule_show();
    return;
  }

  uint32_t now = millis();
  EffectContext ctx{
      this->light_, &this->params_, &this->env_, now, static_cast<int32_t>(this->led_count_),
  };

  // Run the active effect
  if (this->active_fx_ < WLED_EFFECT_COUNT) {
    WLED_EFFECTS[this->active_fx_].fn(ctx);
  }

  // Increment frame counter
  this->env_.call++;

  // Apply ABL (sets local_brightness on the correction object)
  this->apply_abl_();

  this->light_->schedule_show();
}

// ============================================================
// apply_abl_  — Automatic Brightness Limiter
// Scales pixels down in-place if the estimated power draw would exceed max_ma.
// Uses pixel-level scaling to avoid touching the protected ESPColorCorrection.
// ============================================================
void WLEDBridgeComponent::apply_abl_() {
  if (this->max_ma_ == 0 || this->led_ma_ == 0 || this->led_count_ == 0)
    return;

  // Conservative worst-case power estimate: assume all LEDs at current brightness
  // actual = global_bri * led_count * led_ma / 255
  uint32_t estimated_ma = (static_cast<uint32_t>(this->global_bri_) * this->led_count_ * this->led_ma_) / 255u;
  this->current_ma_ = estimated_ma;

  if (estimated_ma <= this->max_ma_)
    return;

  // Scale factor to bring within budget
  uint8_t abl_scale = static_cast<uint8_t>((this->max_ma_ * 255u) / estimated_ma);
  this->current_ma_ = (static_cast<uint32_t>(abl_scale) * this->led_count_ * this->led_ma_) / 255u;

  // Apply scale to all pixels in the buffer
  for (int32_t i = 0; i < this->light_->size(); i++) {
    auto pv = (*this->light_)[i];
    // get() returns un-corrected values; set() re-applies correction
    esphome::Color c = pv.get();
    c.r = scale8(c.r, abl_scale);
    c.g = scale8(c.g, abl_scale);
    c.b = scale8(c.b, abl_scale);
    c.w = scale8(c.w, abl_scale);
    pv.set(c);
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
    this->active_fx_ = fx_index;
    this->reset_segment_state_();
    this->mark_dirty_();
  }
}

void WLEDBridgeComponent::reset_segment_state_() {
  this->env_.step = 0;
  this->env_.call = 0;
  this->env_.aux0 = 0;
  this->env_.aux1 = 0;
  this->env_.free_data();
  for (int32_t i = 0; i < this->light_->size(); i++)
    (*this->light_)[i].set(esphome::Color::BLACK);
}

// ============================================================
// dump_config
// ============================================================
void WLEDBridgeComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "WLED Bridge:");
  ESP_LOGCONFIG(TAG, "  LEDs: %u", this->led_count_);
  ESP_LOGCONFIG(TAG, "  Max current: %u mA", this->max_ma_);
  ESP_LOGCONFIG(TAG, "  mA/LED: %u", this->led_ma_);
  ESP_LOGCONFIG(TAG, "  FPS target: %u", WLED_FPS);
  ESP_LOGCONFIG(TAG, "  Effects: %zu", WLED_EFFECT_COUNT);
  ESP_LOGCONFIG(TAG, "  Palettes: %zu", WLED_PALETTE_COUNT);
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
