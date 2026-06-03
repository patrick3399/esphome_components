#include "esphome/core/defines.h"
#ifdef WLED_BRIDGE_ENTITIES

#include "wled_entities.h"
#include "wled_bridge.h"
#include "wled_effects.h"
#include "wled_palette.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace wled_bridge {

static const char *const TAG = "wled_bridge.entities";

// ============================================================
// WLEDPaletteSelect
// ============================================================

void WLEDPaletteSelect::setup() {
  FixedVector<const char *> opts;
  opts.init(WLED_PALETTE_COUNT);
  for (size_t i = 0; i < WLED_PALETTE_COUNT; i++)
    opts.push_back(WLED_PALETTES[i].name);
  this->traits.set_options(opts);
  this->last_published_ = 255;  // force first publish
  ESP_LOGCONFIG(TAG, "Palette Select: %u options", static_cast<unsigned>(WLED_PALETTE_COUNT));
}

void WLEDPaletteSelect::loop() {
  if (this->bridge_ == nullptr)
    return;
  uint8_t current = this->bridge_->get_params().palette_id;
  if (current != this->last_published_ && current < WLED_PALETTE_COUNT) {
    this->publish_state(static_cast<size_t>(current));
    this->last_published_ = current;
  }
}

void WLEDPaletteSelect::control(const std::string &value) {
  if (this->bridge_ == nullptr)
    return;
  for (size_t i = 0; i < WLED_PALETTE_COUNT; i++) {
    if (value == WLED_PALETTES[i].name) {
      this->bridge_->set_palette(static_cast<uint8_t>(i));
      this->bridge_->publish_light_state();
      this->publish_state(i);
      this->last_published_ = static_cast<uint8_t>(i);
      return;
    }
  }
}

// ============================================================
// WLEDEffectSelect
// ============================================================

void WLEDEffectSelect::setup() {
  FixedVector<const char *> opts;
  opts.init(WLED_EFFECT_COUNT);
  for (size_t i = 0; i < WLED_EFFECT_COUNT; i++)
    opts.push_back(WLED_EFFECTS[i].name);
  this->traits.set_options(opts);
  this->last_published_ = 255;
  ESP_LOGCONFIG(TAG, "Effect Select: %u options", static_cast<unsigned>(WLED_EFFECT_COUNT));
}

void WLEDEffectSelect::loop() {
  if (this->bridge_ == nullptr)
    return;
  uint8_t current = this->bridge_->get_effect_index();
  if (current != this->last_published_ && current < WLED_EFFECT_COUNT) {
    this->publish_state(static_cast<size_t>(current));
    this->last_published_ = current;
  }
}

void WLEDEffectSelect::control(const std::string &value) {
  if (this->bridge_ == nullptr)
    return;
  for (size_t i = 0; i < WLED_EFFECT_COUNT; i++) {
    if (value == WLED_EFFECTS[i].name) {
      this->bridge_->set_effect(static_cast<uint8_t>(i));
      this->bridge_->publish_light_state();
      this->publish_state(i);
      this->last_published_ = static_cast<uint8_t>(i);
      return;
    }
  }
}

// ============================================================
// WLEDNumber
// ============================================================

void WLEDNumber::setup() {
  this->last_published_ = 255;
}

void WLEDNumber::loop() {
  if (this->bridge_ == nullptr)
    return;
  const EffectParams &p = this->bridge_->get_params();
  uint8_t current;
  switch (this->property_) {
    case WLED_NUM_SPEED:
      current = p.speed;
      break;
    case WLED_NUM_INTENSITY:
      current = p.intensity;
      break;
    default:
      return;
  }
  if (current != this->last_published_) {
    this->publish_state(static_cast<float>(current));
    this->last_published_ = current;
  }
}

void WLEDNumber::control(float value) {
  if (this->bridge_ == nullptr)
    return;
  uint8_t v = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, value)));
  switch (this->property_) {
    case WLED_NUM_SPEED:
      this->bridge_->set_speed(v);
      break;
    case WLED_NUM_INTENSITY:
      this->bridge_->set_intensity(v);
      break;
    default:
      return;
  }
  this->bridge_->publish_light_state();
  this->publish_state(static_cast<float>(v));
  this->last_published_ = v;
}

}  // namespace wled_bridge
}  // namespace esphome

#endif  // WLED_BRIDGE_ENTITIES
