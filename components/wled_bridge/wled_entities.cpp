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

// ============================================================
// WLEDPresetSelect
// ============================================================

void WLEDPresetSelect::setup() {
  this->rebuild_options_();
  uint8_t active = this->bridge_->get_active_preset();
  if (active > 0) {
    for (uint8_t i = 0; i < this->option_count_; i++) {
      if (this->option_preset_ids_[i] == active) {
        this->publish_state(this->traits.get_options()[i]);
        this->last_published_ = active;
        break;
      }
    }
  }
}

void WLEDPresetSelect::loop() {
  if (this->bridge_ == nullptr)
    return;

  uint32_t now = millis();
  if (now - this->last_option_check_ms_ > 5000) {
    uint16_t mask = 0;
    for (uint8_t i = 0; i < 16; i++) {
      if (this->bridge_->is_preset_valid(i + 1))
        mask |= (1u << i);
    }
    if (mask != this->last_valid_mask_)
      this->rebuild_options_();
    this->last_option_check_ms_ = now;
  }

  uint8_t current = this->bridge_->get_active_preset();
  if (current != this->last_published_) {
    if (current > 0) {
      for (uint8_t i = 0; i < this->option_count_; i++) {
        if (this->option_preset_ids_[i] == current) {
          this->publish_state(this->traits.get_options()[i]);
          break;
        }
      }
    }
    this->last_published_ = current;
  }
}

void WLEDPresetSelect::control(const std::string &value) {
  if (this->bridge_ == nullptr)
    return;
  const auto &options = this->traits.get_options();
  for (size_t i = 0; i < options.size(); i++) {
    if (options[i] == value && this->option_preset_ids_[i] > 0) {
      this->bridge_->load_preset(this->option_preset_ids_[i]);
      this->bridge_->publish_light_state();
      this->publish_state(value);
      this->last_published_ = this->option_preset_ids_[i];
      return;
    }
  }
}

void WLEDPresetSelect::rebuild_options_() {
  this->option_count_ = 0;

  for (uint8_t i = 1; i <= 16; i++) {
    if (!this->bridge_->is_preset_valid(i))
      continue;
    const auto *preset = this->bridge_->get_preset(i);
    if (preset != nullptr && preset->name[0] != '\0') {
      snprintf(this->name_buf_[this->option_count_], sizeof(this->name_buf_[0]), "%s", preset->name);
    } else {
      snprintf(this->name_buf_[this->option_count_], sizeof(this->name_buf_[0]), "Preset %u", i);
    }
    this->option_preset_ids_[this->option_count_] = i;
    this->option_count_++;
  }
  if (this->option_count_ == 0) {
    snprintf(this->name_buf_[0], sizeof(this->name_buf_[0]), "(no presets)");
    this->option_preset_ids_[0] = 0;
    this->option_count_ = 1;
  }

  FixedVector<const char *> opts;
  opts.init(this->option_count_);
  for (uint8_t i = 0; i < this->option_count_; i++)
    opts.push_back(this->name_buf_[i]);
  this->traits.set_options(opts);

  uint16_t mask = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (this->bridge_->is_preset_valid(i + 1))
      mask |= (1u << i);
  }
  this->last_valid_mask_ = mask;
  ESP_LOGD(TAG, "Preset Select: %u valid presets", this->option_count_);
}

// ============================================================
// WLEDNightlightSwitch
// ============================================================

void WLEDNightlightSwitch::setup() {
  if (this->bridge_ != nullptr)
    this->publish_state(this->bridge_->is_nightlight_active());
}

void WLEDNightlightSwitch::loop() {
  if (this->bridge_ == nullptr)
    return;
  bool current = this->bridge_->is_nightlight_active();
  if (current != this->state) {
    this->publish_state(current);
  }
}

void WLEDNightlightSwitch::write_state(bool state) {
  if (this->bridge_ == nullptr)
    return;
  if (state) {
    this->bridge_->start_nightlight(this->bridge_->get_nightlight_duration_s(),
                                    this->bridge_->get_nightlight_target_brightness(),
                                    this->bridge_->get_nightlight_mode());
  } else {
    this->bridge_->stop_nightlight();
  }
  this->publish_state(state);
}

// ============================================================
// WLEDSyncSendSwitch
// ============================================================

void WLEDSyncSendSwitch::setup() {
  if (this->bridge_ != nullptr)
    this->publish_state(this->bridge_->get_udp_send());
}

void WLEDSyncSendSwitch::loop() {
  if (this->bridge_ == nullptr)
    return;
  bool current = this->bridge_->get_udp_send();
  if (current != this->state) {
    this->publish_state(current);
  }
}

void WLEDSyncSendSwitch::write_state(bool state) {
  if (this->bridge_ == nullptr)
    return;
  this->bridge_->set_udp_send_enabled(state);
  this->publish_state(state);
}

// ============================================================
// WLEDSyncReceiveSwitch
// ============================================================

void WLEDSyncReceiveSwitch::setup() {
  if (this->bridge_ != nullptr)
    this->publish_state(this->bridge_->get_udp_receive());
}

void WLEDSyncReceiveSwitch::loop() {
  if (this->bridge_ == nullptr)
    return;
  bool current = this->bridge_->get_udp_receive();
  if (current != this->state) {
    this->publish_state(current);
  }
}

void WLEDSyncReceiveSwitch::write_state(bool state) {
  if (this->bridge_ == nullptr)
    return;
  this->bridge_->set_udp_receive_enabled(state);
  this->publish_state(state);
}

// ============================================================
// WLEDCurrentSensor
// ============================================================

void WLEDCurrentSensor::loop() {
  if (this->bridge_ == nullptr)
    return;
  uint32_t now = millis();
  if (now - this->last_publish_ms_ < 2000)
    return;
  uint32_t ma = this->bridge_->get_current_ma();
  if (ma != this->last_ma_) {
    this->publish_state(static_cast<float>(ma));
    this->last_ma_ = ma;
    this->last_publish_ms_ = now;
  }
}

}  // namespace wled_bridge
}  // namespace esphome

#endif  // WLED_BRIDGE_ENTITIES
