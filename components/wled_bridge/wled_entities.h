#pragma once
#include "esphome/core/defines.h"
#ifdef WLED_BRIDGE_ENTITIES

#include "esphome/core/component.h"
#include "esphome/components/select/select.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace wled_bridge {

class WLEDBridgeComponent;  // forward

// ---- Palette Select entity ----
class WLEDPaletteSelect : public select::Select, public Component {
 public:
  void set_bridge(WLEDBridgeComponent *bridge) {
    this->bridge_ = bridge;
  }
  void setup() override;
  void loop() override;
  float get_setup_priority() const override {
    return setup_priority::DATA;
  }

 protected:
  void control(const std::string &value) override;

  WLEDBridgeComponent *bridge_{nullptr};
  uint8_t last_published_{255};
};

// ---- Effect Select entity ----
// Exposes the active effect as a Home Assistant Select dropdown.
class WLEDEffectSelect : public select::Select, public Component {
 public:
  void set_bridge(WLEDBridgeComponent *bridge) {
    this->bridge_ = bridge;
  }
  void setup() override;
  void loop() override;
  float get_setup_priority() const override {
    return setup_priority::DATA;
  }

 protected:
  void control(const std::string &value) override;

  WLEDBridgeComponent *bridge_{nullptr};
  uint8_t last_published_{255};
};

// ---- Generic uint8 Number entity ----
// Used for speed, intensity, custom1/2/3, etc.
enum WLEDNumberProperty : uint8_t {
  WLED_NUM_SPEED = 0,
  WLED_NUM_INTENSITY = 1,
};

class WLEDNumber : public number::Number, public Component {
 public:
  void set_bridge(WLEDBridgeComponent *bridge) {
    this->bridge_ = bridge;
  }
  void set_property(WLEDNumberProperty prop) {
    this->property_ = prop;
  }
  void setup() override;
  void loop() override;
  float get_setup_priority() const override {
    return setup_priority::DATA;
  }

 protected:
  void control(float value) override;

  WLEDBridgeComponent *bridge_{nullptr};
  WLEDNumberProperty property_{WLED_NUM_SPEED};
  uint8_t last_published_{255};
};

// ---- Preset Select entity ----
class WLEDPresetSelect : public select::Select, public Component {
 public:
  void set_bridge(WLEDBridgeComponent *bridge) {
    this->bridge_ = bridge;
  }
  void setup() override;
  void loop() override;
  float get_setup_priority() const override {
    return setup_priority::LATE - 10.0f;
  }

 protected:
  void control(const std::string &value) override;
  void rebuild_options_();

  WLEDBridgeComponent *bridge_{nullptr};
  uint8_t last_published_{255};
  uint32_t last_option_check_ms_{0};
  uint16_t last_valid_mask_{0};
  uint8_t option_preset_ids_[16]{};
  uint8_t option_count_{0};
  char name_buf_[16][24]{};
};

// ---- Nightlight Switch entity ----
class WLEDNightlightSwitch : public switch_::Switch, public Component {
 public:
  void set_bridge(WLEDBridgeComponent *bridge) {
    this->bridge_ = bridge;
  }
  void setup() override;
  void loop() override;
  float get_setup_priority() const override {
    return setup_priority::LATE - 10.0f;
  }

 protected:
  void write_state(bool state) override;

  WLEDBridgeComponent *bridge_{nullptr};
};

// ---- UDP Sync Send Switch entity ----
class WLEDSyncSendSwitch : public switch_::Switch, public Component {
 public:
  void set_bridge(WLEDBridgeComponent *bridge) {
    this->bridge_ = bridge;
  }
  void setup() override;
  void loop() override;
  float get_setup_priority() const override {
    return setup_priority::LATE - 10.0f;
  }

 protected:
  void write_state(bool state) override;

  WLEDBridgeComponent *bridge_{nullptr};
};

// ---- UDP Sync Receive Switch entity ----
class WLEDSyncReceiveSwitch : public switch_::Switch, public Component {
 public:
  void set_bridge(WLEDBridgeComponent *bridge) {
    this->bridge_ = bridge;
  }
  void setup() override;
  void loop() override;
  float get_setup_priority() const override {
    return setup_priority::LATE - 10.0f;
  }

 protected:
  void write_state(bool state) override;

  WLEDBridgeComponent *bridge_{nullptr};
};

// ---- Estimated Current Sensor entity ----
class WLEDCurrentSensor : public sensor::Sensor, public Component {
 public:
  void set_bridge(WLEDBridgeComponent *bridge) {
    this->bridge_ = bridge;
  }
  void setup() override {}
  void loop() override;
  float get_setup_priority() const override {
    return setup_priority::LATE - 10.0f;
  }

 protected:
  WLEDBridgeComponent *bridge_{nullptr};
  uint32_t last_ma_{0xFFFFFFFF};
  uint32_t last_publish_ms_{0};
};

}  // namespace wled_bridge
}  // namespace esphome

#endif  // WLED_BRIDGE_ENTITIES
