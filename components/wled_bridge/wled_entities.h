#pragma once
#include "esphome/core/defines.h"
#ifdef WLED_BRIDGE_ENTITIES

#include "esphome/core/component.h"
#include "esphome/components/select/select.h"
#include "esphome/components/number/number.h"

namespace esphome {
namespace wled_bridge {

class WLEDBridgeComponent;  // forward

// ---- Palette Select entity ----
// Exposes the active palette as a Home Assistant Select dropdown.
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

}  // namespace wled_bridge
}  // namespace esphome

#endif  // WLED_BRIDGE_ENTITIES
