#pragma once

#include "esphome/components/output/float_output.h"
#include "../aw9523b.h"

namespace esphome {
namespace aw9523b {

class AW9523BFloatOutput : public output::FloatOutput, public Component {
 public:
  void setup() override;
  void dump_config() override;
  void set_parent(AW9523BComponent *parent) { this->parent_ = parent; }
  void set_pin(uint8_t pin) { this->pin_ = pin; }

 protected:
  void write_state(float state) override;
  AW9523BComponent *parent_;
  uint8_t pin_;
};

}  // namespace aw9523b
}  // namespace esphome
