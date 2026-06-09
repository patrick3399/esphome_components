#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/helpers.h"
#include "../jiecang_desk_controller.h"

namespace esphome {
namespace jiecang_desk_controller {

enum NumberType : uint8_t {
  NUMBER_HEIGHT_CM = 0,
  NUMBER_HEIGHT_PCT,
};

class JiecangDeskNumber : public number::Number, public Parented<JiecangDeskController> {
 public:
  void set_type(uint8_t type) {
    this->type_ = static_cast<NumberType>(type);
  }

 protected:
  void control(float value) override;

  NumberType type_{NUMBER_HEIGHT_CM};
};

}  // namespace jiecang_desk_controller
}  // namespace esphome
