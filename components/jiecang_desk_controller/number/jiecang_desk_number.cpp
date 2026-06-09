#include "jiecang_desk_number.h"

namespace esphome {
namespace jiecang_desk_controller {

void JiecangDeskNumber::control(float value) {
  this->publish_state(value);
  switch (this->type_) {
    case NUMBER_HEIGHT_CM:
      this->parent_->goto_height(value);
      break;
    case NUMBER_HEIGHT_PCT:
      this->parent_->goto_height_pct(value);
      break;
  }
}

}  // namespace jiecang_desk_controller
}  // namespace esphome
