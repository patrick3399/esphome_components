#pragma once

#include "esphome/components/button/button.h"
#include "esphome/core/helpers.h"
#include "../jiecang_desk_controller.h"

namespace esphome {
namespace jiecang_desk_controller {

enum ButtonAction : uint8_t {
  BUTTON_STEP_UP = 0,
  BUTTON_STEP_DOWN,
  BUTTON_STOP,
  BUTTON_MOVE_UP,
  BUTTON_MOVE_DOWN,
  BUTTON_GOTO_M1,
  BUTTON_GOTO_M2,
  BUTTON_GOTO_M3,
  BUTTON_GOTO_M4,
  BUTTON_SAVE_M1,
  BUTTON_SAVE_M2,
  BUTTON_SAVE_M3,
  BUTTON_SAVE_M4,
};

class JiecangDeskButton : public button::Button, public Parented<JiecangDeskController> {
 public:
  void set_action(uint8_t action) {
    this->action_ = static_cast<ButtonAction>(action);
  }

 protected:
  void press_action() override;

  ButtonAction action_{BUTTON_STOP};
};

}  // namespace jiecang_desk_controller
}  // namespace esphome
