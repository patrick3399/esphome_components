#include "jiecang_desk_button.h"

namespace esphome {
namespace jiecang_desk_controller {

void JiecangDeskButton::press_action() {
  switch (this->action_) {
    case BUTTON_STEP_UP:
      this->parent_->step_up();
      break;
    case BUTTON_STEP_DOWN:
      this->parent_->step_down();
      break;
    case BUTTON_STOP:
      this->parent_->stop();
      break;
    case BUTTON_MOVE_UP:
      this->parent_->move_up();
      break;
    case BUTTON_MOVE_DOWN:
      this->parent_->move_down();
      break;
    case BUTTON_GOTO_M1:
      this->parent_->goto_preset(1);
      break;
    case BUTTON_GOTO_M2:
      this->parent_->goto_preset(2);
      break;
    case BUTTON_GOTO_M3:
      this->parent_->goto_preset(3);
      break;
    case BUTTON_GOTO_M4:
      this->parent_->goto_preset(4);
      break;
    case BUTTON_SAVE_M1:
      this->parent_->save_preset(1);
      break;
    case BUTTON_SAVE_M2:
      this->parent_->save_preset(2);
      break;
    case BUTTON_SAVE_M3:
      this->parent_->save_preset(3);
      break;
    case BUTTON_SAVE_M4:
      this->parent_->save_preset(4);
      break;
  }
}

}  // namespace jiecang_desk_controller
}  // namespace esphome
