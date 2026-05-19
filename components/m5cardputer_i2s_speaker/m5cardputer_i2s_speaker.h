#pragma once

#include "esphome/core/component.h"

namespace esphome {
namespace m5cardputer_i2s_speaker {

class M5CardputerI2SSpeaker : public Component {
 public:
  void dump_config() override;
  void play_test_tone();

 protected:
  void play_tone_(const char *label, bool stereo, bool right_slot, bool invert_ws);
};

}  // namespace m5cardputer_i2s_speaker
}  // namespace esphome
