#pragma once

#include "esphome/core/component.h"

namespace esphome {
namespace m5cardputer_legacy_speaker {

class M5CardputerLegacySpeaker : public Component {
 public:
  void dump_config() override;
  void play_test_tone();

 protected:
  void play_tone_(bool stereo);
};

}  // namespace m5cardputer_legacy_speaker
}  // namespace esphome
