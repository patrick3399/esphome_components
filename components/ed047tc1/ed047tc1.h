#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/display/display_buffer.h"

#include "epdiy.h"
#include "epd_highlevel.h"
#include "epd_board.h"
#include "epd_display.h"
#include "output_lcd/lcd_driver.h"


#ifndef EPD_WIDTH
#define EPD_WIDTH 960
#endif
#ifndef EPD_HEIGHT
#define EPD_HEIGHT 540
#endif

namespace esphome {
namespace ed047tc1 {

class ED047TC1Display : public display::DisplayBuffer {
 public:
  static ED047TC1Display* instance;

  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return esphome::setup_priority::HARDWARE; }

  void set_pwr_pin(GPIOPin *pwr_pin) { pwr_pin_ = pwr_pin; }
  void set_bst_en_pin(GPIOPin *bst_en_pin) { bst_en_pin_ = bst_en_pin; }
  void set_xstl_pin(GPIOPin *xstl_pin) { xstl_pin_ = xstl_pin; }
  void set_pclk_pin(GPIOPin *pclk_pin) { pclk_pin_ = pclk_pin; }
  void set_xle_pin(GPIOPin *xle_pin) { xle_pin_ = xle_pin; }
  void set_spv_pin(GPIOPin *spv_pin) { spv_pin_ = spv_pin; }
  void set_ckv_pin(GPIOPin *ckv_pin) { ckv_pin_ = ckv_pin; }
  void set_data_pins(const std::vector<GPIOPin*>& data_pins) {
    for (size_t i = 0; i < data_pins.size() && i < 8; ++i) {
        d_pins_[i] = data_pins[i];
    }
  }

  display::DisplayType get_display_type() override { return display::DisplayType::DISPLAY_TYPE_GRAYSCALE; }

  GPIOPin *pwr_pin_{nullptr};
  GPIOPin *bst_en_pin_{nullptr};
  GPIOPin *xstl_pin_{nullptr};
  GPIOPin *pclk_pin_{nullptr};
  GPIOPin *xle_pin_{nullptr};
  GPIOPin *spv_pin_{nullptr};
  GPIOPin *ckv_pin_{nullptr};
  std::array<GPIOPin*, 8> d_pins_{};

 protected:
  void draw_absolute_pixel_internal(int x, int y, Color color) override;
  int get_width_internal() override { return EPD_WIDTH; }
  int get_height_internal() override { return EPD_HEIGHT; }
  int get_bpp() const { return 8; }

  EpdiyHighlevelState hl_state_;
  uint32_t esphome_buffer_size_{0};
};

}  // namespace ed047tc1
}  // namespace esphome