#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace pca9505 {

// PCA9505 Register Map (with Auto-Increment bit D7=0, D5-D6=0)
// See Table 4 in PCA9505 Datasheet [cite: 56]
const uint8_t PCA9505_REG_INPUT_PORT_BASE = 0x00;    // IP0-IP4 (0x00-0x04)
const uint8_t PCA9505_REG_OUTPUT_PORT_BASE = 0x08;   // OP0-OP4 (0x08-0x0C)
const uint8_t PCA9505_REG_POLARITY_INV_BASE = 0x10; // PI0-PI4 (0x10-0x14)
const uint8_t PCA9505_REG_IO_CONFIG_BASE = 0x18;    // IOC0-IOC4 (0x18-0x1C)
const uint8_t PCA9505_REG_MASK_INT_BASE = 0x20;     // MSK0-MSK4 (0x20-0x24)

const uint8_t PCA9505_CMD_AUTO_INCREMENT = 0x80; // Bit D7 for auto-increment

const int PCA9505_NUM_BANKS = 5;
const int PCA9505_PINS_PER_BANK = 8;

class PCA9505Component : public Component, public i2c::I2CDevice {
 public:
  PCA9505Component() = default;

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  bool digital_read(uint8_t pin);
  void digital_write(uint8_t pin, bool value);
  void pin_mode(uint8_t pin, gpio::Flags flags);

 protected:
  bool read_bank_input(uint8_t bank);
  bool write_bank_output(uint8_t bank);
  bool write_bank_config(uint8_t bank);
  // Optional: bool write_bank_polarity(uint8_t bank);
  // Optional: bool write_bank_interrupt_mask(uint8_t bank);

  esphome::i2c::ErrorCode read_pca_register(uint8_t reg, uint8_t *data, bool auto_increment = false);
  esphome::i2c::ErrorCode write_pca_register(uint8_t reg, uint8_t value, bool auto_increment = false);
  esphome::i2c::ErrorCode read_pca_registers(uint8_t start_reg, uint8_t *data, uint8_t len, bool auto_increment = true);
  esphome::i2c::ErrorCode write_pca_registers(uint8_t start_reg, uint8_t *values, uint8_t len, bool auto_increment = true);


  uint8_t input_state_[PCA9505_NUM_BANKS]{0};     // Stores last read input values for each bank
  uint8_t output_state_[PCA9505_NUM_BANKS]{0};    // Stores desired output values for each bank [cite: 75] (defaults to 0x00)
  uint8_t io_config_[PCA9505_NUM_BANKS];     // Stores I/O configuration (1=input, 0=output) [cite: 89] (defaults to 0xFF)
  // Optional: uint8_t polarity_inv_[PCA9505_NUM_BANKS]{0}; // [cite: 80] (defaults to 0x00)
  // Optional: uint8_t interrupt_mask_[PCA9505_NUM_BANKS]; // [cite: 95] (defaults to 0xFF)
};

class PCA9505GPIOPin : public GPIOPin {
 public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  void set_parent(PCA9505Component *parent) { parent_ = parent; }
  void set_pin(uint8_t pin) { pin_ = pin; }
  void set_inverted(bool inverted) { inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { flags_ = flags; }
  gpio::Flags get_flags() const override { return this->flags_; }

 protected:
  PCA9505Component *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace pca9505
}  // namespace esphome