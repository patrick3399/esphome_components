#include "aw9523.h"
#include "esphome/core/log.h"

namespace esphome {
namespace aw9523 {

static const char *const TAG = "aw9523";

// AW9523B Register Map (based on AW9523B Datasheet V1.1.1)
enum AW9523Register : uint8_t {
  REG_INPUT_PORT0 = 0x00,   // Input status of P0 (Read Only)
  REG_INPUT_PORT1 = 0x01,   // Input status of P1 (Read Only)
  REG_OUTPUT_PORT0 = 0x02,  // Output status of P0
  REG_OUTPUT_PORT1 = 0x03,  // Output status of P1
  REG_CONFIG_PORT0 = 0x04,  // Direction config for P0 (0=Output, 1=Input, Default: 00H)
  REG_CONFIG_PORT1 = 0x05,  // Direction config for P1 (0=Output, 1=Input, Default: 00H)
  // REG_INT_PORT0 = 0x06,    // Interrupt enable for P0 (0=Enable, 1=Disable, Default: 00H) - Not used in this basic impl.
  // REG_INT_PORT1 = 0x07,    // Interrupt enable for P1 (0=Enable, 1=Disable, Default: 00H) - Not used in this basic impl.
  REG_CHIP_ID = 0x10,       // Chip ID (Default: 23H, Read Only)
  REG_CTL = 0x11,           // Global Control Register (Default: 00H)
                            // D[4] GPOMD: P0 Open-Drain (0, default) or Push-Pull (1)
                            // D[1:0] ISEL: LED current range
  REG_P0_MODE_SWITCH = 0x12, // P0_7~P0_0 Mode: 0=LED, 1=GPIO (Default: FFH)
  REG_P1_MODE_SWITCH = 0x13, // P1_7~P1_0 Mode: 0=LED, 1=GPIO (Default: FFH)
  // REG_SW_RSTN = 0x7F       // Software Reset (Write 00H to reset) - Not used in this basic impl.
};

void AW9523::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AW9523B...");

  uint8_t id = 0;
  if (!this->read_byte_wrapper(REG_CHIP_ID, &id)) {
    ESP_LOGE(TAG, "Chip ID read failed. Check I2C address (0x%02X) and wiring.", this->address_);
    this->mark_failed();
    return;
  }

  if (id != 0x23) {
    ESP_LOGE(TAG, "AW9523B not found or wrong ID (expected 0x23, got 0x%02X).", id);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "AW9523B CHIP_ID: 0x%02X. Found at I2C address 0x%02X", id, this->address_);

  // Configure CTL (0x11H)
  // D[4] GPOMD: Set P0 to Push-Pull mode (1). Default is Open-Drain (0).
  // D[1:0] ISEL: Keep default 00 (0~IMAX for LED, less relevant for GPIO only).
  // Reserved bits D[7:5], D[3:2] must be 0.
  // Value to write: 0b00010000 = 0x10
  uint8_t ctl_val = 0x10;
  ESP_LOGD(TAG, "Writing 0x%02X to CTL (0x11) to set P0 to Push-Pull.", ctl_val);
  if (!this->write_byte_wrapper(REG_CTL, ctl_val)) {
    ESP_LOGE(TAG, "Failed to write CTL register.");
    this->mark_failed();
    return;
  }

  // Configure P0 and P1 pins to GPIO mode (REG_P0_MODE_SWITCH = 0x12, REG_P1_MODE_SWITCH = 0x13)
  // Datasheet: 1 = GPIO mode, 0 = LED mode. Default is 0xFF (all GPIO).
  // Writing 0xFF ensures this state.
  ESP_LOGD(TAG, "Writing 0xFF to P0_MODE_SWITCH (0x12) to set P0 pins to GPIO mode.");
  if (!this->write_byte_wrapper(REG_P0_MODE_SWITCH, 0xFF)) {
    ESP_LOGE(TAG, "Failed to set Port 0 to GPIO mode.");
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "Writing 0xFF to P1_MODE_SWITCH (0x13) to set P1 pins to GPIO mode.");
  if (!this->write_byte_wrapper(REG_P1_MODE_SWITCH, 0xFF)) {
    ESP_LOGE(TAG, "Failed to set Port 1 to GPIO mode.");
    this->mark_failed();
    return;
  }

  // Set all pins to INPUT direction initially (REG_CONFIG_PORT0 = 0x04, REG_CONFIG_PORT1 = 0x05)
  // Datasheet: 0 = Output mode, 1 = Input mode. Default is 0x00 (all Output).
  // ESPHome components usually set all to input, then pin_mode configures specifics.
  ESP_LOGD(TAG, "Writing 0xFF to CONFIG_PORT0 (0x04) to set P0 pins to INPUT direction.");
  if (!this->write_byte_wrapper(REG_CONFIG_PORT0, 0xFF)) {
    ESP_LOGE(TAG, "Failed to set Port 0 direction to input.");
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "Writing 0xFF to CONFIG_PORT1 (0x05) to set P1 pins to INPUT direction.");
  if (!this->write_byte_wrapper(REG_CONFIG_PORT1, 0xFF)) {
    ESP_LOGE(TAG, "Failed to set Port 1 direction to input.");
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "AW9523B setup successful.");
}

void AW9523::dump_config() {
  ESP_LOGCONFIG(TAG, "AW9523B:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with AW9523B failed!");
  }
}

void AW9523::pin_mode(uint8_t pin, gpio::Flags flags) {
  uint8_t reg = pin < 8 ? REG_CONFIG_PORT0 : REG_CONFIG_PORT1; // Pins 0-7 are P0, 8-15 are P1
  uint8_t bit_in_reg = pin % 8;
  bool is_input_mode = false;

  if ((flags & gpio::FLAG_INPUT) && !(flags & gpio::FLAG_OUTPUT)) {
    is_input_mode = true; // For AW9523B Config reg: 1 = Input
  } else if ((flags & gpio::FLAG_OUTPUT) && !(flags & gpio::FLAG_INPUT)) {
    is_input_mode = false; // For AW9523B Config reg: 0 = Output
  } else {
    ESP_LOGW(TAG, "Pin %u: Invalid mode flags 0x%X. Defaulting to input.", pin, static_cast<uint8_t>(flags));
    is_input_mode = true;
  }

  ESP_LOGD(TAG, "Setting pin P%d_%d (abs %u) to %s mode.", (pin < 8 ? 0 : 1), bit_in_reg, pin, is_input_mode ? "INPUT" : "OUTPUT");
  if (!this->update_register_bit(reg, bit_in_reg, is_input_mode)) {
      ESP_LOGE(TAG, "Failed to set pin mode for pin %u", pin);
  }
}

bool AW9523::digital_read(uint8_t pin) {
  uint8_t reg = pin < 8 ? REG_INPUT_PORT0 : REG_INPUT_PORT1;
  uint8_t bit_in_reg = pin % 8;
  uint8_t port_value = 0;

  if (!this->read_byte_wrapper(reg, &port_value)) {
    ESP_LOGW(TAG, "Failed to read input register for pin %u", pin);
    return false; // Or handle error appropriately
  }
  return (port_value >> bit_in_reg) & 0x01;
}

void AW9523::digital_write(uint8_t pin, bool value) {
  uint8_t reg = pin < 8 ? REG_OUTPUT_PORT0 : REG_OUTPUT_PORT1;
  uint8_t bit_in_reg = pin % 8;

  ESP_LOGV(TAG, "Writing %s to pin P%d_%d (abs %u).", ONOFF(value), (pin < 8 ? 0 : 1), bit_in_reg, pin);
  if (!this->update_register_bit(reg, bit_in_reg, value)) {
      ESP_LOGE(TAG, "Failed to write output register for pin %u", pin);
  }
}

bool AW9523::read_byte_wrapper(uint8_t reg, uint8_t *value) {
  if (this->read_byte(reg, value) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C Read from reg 0x%02X failed!", reg);
    return false;
  }
  return true;
}

bool AW9523::write_byte_wrapper(uint8_t reg, uint8_t value) {
  if (this->write_byte(reg, value) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "I2C Write to reg 0x%02X with value 0x%02X failed!", reg, value);
    return false;
  }
  return true;
}

bool AW9523::update_register_bit(uint8_t reg, uint8_t bit, bool bit_value) {
  uint8_t current_val;
  if (!this->read_byte_wrapper(reg, &current_val)) {
    return false;
  }
  if (bit_value) {
    current_val |= (1 << bit);
  } else {
    current_val &= ~(1 << bit);
  }
  return this->write_byte_wrapper(reg, current_val);
}

// AW9523GPIOPin implementation
void AW9523GPIOPin::setup() {
  ESP_LOGV(TAG, "Setting up AW9523GPIOPin %u", pin_);
  this->pin_mode(flags_); // Initial mode setup
}

void AW9523GPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->pin_mode(this->pin_, flags);
}

bool AW9523GPIOPin::digital_read() {
  return this->parent_->digital_read(this->pin_) != this->inverted_;
}

void AW9523GPIOPin::digital_write(bool value) {
  this->parent_->digital_write(this->pin_, value != this->inverted_);
}

std::string AW9523GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "Pin %u (AW9523)", pin_);
  return buffer;
}

}  // namespace aw9523
}  // namespace esphome
