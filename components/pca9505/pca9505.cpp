#include "pca9505.h"
#include "esphome/core/log.h"
#include <cstring> // For memcpy
#include <vector>  // For std::vector in write_pca_registers

namespace esphome {
namespace pca9505 {

static const char *const TAG = "pca9505";

void PCA9505Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PCA9505...");

  for (int i = 0; i < PCA9505_NUM_BANKS; i++) {
    this->io_config_[i] = 0xFF;
    this->output_state_[i] = 0x00;
  }

  uint8_t temp_input[PCA9505_NUM_BANKS];
  // For setup check, we send command for IP0, then read 5 bytes with auto-increment
  if (this->read_pca_registers(PCA9505_REG_INPUT_PORT_BASE, temp_input, PCA9505_NUM_BANKS, true) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "PCA9505 not available under I2C address 0x%02X", this->address_);
    this->mark_failed();
    return;
  }
  memcpy(this->input_state_, temp_input, sizeof(this->input_state_));
  ESP_LOGCONFIG(TAG, "PCA9505 successfully initialized.");
}

void PCA9505Component::dump_config() {
  ESP_LOGCONFIG(TAG, "PCA9505:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with PCA9505 failed!");
  }
  for (int i = 0; i < PCA9505_NUM_BANKS; i++) {
    ESP_LOGCONFIG(TAG, "  Bank %d: IO_Config=0x%02X, Output_State=0x%02X, Input_State=0x%02X",
                  i, this->io_config_[i], this->output_state_[i], this->input_state_[i]);
  }
}

float PCA9505Component::get_setup_priority() const { return setup_priority::IO; }

// Helper to read a single register
esphome::i2c::ErrorCode PCA9505Component::read_pca_register(uint8_t reg, uint8_t *data, bool auto_increment) {
    uint8_t command = reg;
    // PCA9505 command register: AI is bit 7. D5-D6 are 00.
    // If auto_increment is true for a single read, it doesn't hurt but is typically for multi-byte.
    // For single byte read, auto_increment is often set to false in other drivers,
    // but PCA9505 datasheet command register description (Fig 6) shows AI as D7.
    // The default for command register at power up is 0x80 (AI=1, register=0).
    // Let's ensure AI is only set if requested.
    if (auto_increment) {
      command |= PCA9505_CMD_AUTO_INCREMENT;
    }


    // PCA9505 read sequence: S | ADDR+W | ACK | CMD_BYTE | ACK | Sr | ADDR+R | ACK | DATA_BYTE | NACK | P
    // Write command byte, but DO NOT send a STOP condition (send_stop = false)
    i2c::ErrorCode err = this->write(&command, 1, false);
    if (err != i2c::ERROR_OK) {
        ESP_LOGW(TAG, "Failed to write command 0x%02X for read: error %d", command, err);
        return err;
    }
    // Read 1 byte, and the bus will send a STOP condition after this.
    return this->read(data, 1);
}

// Helper to write a single register
esphome::i2c::ErrorCode PCA9505Component::write_pca_register(uint8_t reg, uint8_t value, bool auto_increment) {
    uint8_t command = reg;
    if (auto_increment) {
      command |= PCA9505_CMD_AUTO_INCREMENT;
    }
    
    uint8_t buffer[2] = {command, value};
    // Write 2 bytes (command + data), and send a STOP condition (default for write)
    return this->write(buffer, 2, true);
}

// Helper to read multiple registers
esphome::i2c::ErrorCode PCA9505Component::read_pca_registers(uint8_t start_reg, uint8_t *data, uint8_t len, bool auto_increment) {
    uint8_t command = start_reg;
    if (auto_increment) { // This should generally be true for multi-byte reads
        command |= PCA9505_CMD_AUTO_INCREMENT;
    }

    // Write command byte, DO NOT send a STOP condition (send_stop = false)
    i2c::ErrorCode err = this->write(&command, 1, false);
    if (err != i2c::ERROR_OK) {
        ESP_LOGW(TAG, "Failed to write command 0x%02X for multi-read: error %d", command, err);
        return err;
    }
    // Read 'len' bytes, and the bus will send a STOP condition after this.
    return this->read(data, len);
}

// Helper to write multiple registers
esphome::i2c::ErrorCode PCA9505Component::write_pca_registers(uint8_t start_reg, uint8_t *values, uint8_t len, bool auto_increment) {
    uint8_t command = start_reg;
    if (auto_increment) { // This should generally be true for multi-byte writes
        command |= PCA9505_CMD_AUTO_INCREMENT;
    }

    std::vector<uint8_t> buffer;
    buffer.push_back(command);
    for (uint8_t i = 0; i < len; i++) {
        buffer.push_back(values[i]);
    }
    // Write (1+len) bytes, and send a STOP condition (default for write)
    return this->write(buffer.data(), buffer.size(), true);
}


bool PCA9505Component::read_bank_input(uint8_t bank) {
  if (bank >= PCA9505_NUM_BANKS) return false;
  // For reading a single bank's input, auto-increment is not strictly necessary
  i2c::ErrorCode err = this->read_pca_register(PCA9505_REG_INPUT_PORT_BASE + bank, &this->input_state_[bank], false);
  if (err != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to read input bank %d: error %d", bank, err);
    this->status_set_warning();
    return false;
  }
  this->status_clear_warning();
  return true;
}

bool PCA9505Component::write_bank_output(uint8_t bank) {
  if (bank >= PCA9505_NUM_BANKS) return false;
  // For writing a single bank's output, auto-increment is not strictly necessary
  i2c::ErrorCode err = this->write_pca_register(PCA9505_REG_OUTPUT_PORT_BASE + bank, this->output_state_[bank], false);
  if (err != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to write output bank %d: error %d", bank, err);
    this->status_set_warning();
    return false;
  }
  this->status_clear_warning();
  return true;
}

bool PCA9505Component::write_bank_config(uint8_t bank) {
  if (bank >= PCA9505_NUM_BANKS) return false;
  // For writing a single bank's config, auto-increment is not strictly necessary
  i2c::ErrorCode err = this->write_pca_register(PCA9505_REG_IO_CONFIG_BASE + bank, this->io_config_[bank], false);
   if (err != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to write IO_config bank %d: error %d", bank, err);
    this->status_set_warning();
    return false;
  }
  this->status_clear_warning();
  return true;
}

bool PCA9505Component::digital_read(uint8_t pin) {
  if (pin >= (PCA9505_NUM_BANKS * PCA9505_PINS_PER_BANK)) return false;
  uint8_t bank = pin / PCA9505_PINS_PER_BANK;
  uint8_t pin_in_bank = pin % PCA9505_PINS_PER_BANK;

  if (!this->read_bank_input(bank)) {
    return false;
  }
  return (this->input_state_[bank] >> pin_in_bank) & 1;
}

void PCA9505Component::digital_write(uint8_t pin, bool value) {
  if (pin >= (PCA9505_NUM_BANKS * PCA9505_PINS_PER_BANK)) return;
  uint8_t bank = pin / PCA9505_PINS_PER_BANK;
  uint8_t pin_in_bank = pin % PCA9505_PINS_PER_BANK;

  if ((this->io_config_[bank] >> pin_in_bank) & 1) {
     ESP_LOGW(TAG, "Pin %d (Bank %d, Pin %d) is configured as input. Cannot write.", pin, bank, pin_in_bank);
     return;
  }

  if (value) {
    this->output_state_[bank] |= (1 << pin_in_bank);
  } else {
    this->output_state_[bank] &= ~(1 << pin_in_bank);
  }

  if (!this->write_bank_output(bank)) {
    // Error logged in write_bank_output
  }
}

void PCA9505Component::pin_mode(uint8_t pin, gpio::Flags flags) {
  if (pin >= (PCA9505_NUM_BANKS * PCA9505_PINS_PER_BANK)) return;
  uint8_t bank = pin / PCA9505_PINS_PER_BANK;
  uint8_t pin_in_bank = pin % PCA9505_PINS_PER_BANK;

  // Corrected flag checking:
  if (flags == gpio::FLAG_INPUT) {
    this->io_config_[bank] |= (1 << pin_in_bank); // Set bit for input
  } else if (flags == gpio::FLAG_OUTPUT) {
    this->io_config_[bank] &= ~(1 << pin_in_bank); // Clear bit for output
  }
  // Note: ESPHome's gpio::Flags can also include PULLUP, PULLDOWN, OPEN_DRAIN.
  // This basic implementation only handles INPUT and OUTPUT direction.
  // PCA9505 has internal pull-ups (not PCA9506). Open-drain is not directly
  // configured on PCA9505 per pin (outputs are totem-pole).

  if (!this->write_bank_config(bank)) {
    // Error logged in write_bank_config
  }
}

// PCA9505GPIOPin Implementations
void PCA9505GPIOPin::setup() { this->pin_mode(this->flags_); }

void PCA9505GPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->pin_mode(this->pin_, flags);
}

bool PCA9505GPIOPin::digital_read() {
  return this->parent_->digital_read(this->pin_) != this->inverted_;
}

void PCA9505GPIOPin::digital_write(bool value) {
  this->parent_->digital_write(this->pin_, value != this->inverted_);
}

std::string PCA9505GPIOPin::dump_summary() const {
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "Pin %u (Bank %u Pin %u) via PCA9505",
           pin_, pin_ / PCA9505_PINS_PER_BANK, pin_ % PCA9505_PINS_PER_BANK);
  return buffer;
}

}  // namespace pca9505
}  // namespace esphome