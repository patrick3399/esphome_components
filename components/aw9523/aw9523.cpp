#include "aw9523.h"
#include "esphome/core/log.h"

namespace esphome {
namespace aw9523 {

static const char *const TAG = "aw9523";

void AW9523Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AW9523...");

  uint8_t chip_id = 0;
  if (!this->read_reg(AW9523_REG_CHIPID, &chip_id) || chip_id != 0x23) {
    ESP_LOGE(TAG, "Invalid Chip ID: 0x%02X (expected 0x23)", chip_id);
    this->mark_failed();
    return;
  }

  // Soft reset — all registers return to datasheet defaults
  if (!this->write_reg(AW9523_REG_SOFTRESET, 0x00)) {
    ESP_LOGE(TAG, "Failed to reset AW9523");
    this->mark_failed();
    return;
  }

  // Disable all interrupts immediately after reset.
  // INTENABLE reset value is 0x00 (all enabled), which asserts INT on any pin change.
  // 0xFF = all disabled.
  if (!this->write_reg(AW9523_REG_INTENABLE0, 0xFF) || !this->write_reg(AW9523_REG_INTENABLE1, 0xFF)) {
    ESP_LOGE(TAG, "Failed to disable AW9523 interrupts");
    this->mark_failed();
    return;
  }

  // Set all pins to GPIO mode (reset value is already 0xFF, but be explicit)
  if (!this->write_reg(AW9523_REG_LEDMODE0, 0xFF) || !this->write_reg(AW9523_REG_LEDMODE1, 0xFF)) {
    ESP_LOGE(TAG, "Failed to initialize AW9523 GPIO mode");
    this->mark_failed();
    return;
  }

  // GCR: set P0 push-pull (bit 4) and apply IMAX divider (bits [1:0])
  if (!this->write_reg(AW9523_REG_GCR, (1 << 4) | (this->imax_divider_ & 0x03))) {
    ESP_LOGE(TAG, "Failed to configure AW9523 global control");
    this->mark_failed();
    return;
  }

  // Seed output shadow from hardware (reset value is 0xFF, but read to be safe)
  if (!this->read_reg(AW9523_REG_OUTPUT0, &this->output_mask_0_) ||
      !this->read_reg(AW9523_REG_OUTPUT1, &this->output_mask_1_)) {
    ESP_LOGE(TAG, "Failed to read AW9523 output latches");
    this->mark_failed();
    return;
  }

  // Set all pins as input (1=input in CONFIG register)
  this->dir_mask_0_ = 0xFF;
  this->dir_mask_1_ = 0xFF;
  if (!this->write_reg(AW9523_REG_CONFIG0, this->dir_mask_0_) ||
      !this->write_reg(AW9523_REG_CONFIG1, this->dir_mask_1_)) {
    ESP_LOGE(TAG, "Failed to initialize AW9523 pin directions");
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "AW9523 setup complete.");
}

void AW9523Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AW9523:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with AW9523 failed!");
    return;
  }
  ESP_LOGCONFIG(TAG, "  IMAX divider: %u (max ~%u mA)", this->imax_divider_,
                (unsigned)(37 * (4 - this->imax_divider_) / 4));
  ESP_LOGCONFIG(TAG, "  Latch inputs: %s", this->latch_inputs_ ? "YES" : "NO");
  if (this->latch_inputs_)
    ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", (unsigned)this->get_update_interval());
}

void AW9523Component::update() {
  // Skip if inputs are read directly on demand
  if (!this->latch_inputs_)
    return;

  if (!this->read_reg(AW9523_REG_INPUT0, &this->input_mask_0_) ||
      !this->read_reg(AW9523_REG_INPUT1, &this->input_mask_1_)) {
    ESP_LOGW(TAG, "Failed to read input ports");
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();
}

bool AW9523Component::digital_read(uint8_t pin) {
  uint8_t port = pin / 8;
  uint8_t bit = pin % 8;

  if (!this->latch_inputs_) {
    uint8_t val = 0;
    if (!this->read_reg(port == 0 ? AW9523_REG_INPUT0 : AW9523_REG_INPUT1, &val)) {
      ESP_LOGW(TAG, "Failed to read input for pin %u", pin);
      this->status_set_warning();
      return ((port == 0 ? this->input_mask_0_ : this->input_mask_1_) >> bit) & 0x01;
    }
    if (port == 0)
      this->input_mask_0_ = val;
    else
      this->input_mask_1_ = val;
    this->status_clear_warning();
    return (val >> bit) & 0x01;
  }

  return ((port == 0 ? this->input_mask_0_ : this->input_mask_1_) >> bit) & 0x01;
}

void AW9523Component::digital_write(uint8_t pin, bool value) {
  uint8_t port = pin / 8;
  uint8_t bit = pin % 8;
  uint8_t &mask = port == 0 ? this->output_mask_0_ : this->output_mask_1_;
  uint8_t reg = port == 0 ? AW9523_REG_OUTPUT0 : AW9523_REG_OUTPUT1;
  uint8_t new_mask = mask;

  if (value) {
    new_mask |= (1 << bit);
  } else {
    new_mask &= ~(1 << bit);
  }

  if (!this->write_reg(reg, new_mask)) {
    ESP_LOGW(TAG, "Failed to write output for pin %u", pin);
    this->status_set_warning();
    return;
  }
  mask = new_mask;
  this->status_clear_warning();
}

void AW9523Component::pin_mode(uint8_t pin, gpio::Flags flags) {
  uint8_t port = pin / 8;
  uint8_t bit = pin % 8;

  // Ensure GPIO mode using shadow (avoids hardware read on every pin_mode call)
  uint8_t &ledmode = port == 0 ? this->ledmode_mask_0_ : this->ledmode_mask_1_;
  uint8_t ledmode_reg = port == 0 ? AW9523_REG_LEDMODE0 : AW9523_REG_LEDMODE1;
  const uint8_t new_ledmode = ledmode | (1 << bit);  // 1 = GPIO mode
  if (!this->write_reg(ledmode_reg, new_ledmode)) {
    ESP_LOGW(TAG, "Failed to set GPIO mode for pin %u", pin);
    this->status_set_warning();
    return;
  }
  ledmode = new_ledmode;

  // Set direction
  uint8_t &dir = port == 0 ? this->dir_mask_0_ : this->dir_mask_1_;
  uint8_t config_reg = port == 0 ? AW9523_REG_CONFIG0 : AW9523_REG_CONFIG1;
  uint8_t new_dir = dir;
  if (flags & gpio::FLAG_INPUT) {
    new_dir |= (1 << bit);  // 1 = input
  } else {
    new_dir &= ~(1 << bit);  // 0 = output
  }
  if (!this->write_reg(config_reg, new_dir)) {
    ESP_LOGW(TAG, "Failed to set direction for pin %u", pin);
    this->status_set_warning();
    return;
  }
  dir = new_dir;
  this->status_clear_warning();
}

void AW9523Component::led_mode(uint8_t pin) {
  if (pin > 15)
    return;
  uint8_t port = pin / 8;
  uint8_t bit = pin % 8;

  // Switch to LED (constant-current) mode: clear bit in LEDMODE register
  uint8_t &ledmode = port == 0 ? this->ledmode_mask_0_ : this->ledmode_mask_1_;
  uint8_t ledmode_reg = port == 0 ? AW9523_REG_LEDMODE0 : AW9523_REG_LEDMODE1;
  const uint8_t new_ledmode = ledmode & ~(1 << bit);  // 0 = LED mode
  if (!this->write_reg(ledmode_reg, new_ledmode)) {
    ESP_LOGW(TAG, "Failed to set LED mode for pin %u", pin);
    this->status_set_warning();
    return;
  }
  ledmode = new_ledmode;

  // LED mode requires the CONFIG direction bit to be output (0)
  uint8_t &dir = port == 0 ? this->dir_mask_0_ : this->dir_mask_1_;
  uint8_t config_reg = port == 0 ? AW9523_REG_CONFIG0 : AW9523_REG_CONFIG1;
  const uint8_t new_dir = dir & ~(1 << bit);
  if (!this->write_reg(config_reg, new_dir)) {
    ESP_LOGW(TAG, "Failed to set LED output direction for pin %u", pin);
    this->status_set_warning();
    return;
  }
  dir = new_dir;
  this->status_clear_warning();
}

void AW9523Component::set_led_value(uint8_t pin, uint8_t value) {
  if (pin > 15)
    return;
  if (!this->write_reg(AW9523_DIM_REGS[pin], value)) {
    ESP_LOGW(TAG, "Failed to set LED value for pin %u", pin);
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();
}

bool AW9523Component::read_reg(uint8_t reg, uint8_t *value) {
  if (this->is_failed())
    return false;
  return this->read_byte(reg, value);
}

bool AW9523Component::write_reg(uint8_t reg, uint8_t value) {
  if (this->is_failed())
    return false;
  return this->write_byte(reg, value);
}

void AW9523GPIOPin::setup() {
  this->pin_mode(this->flags_);
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
  snprintf(buffer, sizeof(buffer), "%u via AW9523", this->pin_);
  return buffer;
}

}  // namespace aw9523
}  // namespace esphome
