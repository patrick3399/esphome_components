#include "aw9523b.h"
#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/progmem.h"

namespace esphome {
namespace aw9523b {

static const uint8_t AW9523B_INPUT_P0    = 0x00;
static const uint8_t AW9523B_INPUT_P1    = 0x01;
static const uint8_t AW9523B_OUTPUT_P0   = 0x02;
static const uint8_t AW9523B_OUTPUT_P1   = 0x03;
static const uint8_t AW9523B_CONFIG_P0   = 0x04;
static const uint8_t AW9523B_CONFIG_P1   = 0x05;
static const uint8_t AW9523B_INT_P0      = 0x06;
static const uint8_t AW9523B_INT_P1      = 0x07;
static const uint8_t AW9523B_ID          = 0x10;
static const uint8_t AW9523B_CTL         = 0x11;
static const uint8_t AW9523B_LED_MODE_P0 = 0x12;
static const uint8_t AW9523B_LED_MODE_P1 = 0x13;
// DIM registers: pin order is P0_0..P0_7 then P1_0..P1_7
// but the register addresses are non-sequential across ports
static const uint8_t AW9523B_DIM_REGS[16] = {
  0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b,  // P0_0..P0_7
  0x20, 0x21, 0x22, 0x23, 0x2c, 0x2d, 0x2e, 0x2f,  // P1_0..P1_7
};
static const uint8_t AW9523B_SW_RST = 0x7F;

static const char *const TAG = "aw9523b";

#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
PROGMEM_STRING_TABLE(P0DriveModeStrings, "OPEN_DRAIN", "PUSH_PULL");
static const LogString *p0_drive_mode_to_string(AW9523BP0DriveMode mode) {
  return P0DriveModeStrings::get_log_str(mode, 0);
}
PROGMEM_STRING_TABLE(LEDMaxCurrentStrings, "37mA", "27.75mA", "18.5mA", "9.25mA");
static const LogString *led_max_current_to_string(AW9523BLEDMaxCurrent current) {
  return LEDMaxCurrentStrings::get_log_str(current, 0);
}
#endif

#define AW9523B_ERROR_FAILED(expr) \
  if (!(expr)) { \
    this->mark_failed(); \
    return; \
  }

void AW9523BComponent::setup() {
  ESP_LOGD(TAG, "Setting up AW9523B...");

  if (this->reset_) {
    AW9523B_ERROR_FAILED(this->write_byte(AW9523B_SW_RST, 0x00));
    delay(2);
    // After soft reset, hardware output latches = 0xFFFF; sync our shadow to match.
    this->output_mask_ = 0xFFFF;
    uint16_t all_disabled = 0xFFFF;
    AW9523B_ERROR_FAILED(this->write_bytes(AW9523B_INT_P0, reinterpret_cast<uint8_t *>(&all_disabled), 2));
  }

  uint8_t chip_id;
  AW9523B_ERROR_FAILED(this->read_byte(AW9523B_ID, &chip_id));
  if (chip_id != 0x23) {
    ESP_LOGE(TAG, "Invalid chip ID: expected 0x23, got 0x%02X", chip_id);
    this->mark_failed();
    return;
  }

  uint8_t ctl;
  AW9523B_ERROR_FAILED(this->read_byte(AW9523B_CTL, &ctl));
  if (this->p0_drive_mode_ == PUSH_PULL)
    ctl |= (1 << 4);
  else
    ctl &= ~(1 << 4);
  ctl &= ~0x03;
  ctl |= static_cast<uint8_t>(this->led_max_current_) & 0x03;
  AW9523B_ERROR_FAILED(this->write_byte(AW9523B_CTL, ctl));

  if (!this->reset_) {
    if (!this->read_gpio_modes_()) { this->mark_failed(); return; }
    if (!this->read_gpio_outputs_()) { this->mark_failed(); return; }
    if (!this->read_led_modes_()) { this->mark_failed(); return; }
  }

  if (this->interrupt_pin_ != nullptr) {
    this->interrupt_pin_->setup();
    this->interrupt_pin_->attach_interrupt(&AW9523BComponent::gpio_intr, this, gpio::INTERRUPT_FALLING_EDGE);
    this->set_invalidate_on_read_(false);
  }
  this->disable_loop();

  ESP_LOGI(TAG, "AW9523B setup complete.");
}

void AW9523BComponent::loop() {
  this->reset_pin_cache_();
  if (this->interrupt_pin_ != nullptr && this->interrupt_pin_->digital_read()) {
    this->disable_loop();
  }
}

void AW9523BComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "AW9523B:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  " ESP_LOG_MSG_COMM_FAIL);
    return;
  }
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  ESP_LOGCONFIG(TAG, "  P0 drive mode: %s", LOG_STR_ARG(p0_drive_mode_to_string(this->p0_drive_mode_)));
  ESP_LOGCONFIG(TAG, "  LED max current: %s", LOG_STR_ARG(led_max_current_to_string(this->led_max_current_)));
#endif
  LOG_PIN("  Interrupt Pin: ", this->interrupt_pin_);
}

void IRAM_ATTR AW9523BComponent::gpio_intr(AW9523BComponent *arg) {
  arg->enable_loop_soon_any_context();
}

void AW9523BComponent::pin_mode(uint8_t pin, gpio::Flags flags) {
  uint16_t mask = 1 << pin;
  if (flags & gpio::FLAG_INPUT)
    this->mode_mask_ |= mask;
  else if (flags & gpio::FLAG_OUTPUT)
    this->mode_mask_ &= ~mask;
  this->write_gpio_modes_();
}

bool AW9523BComponent::read_gpio_modes_() {
  if (this->is_failed()) return false;
  uint8_t data[2];
  if (!this->read_bytes(AW9523B_CONFIG_P0, data, 2)) {
    this->status_set_warning(LOG_STR("Failed to read GPIO modes"));
    return false;
  }
  this->status_clear_warning();
  this->mode_mask_ = data[0] | (data[1] << 8);
  return true;
}

bool AW9523BComponent::write_gpio_modes_() {
  if (this->is_failed()) return false;
  if (!this->write_bytes(AW9523B_CONFIG_P0, reinterpret_cast<uint8_t *>(&this->mode_mask_), 2)) {
    this->status_set_warning(LOG_STR("Failed to write GPIO modes"));
    return false;
  }
  this->status_clear_warning();
  return true;
}

bool AW9523BComponent::read_gpio_outputs_() {
  if (this->is_failed()) return false;
  uint8_t data[2];
  if (!this->read_bytes(AW9523B_OUTPUT_P0, data, 2)) {
    this->status_set_warning(LOG_STR("Failed to read output registers"));
    return false;
  }
  this->status_clear_warning();
  this->output_mask_ = data[0] | (data[1] << 8);
  return true;
}

bool AW9523BComponent::read_gpio_interrupts_() {
  if (this->is_failed()) return false;
  uint8_t data[2];
  if (!this->read_bytes(AW9523B_INT_P0, data, 2)) {
    this->status_set_warning(LOG_STR("Failed to read interrupt registers"));
    return false;
  }
  this->status_clear_warning();
  this->interrupt_mask_ = data[0] | (data[1] << 8);
  return true;
}

bool AW9523BComponent::read_led_modes_() {
  if (this->is_failed()) return false;
  uint8_t data[2];
  if (!this->read_bytes(AW9523B_LED_MODE_P0, data, 2)) {
    this->status_set_warning(LOG_STR("Failed to read LED mode registers"));
    return false;
  }
  this->status_clear_warning();
  this->led_mode_mask_ = data[0] | (data[1] << 8);
  return true;
}

bool AW9523BComponent::digital_read_hw(uint8_t pin) {
  if (this->is_failed()) return false;
  uint8_t data[2];
  if (!this->read_bytes(AW9523B_INPUT_P0, data, 2)) {
    this->status_set_warning(LOG_STR("Failed to read input registers"));
    return false;
  }
  this->status_clear_warning();
  this->input_mask_ = data[0] | (data[1] << 8);
  return true;
}

bool AW9523BComponent::digital_read_cache(uint8_t pin) {
  return (this->input_mask_ >> pin) & 0x01;
}

void AW9523BComponent::digital_write_hw(uint8_t pin, bool value) {
  if (this->is_failed()) return;
  uint16_t new_mask = this->output_mask_;
  if (value)
    new_mask |= (1 << pin);
  else
    new_mask &= ~(1 << pin);
  if (!this->write_bytes(AW9523B_OUTPUT_P0, reinterpret_cast<uint8_t *>(&new_mask), 2)) {
    this->status_set_warning(LOG_STR("Failed to write output registers"));
    return;
  }
  this->output_mask_ = new_mask;
  this->status_clear_warning();
}

bool AW9523BComponent::setup_led_mode(uint8_t pin) {
  if (this->is_failed()) return false;
  this->led_mode_mask_ &= ~(1 << pin);
  if (!this->write_bytes(AW9523B_LED_MODE_P0, reinterpret_cast<uint8_t *>(&this->led_mode_mask_), 2)) {
    this->status_set_warning(LOG_STR("Failed to write LED mode"));
    return false;
  }
  this->status_clear_warning();
  return true;
}

void AW9523BComponent::write_led_current(uint8_t pin, uint8_t current) {
  if (this->is_failed()) return;
  if (!this->write_byte(AW9523B_DIM_REGS[pin], current)) {
    this->status_set_warning(LOG_STR("Failed to write LED current"));
    return;
  }
  this->status_clear_warning();
}

void AW9523BComponent::setup_gpio_interrupt(uint8_t pin, bool enable) {
  bool ok = enable ? this->enable_gpio_interrupt_(pin) : this->disable_gpio_interrupt_(pin);
  if (!ok)
    ESP_LOGW(TAG, "Failed to configure interrupt for pin %d", pin);
}

bool AW9523BComponent::enable_gpio_interrupt_(uint8_t pin) {
  this->interrupt_mask_ &= ~(1 << pin);
  if (!this->write_bytes(AW9523B_INT_P0, reinterpret_cast<uint8_t *>(&this->interrupt_mask_), 2)) {
    this->status_set_warning(LOG_STR("Failed to write interrupt config"));
    return false;
  }
  this->status_clear_warning();
  return true;
}

bool AW9523BComponent::disable_gpio_interrupt_(uint8_t pin) {
  this->interrupt_mask_ |= (1 << pin);
  if (!this->write_bytes(AW9523B_INT_P0, reinterpret_cast<uint8_t *>(&this->interrupt_mask_), 2)) {
    this->status_set_warning(LOG_STR("Failed to write interrupt config"));
    return false;
  }
  this->status_clear_warning();
  return true;
}

void AW9523BGPIOPin::setup() {
  this->pin_mode(this->flags_);
  this->parent_->setup_gpio_interrupt(this->pin_, this->use_interrupt_);
}

void AW9523BGPIOPin::pin_mode(gpio::Flags flags) {
  this->parent_->pin_mode(this->pin_, flags);
}

bool AW9523BGPIOPin::digital_read() {
  return this->parent_->digital_read(this->pin_) != this->inverted_;
}

void AW9523BGPIOPin::digital_write(bool value) {
  this->parent_->digital_write(this->pin_, value != this->inverted_);
}

size_t AW9523BGPIOPin::dump_summary(char *buffer, size_t len) const {
  return snprintf(buffer, len, "%u via AW9523B", this->pin_);
}

}  // namespace aw9523b
}  // namespace esphome
