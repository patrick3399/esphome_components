#include "qmi8658.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace qmi8658 {

static const char *const TAG = "qmi8658";

// Accel sensitivity in g/LSB × GRAVITY_EARTH → m/s²
// ±2g→1/16384, ±4g→1/8192, ±8g→1/4096, ±16g→1/2048
static constexpr float ACCEL_SENSITIVITY[4] = {
    1.0f / 16384.0f,
    1.0f / 8192.0f,
    1.0f / 4096.0f,
    1.0f / 2048.0f,
};

// Gyro sensitivity in dps/LSB
// ±16→1/2048, ±32→1/1024, ±64→1/512, ±128→1/256,
// ±256→1/128, ±512→1/64, ±1024→1/32, ±2048→1/16
static constexpr float GYRO_SENSITIVITY[8] = {
    1.0f / 2048.0f, 1.0f / 1024.0f, 1.0f / 512.0f, 1.0f / 256.0f,
    1.0f / 128.0f,  1.0f / 64.0f,   1.0f / 32.0f,  1.0f / 16.0f,
};

static constexpr float GRAVITY_EARTH = 9.80665f;

void QMI8658Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up QMI8658...");

  uint8_t reset_val = 0xB0;
  if (this->write_register(QMI8658_REG_RESET, &reset_val, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "I2C error during reset");
    this->mark_failed();
    return;
  }

  // Wait 150 ms for device to complete reset, then configure
  this->set_timeout(150, [this]() { this->configure_(); });
}

bool QMI8658Component::send_ctrl9_cmd_(uint8_t cmd) {
  if (this->write_register(QMI8658_REG_CTRL9, &cmd, 1) != i2c::ERROR_OK)
    return false;
  if (cmd == 0x00)
    return true;
  // Poll STATUSINT bit7 (Ctrl9CmdDone), max 50ms
  uint8_t status = 0;
  for (int i = 0; i < 50; i++) {
    delay(1);
    if (this->read_register(QMI8658_REG_STATUSINT, &status, 1) != i2c::ERROR_OK)
      return false;
    if (status & 0x80) {
      uint8_t ack = 0x00;
      this->write_register(QMI8658_REG_CTRL9, &ack, 1);
      return true;
    }
  }
  return false;
}

void QMI8658Component::configure_() {
  // Verify WHO_AM_I
  uint8_t who_am_i = 0;
  if (this->read_register(QMI8658_REG_WHO_AM_I, &who_am_i, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "I2C error reading WHO_AM_I");
    this->mark_failed();
    return;
  }
  if (who_am_i != QMI8658_WHO_AM_I_VALUE) {
    ESP_LOGE(TAG, "Unexpected WHO_AM_I: 0x%02X (expected 0x%02X)", who_am_i, QMI8658_WHO_AM_I_VALUE);
    this->mark_failed();
    return;
  }

  // CTRL1 = 0x40: ADDR_AI=1 (auto-increment address for burst reads), little-endian output
  uint8_t ctrl1 = 0x40;
  if (this->write_register(QMI8658_REG_CTRL1, &ctrl1, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "I2C error writing CTRL1");
    this->mark_failed();
    return;
  }

  // CTRL2: accel range in bits[6:4], ODR in bits[3:0]
  uint8_t ctrl2 = static_cast<uint8_t>((this->accel_range_ << 4) | static_cast<uint8_t>(this->accel_odr_));
  if (this->write_register(QMI8658_REG_CTRL2, &ctrl2, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "I2C error writing CTRL2");
    this->mark_failed();
    return;
  }

  // CTRL3: gyro range in bits[6:4], ODR in bits[3:0]
  uint8_t ctrl3 = static_cast<uint8_t>((this->gyro_range_ << 4) | static_cast<uint8_t>(this->gyro_odr_));
  if (this->write_register(QMI8658_REG_CTRL3, &ctrl3, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "I2C error writing CTRL3");
    this->mark_failed();
    return;
  }

  // CTRL5: LPF config (bit0=aLPF_EN, bits[2:1]=aLPF_MODE, bit4=gLPF_EN, bits[6:5]=gLPF_MODE)
  uint8_t ctrl5 = 0x00;
  if (this->accel_lpf_mode_ != LPF_DISABLED)
    ctrl5 |= 0x01 | (static_cast<uint8_t>(this->accel_lpf_mode_) << 1);
  if (this->gyro_lpf_mode_ != LPF_DISABLED)
    ctrl5 |= 0x10 | (static_cast<uint8_t>(this->gyro_lpf_mode_) << 5);
  if (ctrl5 != 0x00) {
    if (this->write_register(QMI8658_REG_CTRL5, &ctrl5, 1) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "I2C error writing CTRL5");
      this->mark_failed();
      return;
    }
  }

  // CTRL7 = 0x83: syncSmpl=bit7=1 (locks regs during burst read), gEN=bit1=1, aEN=bit0=1
  uint8_t ctrl7 = 0x83;
  if (this->write_register(QMI8658_REG_CTRL7, &ctrl7, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "I2C error writing CTRL7");
    this->mark_failed();
    return;
  }

  // Disable AHB clock gating — datasheet requires this after enabling syncSmpl (CTRL7 bit7)
  uint8_t ahb_val = 0x00;
  if (this->write_register(QMI8658_REG_CAL1_L, &ahb_val, 1) == i2c::ERROR_OK) {
    if (!this->send_ctrl9_cmd_(0x08)) {
      ESP_LOGW(TAG, "CTRL9 AHB clock gating disable timed out — syncSmpl may still work");
    }
  }

  this->setup_complete_ = true;
  ESP_LOGI(TAG, "QMI8658 setup complete");
}

void QMI8658Component::update() {
  if (!this->setup_complete_)
    return;

  // Burst-read 14 bytes from TEMP_L (0x33):
  //   buf[0..1]   = TEMP   (little-endian int16)
  //   buf[2..3]   = AX     (little-endian int16)
  //   buf[4..5]   = AY
  //   buf[6..7]   = AZ
  //   buf[8..9]   = GX
  //   buf[10..11] = GY
  //   buf[12..13] = GZ
  uint8_t buf[14];
  if (this->read_register(QMI8658_REG_TEMP_L, buf, 14) != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Failed to read sensor data");
    return;
  }

  auto raw16 = [&](int i) -> int16_t { return static_cast<int16_t>(static_cast<uint16_t>(buf[i + 1]) << 8 | buf[i]); };

  float accel_sens = ACCEL_SENSITIVITY[this->accel_range_] * GRAVITY_EARTH;
  float gyro_sens = GYRO_SENSITIVITY[this->gyro_range_];

  float accel_x = raw16(2) * accel_sens;
  float accel_y = raw16(4) * accel_sens;
  float accel_z = raw16(6) * accel_sens;
  float gyro_x = raw16(8) * gyro_sens;
  float gyro_y = raw16(10) * gyro_sens;
  float gyro_z = raw16(12) * gyro_sens;
  float temp_c = raw16(0) / 256.0f;

  ESP_LOGD(TAG, "accel={x=%.3f m/s², y=%.3f m/s², z=%.3f m/s²} gyro={x=%.3f °/s, y=%.3f °/s, z=%.3f °/s} temp=%.1f°C",
           accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp_c);

  if (this->accel_x_ != nullptr)
    this->accel_x_->publish_state(accel_x);
  if (this->accel_y_ != nullptr)
    this->accel_y_->publish_state(accel_y);
  if (this->accel_z_ != nullptr)
    this->accel_z_->publish_state(accel_z);
  if (this->gyro_x_ != nullptr)
    this->gyro_x_->publish_state(gyro_x);
  if (this->gyro_y_ != nullptr)
    this->gyro_y_->publish_state(gyro_y);
  if (this->gyro_z_ != nullptr)
    this->gyro_z_->publish_state(gyro_z);
  if (this->temperature_ != nullptr)
    this->temperature_->publish_state(temp_c);

  this->status_clear_warning();
}

void QMI8658Component::dump_config() {
  static constexpr const char *ACCEL_RANGE_STRINGS[4] = {"±2g", "±4g", "±8g", "±16g"};
  static constexpr const char *GYRO_RANGE_STRINGS[8] = {
      "±16°/s", "±32°/s", "±64°/s", "±128°/s", "±256°/s", "±512°/s", "±1024°/s", "±2048°/s",
  };
  static constexpr const char *LPF_MODE_STRINGS[4] = {"mode 0", "mode 1", "mode 2", "mode 3"};

  ESP_LOGCONFIG(TAG, "QMI8658:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    return;
  }
  ESP_LOGCONFIG(TAG, "  Accelerometer range: %s  ODR: 0x%02X  LPF: %s",
                ACCEL_RANGE_STRINGS[this->accel_range_], static_cast<uint8_t>(this->accel_odr_),
                this->accel_lpf_mode_ == LPF_DISABLED ? "disabled" : LPF_MODE_STRINGS[this->accel_lpf_mode_]);
  ESP_LOGCONFIG(TAG, "  Gyroscope range:     %s  ODR: 0x%02X  LPF: %s",
                GYRO_RANGE_STRINGS[this->gyro_range_], static_cast<uint8_t>(this->gyro_odr_),
                this->gyro_lpf_mode_ == LPF_DISABLED ? "disabled" : LPF_MODE_STRINGS[this->gyro_lpf_mode_]);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_);
  LOG_SENSOR("  ", "Gyroscope X", this->gyro_x_);
  LOG_SENSOR("  ", "Gyroscope Y", this->gyro_y_);
  LOG_SENSOR("  ", "Gyroscope Z", this->gyro_z_);
  LOG_SENSOR("  ", "Temperature", this->temperature_);
}

}  // namespace qmi8658
}  // namespace esphome
