#include "qmi8658.h"
#include "esphome/core/log.h"

namespace esphome {
namespace qmi8658 {

static const char *const TAG = "qmi8658";

// Accel sensitivity in g/LSB, indexed by AccelRange enum value.
// Multiply by GRAVITY_EARTH to get m/s².
// ±2g→1/16384, ±4g→1/8192, ±8g→1/4096, ±16g→1/2048
static constexpr float ACCEL_SENSITIVITY[4] = {
    1.0f / 16384.0f,
    1.0f / 8192.0f,
    1.0f / 4096.0f,
    1.0f / 2048.0f,
};

// Gyro sensitivity in dps/LSB, indexed by GyroRange enum value.
// ±16dps→1/2048, ±32→1/1024, ±64→1/512, ±128→1/256,
// ±256→1/128, ±512→1/64, ±1024→1/32, ±2048→1/16
static constexpr float GYRO_SENSITIVITY[8] = {
    1.0f / 2048.0f, 1.0f / 1024.0f, 1.0f / 512.0f, 1.0f / 256.0f,
    1.0f / 128.0f,  1.0f / 64.0f,   1.0f / 32.0f,  1.0f / 16.0f,
};

static constexpr float GRAVITY_EARTH = 9.80665f;

void QMI8658Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up QMI8658...");

  // Soft reset: write 0xB0 to RESET register
  uint8_t reset_val = 0xB0;
  if (this->write_register(QMI8658_REG_RESET, &reset_val, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "I2C error during reset");
    this->mark_failed();
    return;
  }

  // Wait 150 ms for device to complete reset, then configure
  this->set_timeout(150, [this]() { this->configure_(); });
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

  // CTRL2: accelerometer config — range in bits[6:4], ODR=0x05 (250 Hz) in bits[3:0]
  uint8_t ctrl2 = static_cast<uint8_t>((this->accel_range_ << 4) | 0x05);
  if (this->write_register(QMI8658_REG_CTRL2, &ctrl2, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "I2C error writing CTRL2");
    this->mark_failed();
    return;
  }

  // CTRL3: gyroscope config — range in bits[6:4], ODR=0x05 (235 Hz) in bits[3:0]
  uint8_t ctrl3 = static_cast<uint8_t>((this->gyro_range_ << 4) | 0x05);
  if (this->write_register(QMI8658_REG_CTRL3, &ctrl3, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "I2C error writing CTRL3");
    this->mark_failed();
    return;
  }

  // CTRL7 = 0x83: syncSmpl=bit7=1 (locks regs during burst read), gEN=bit1=1, aEN=bit0=1
  uint8_t ctrl7 = 0x83;
  if (this->write_register(QMI8658_REG_CTRL7, &ctrl7, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "I2C error writing CTRL7");
    this->mark_failed();
    return;
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

  int16_t raw_temp = raw16(0);
  int16_t raw_ax = raw16(2);
  int16_t raw_ay = raw16(4);
  int16_t raw_az = raw16(6);
  int16_t raw_gx = raw16(8);
  int16_t raw_gy = raw16(10);
  int16_t raw_gz = raw16(12);

  float accel_sens = ACCEL_SENSITIVITY[this->accel_range_] * GRAVITY_EARTH;
  float gyro_sens = GYRO_SENSITIVITY[this->gyro_range_];

  float accel_x = raw_ax * accel_sens;
  float accel_y = raw_ay * accel_sens;
  float accel_z = raw_az * accel_sens;
  float gyro_x = raw_gx * gyro_sens;
  float gyro_y = raw_gy * gyro_sens;
  float gyro_z = raw_gz * gyro_sens;
  float temp_c = raw_temp / 256.0f;

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

  ESP_LOGCONFIG(TAG, "QMI8658:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
    return;
  }
  ESP_LOGCONFIG(TAG, "  Accelerometer range: %s", ACCEL_RANGE_STRINGS[this->accel_range_]);
  ESP_LOGCONFIG(TAG, "  Gyroscope range:     %s", GYRO_RANGE_STRINGS[this->gyro_range_]);
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
