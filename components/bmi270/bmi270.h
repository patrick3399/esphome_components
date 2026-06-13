#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace bmi270 {

enum PowerSaveMode {
  POWER_SAVE_MODE_NORMAL = 0,
  POWER_SAVE_MODE_LOW_POWER = 1,
};

class BMI270Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;

  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) {
    this->accel_x_sensor_ = accel_x_sensor;
  }
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) {
    this->accel_y_sensor_ = accel_y_sensor;
  }
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) {
    this->accel_z_sensor_ = accel_z_sensor;
  }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) {
    this->gyro_x_sensor_ = gyro_x_sensor;
  }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) {
    this->gyro_y_sensor_ = gyro_y_sensor;
  }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) {
    this->gyro_z_sensor_ = gyro_z_sensor;
  }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) {
    this->temperature_sensor_ = temperature_sensor;
  }
  void set_power_save_mode(PowerSaveMode mode) {
    this->power_save_mode_ = mode;
  }

 protected:
  bool load_config_file_();
  bool apply_power_save_mode_();
  bool write_checked_(uint8_t reg, const uint8_t *data, size_t length, const char *operation);

  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  PowerSaveMode power_save_mode_{POWER_SAVE_MODE_NORMAL};
  bool setup_complete_{false};
  uint8_t setup_status_{0xFF};  // INTERNAL_STATUS at init; 0xFF = not yet reached
};

}  // namespace bmi270
}  // namespace esphome
