#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace qmi8658 {

// Register map
static constexpr uint8_t QMI8658_REG_WHO_AM_I = 0x00;
static constexpr uint8_t QMI8658_REG_CTRL1 = 0x02;
static constexpr uint8_t QMI8658_REG_CTRL2 = 0x03;
static constexpr uint8_t QMI8658_REG_CTRL3 = 0x04;
static constexpr uint8_t QMI8658_REG_CTRL5 = 0x06;
static constexpr uint8_t QMI8658_REG_CTRL7 = 0x08;
static constexpr uint8_t QMI8658_REG_CTRL9 = 0x0A;
static constexpr uint8_t QMI8658_REG_CAL1_L = 0x0B;
static constexpr uint8_t QMI8658_REG_STATUSINT = 0x2D;
static constexpr uint8_t QMI8658_REG_TEMP_L = 0x33;
static constexpr uint8_t QMI8658_REG_RESET = 0x60;

static constexpr uint8_t QMI8658_WHO_AM_I_VALUE = 0x05;

// Regular enums (not enum class) — required for ESPHome codegen
enum AccelRange : uint8_t {
  ACCEL_RANGE_2G = 0,
  ACCEL_RANGE_4G = 1,
  ACCEL_RANGE_8G = 2,
  ACCEL_RANGE_16G = 3,
};

enum GyroRange : uint8_t {
  GYRO_RANGE_16DPS = 0,
  GYRO_RANGE_32DPS = 1,
  GYRO_RANGE_64DPS = 2,
  GYRO_RANGE_128DPS = 3,
  GYRO_RANGE_256DPS = 4,
  GYRO_RANGE_512DPS = 5,
  GYRO_RANGE_1024DPS = 6,
  GYRO_RANGE_2048DPS = 7,
};

// Accel ODR register values for CTRL2 bits[3:0]
enum AccelODR : uint8_t {
  ACCEL_ODR_1000HZ = 0x03,
  ACCEL_ODR_500HZ = 0x04,
  ACCEL_ODR_250HZ = 0x05,
  ACCEL_ODR_125HZ = 0x06,
  ACCEL_ODR_62HZ = 0x07,
  ACCEL_ODR_31HZ = 0x08,
  ACCEL_ODR_128HZ_LP = 0x0C,
  ACCEL_ODR_21HZ_LP = 0x0D,
  ACCEL_ODR_11HZ_LP = 0x0E,
  ACCEL_ODR_3HZ_LP = 0x0F,
};

// Gyro ODR register values for CTRL3 bits[3:0]
enum GyroODR : uint8_t {
  GYRO_ODR_7174HZ = 0x00,
  GYRO_ODR_3587HZ = 0x01,
  GYRO_ODR_1793HZ = 0x02,
  GYRO_ODR_896HZ = 0x03,
  GYRO_ODR_448HZ = 0x04,
  GYRO_ODR_224HZ = 0x05,
  GYRO_ODR_112HZ = 0x06,
  GYRO_ODR_56HZ = 0x07,
  GYRO_ODR_28HZ = 0x08,
};

// LPF mode for CTRL5; 0xFF = disabled
enum LpfMode : uint8_t {
  LPF_DISABLED = 0xFF,
  LPF_MODE_0 = 0,
  LPF_MODE_1 = 1,
  LPF_MODE_2 = 2,
  LPF_MODE_3 = 3,
};

class QMI8658Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override {
    return setup_priority::DATA;
  }

  void set_accel_range(AccelRange r) {
    this->accel_range_ = r;
  }
  void set_gyro_range(GyroRange r) {
    this->gyro_range_ = r;
  }
  void set_accel_odr(AccelODR odr) {
    this->accel_odr_ = odr;
  }
  void set_gyro_odr(GyroODR odr) {
    this->gyro_odr_ = odr;
  }
  void set_accel_lpf_mode(LpfMode m) {
    this->accel_lpf_mode_ = m;
  }
  void set_gyro_lpf_mode(LpfMode m) {
    this->gyro_lpf_mode_ = m;
  }

  void set_accel_x_sensor(sensor::Sensor *s) {
    this->accel_x_ = s;
  }
  void set_accel_y_sensor(sensor::Sensor *s) {
    this->accel_y_ = s;
  }
  void set_accel_z_sensor(sensor::Sensor *s) {
    this->accel_z_ = s;
  }
  void set_gyro_x_sensor(sensor::Sensor *s) {
    this->gyro_x_ = s;
  }
  void set_gyro_y_sensor(sensor::Sensor *s) {
    this->gyro_y_ = s;
  }
  void set_gyro_z_sensor(sensor::Sensor *s) {
    this->gyro_z_ = s;
  }
  void set_temperature_sensor(sensor::Sensor *s) {
    this->temperature_ = s;
  }

 protected:
  void configure_();
  bool send_ctrl9_cmd_(uint8_t cmd);

  AccelRange accel_range_{ACCEL_RANGE_4G};
  GyroRange gyro_range_{GYRO_RANGE_256DPS};
  AccelODR accel_odr_{ACCEL_ODR_250HZ};
  GyroODR gyro_odr_{GYRO_ODR_224HZ};
  LpfMode accel_lpf_mode_{LPF_DISABLED};
  LpfMode gyro_lpf_mode_{LPF_DISABLED};
  bool setup_complete_{false};

  sensor::Sensor *accel_x_{nullptr};
  sensor::Sensor *accel_y_{nullptr};
  sensor::Sensor *accel_z_{nullptr};
  sensor::Sensor *gyro_x_{nullptr};
  sensor::Sensor *gyro_y_{nullptr};
  sensor::Sensor *gyro_z_{nullptr};
  sensor::Sensor *temperature_{nullptr};
};

}  // namespace qmi8658
}  // namespace esphome
