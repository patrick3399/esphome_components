# bmi270

Bosch BMI270 6-axis IMU (accelerometer + gyroscope + temperature) over I2C.

## Hardware

- Interface: I2C (address `0x68` or `0x69`, default `0x68`)
- Accelerometer: ±2/4/8/16 g, 3-axis
- Gyroscope: up to ±2000 °/s, 3-axis
- Temperature sensor: on-chip

## Configuration

```yaml
bmi270:
  id: imu
  address: 0x68          # optional, default 0x68
  update_interval: 60s   # optional, default 60 s
  power_save_mode: NORMAL  # optional: NORMAL (default) or LOW_POWER

sensor:
  - platform: bmi270
    acceleration_x:
      name: "Accel X"
    acceleration_y:
      name: "Accel Y"
    acceleration_z:
      name: "Accel Z"
    gyroscope_x:
      name: "Gyro X"
    gyroscope_y:
      name: "Gyro Y"
    gyroscope_z:
      name: "Gyro Z"
    temperature:
      name: "IMU Temp"
```

All sensor keys are optional — only declare the axes you need.

| Option | Default | Notes |
|---|---|---|
| `power_save_mode` | `NORMAL` | `LOW_POWER` reduces ODR and power draw |
| `update_interval` | `60s` | Polling rate for all sensors |

## Demo

[devices/m5stack/m5papers3.yaml](../../devices/m5stack/m5papers3.yaml)
