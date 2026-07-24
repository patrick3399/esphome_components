# qmi8658 (deprecated)

**Deprecated since ESPHome 2026.7.0** — core ESPHome ships its own `qmi8658`
driver built on the `motion` framework (configurable range/ODR, axis remapping,
runtime calibration with NVS persistence, `pitch`/`roll` level sensors). Use that
instead; this copy is kept only for reference and is not loaded by any device
YAML.

Note the units difference when migrating: the core `motion` platform publishes
acceleration in **g** (`UNIT_G`), whereas this legacy component published
**m/s²** — rescale any downstream math by ~9.8 (see how `esp32s3-matrix.yaml`
retuned its spirit-level `scale`).

QMI8658 6-axis IMU (accelerometer + gyroscope + temperature) over I2C. Exposed
range and ODR like the core ESPHome `bmi270` driver, plus low-pass filter
settings that `bmi270` did not have.

## Hardware

- Interface: I2C (address `0x6A` or `0x6B`, default `0x6B`)
- Accelerometer: ±2/4/8/16 g, selectable ODR up to 1000 Hz
- Gyroscope: ±16–2048 °/s, selectable ODR up to 7174 Hz
- Temperature sensor: on-chip

## Configuration

```yaml
qmi8658:
  id: imu
  address: 0x6B           # optional, default 0x6B
  update_interval: 60s    # optional, default 60 s
  accelerometer_range: 4G       # optional: 2G, 4G (default), 8G, 16G
  gyroscope_range: 256DPS       # optional: 16/32/64/128/256(default)/512/1024/2048 DPS
  accelerometer_odr: 250HZ      # optional: 1000/500/250(default)/125/62/31 HZ, 128/21/11/3 HZ_LP
  gyroscope_odr: 224HZ          # optional: 7174/3587/1793/896/448/224(default)/112/56/28 HZ
  accelerometer_lpf_mode: DISABLED   # optional: DISABLED(default), 0, 1, 2, 3
  gyroscope_lpf_mode: DISABLED       # optional: DISABLED(default), 0, 1, 2, 3

sensor:
  - platform: qmi8658
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

## Demos

- [devices/waveshare/esp32s3-matrix.yaml](../../devices/waveshare/esp32s3-matrix.yaml)
- [devices/waveshare/esp32s3-touch-amoled-2.41.yaml](../../devices/waveshare/esp32s3-touch-amoled-2.41.yaml)
