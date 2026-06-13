# ESPHome Components

Custom ESPHome components and device YAMLs.

## Components

| Component | Purpose | README |
| --- | --- | --- |
| `amg8833` | Panasonic AMG8833 8×8 thermal camera — JPEG camera, temp sensors, presence detection | [README](components/amg8833/README.md) |
| `mcu90640` | GY-MCU90640 32×24 thermal camera module (MLX90640 + STM32) over UART | [README](components/mcu90640/README.md) |
| `wled_bridge` | WLED v16-compatible bridge for ESPHome addressable LED strips | [README](components/wled_bridge/README.md) |
| `74hc138_keypad` | Keyboard matrix scanner for 74HC138 decoder-based designs | [README](components/74hc138_keypad/README.md) |
| `aw9523` | AW9523B I2C GPIO expander with GPIO and LED-driver support | [README](components/aw9523/README.md) |
| `bmi270` | Bosch BMI270 6-axis IMU sensor | [README](components/bmi270/README.md) |
| `ed047tc1` | ED047TC1 4.7" e-paper display driver using EPDIY | [README](components/ed047tc1/README.md) |
| `esp_sr_wake_word` | Experimental ESP-SR WakeNet wake word component | [README](components/esp_sr_wake_word/README.md) |
| `i2s_audio` | Local override of ESPHome `i2s_audio` with `output_gain` speaker support | [README](components/i2s_audio/README.md) |
| `jiecang_desk_controller` | Jiecang sit-stand desk controller over UART | [README](components/jiecang_desk_controller/README.md) |
| `qmi8658` | QMI8658 6-axis IMU sensor | [README](components/qmi8658/README.md) |
| `ys_irtm_uart` | YS-IRTM UART infrared transceiver: NEC TX/RX, HA IR/RF Proxy bridge | [README](components/ys_irtm_uart/README.md) |

## Devices

### M5Stack — [devices/m5stack/](devices/m5stack/README.md)

| Device | YAML | Key features | Local components |
| --- | --- | --- | --- |
| M5Stack Paper S3 | [m5papers3.yaml](devices/m5stack/m5papers3.yaml) | ED047TC1 e-paper, LVGL, BMI270 IMU | `ed047tc1`, `bmi270` |
| M5Stack StamPLC | [m5stamplc.yaml](devices/m5stack/m5stamplc.yaml) | Relay outputs, digital inputs, ST7789 | `aw9523` |
| M5Stack CoreS3 SE | [m5cores3se.yaml](devices/m5stack/m5cores3se.yaml) | Voice assistant, speaker, LVGL | none (M5Stack official) |
| M5Stack CoreS3 SE ESP-SR | [m5cores3se-espsr.yaml](devices/m5stack/m5cores3se-espsr.yaml) | Voice assistant (ESP-SR WakeNet), speaker, LVGL | `esp_sr_wake_word` |
| M5Stack Cardputer | [m5cardputer.yaml](devices/m5stack/m5cardputer.yaml) | 74HC138 keyboard, HA conversation, IR, RTTTL | `74hc138_keypad`, `i2s_audio` |

### Waveshare — [devices/waveshare/](devices/waveshare/README.md)

| Device | YAML | Key features | Local components |
| --- | --- | --- | --- |
| ESP32-S3 Matrix | [esp32s3-matrix.yaml](devices/waveshare/esp32s3-matrix.yaml) | 8×8 WS2812B matrix, QMI8658 IMU | `qmi8658` |
| ESP32-S3 Touch AMOLED 2.41 | [esp32s3-touch-amoled-2.41.yaml](devices/waveshare/esp32s3-touch-amoled-2.41.yaml) | AMOLED, touch, QMI8658 IMU, LVGL | `qmi8658` |
| ESP32-S3 Touch LCD 1.85C | [esp32s3-touch-lcd-1.85c.yaml](devices/waveshare/esp32s3-touch-lcd-1.85c.yaml) | QSPI LCD, touch, voice assistant | none |
| ESP32-S3 ETH | [esp32s3-eth.yaml](devices/waveshare/esp32s3-eth.yaml) | W5500 Ethernet, camera | none |
| ESP32-S3 GEEK | [esp32s3-geek.yaml](devices/waveshare/esp32s3-geek.yaml) | ST7789 display, WiFi diagnostics | none |
| ESP32-P4 WiFi6 Touch LCD 10.1 | [esp32p4-wifi6-touch-lcd-10.1.yaml](devices/waveshare/esp32p4-wifi6-touch-lcd-10.1.yaml) | P4+C6, MIPI-DSI, voice assistant | none |
