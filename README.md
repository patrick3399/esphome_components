# ESPHome Components

Custom ESPHome components and demo YAMLs.

## Components

Components live under [components/](components/).

| Component | Purpose | Demo usage |
| --- | --- | --- |
| `74hc138_keypad` | Keyboard matrix scanner for 74HC138 decoder based designs. | M5Stack Cardputer keyboard input. |
| `aw9523` | I2C GPIO expander with GPIO and LED-driver support. | M5Stack StamPLC relay/input expander. |
| `bmi270` | Bosch BMI270 6-axis IMU sensor. | M5Stack Paper S3 motion sensors. |
| `ed047tc1` | ED047TC1 4.7 inch e-paper display driver using EPDIY. | M5Stack Paper S3 LVGL e-paper UI. |
| `esp_sr_wake_word` | Experimental ESP-SR wake word component. | CoreS3 SE ESP-SR voice assistant variant. |
| `i2s_audio` | Local override of ESPHome `i2s_audio` speaker support. | M5Stack Cardputer speaker `output_gain`. |
| `pca9505` | I2C GPIO expander. | Available, not used by the current demo YAMLs. |
| `qmi8658` | QMI8658 6-axis IMU sensor. | Waveshare ESP32-S3 Matrix and Touch AMOLED demos. |

## Demo Device YAMLs

Demo YAMLs live under [devices/](devices/).

| Device | YAML | Demo features | Local components |
| --- | --- | --- | --- |
| M5Stack Paper S3 | [devices/m5stack/m5papers3.yaml](devices/m5stack/m5papers3.yaml) | ED047TC1 e-paper display, LVGL clock/status pages, touch navigation, battery/charging status, BMI270 IMU page, debug/system page, RTTTL page. | `ed047tc1`, `bmi270` |
| M5Stack StamPLC | [devices/m5stack/m5stamplc.yaml](devices/m5stack/m5stamplc.yaml) | Relay outputs, digital inputs, board temperature/current sensors, ST7789 display text, RTC sync, RTTTL service. | `aw9523` |
| M5Stack CoreS3 SE | [devices/m5stack/m5cores3se.yaml](devices/m5stack/m5cores3se.yaml) | Voice assistant, on-device wake word through ESPHome `micro_wake_word`, speaker media player, microphone mute, LCD/touch LVGL UI, PMU/battery sensors. | None; uses M5Stack official components. |
| M5Stack CoreS3 SE ESP-SR | [devices/m5stack/m5cores3se-espsr.yaml](devices/m5stack/m5cores3se-espsr.yaml) | Experimental voice assistant variant using ESP-SR wake word models, speaker media player, LCD/touch LVGL UI, PMU/battery sensors. | `esp_sr_wake_word` |
| M5Stack Cardputer | [devices/m5stack/m5cardputer.yaml](devices/m5stack/m5cardputer.yaml) | Keyboard text input, typed Home Assistant conversation flow, voice assistant push-to-talk, ST7789 status/debug display, RTTTL, IR transmitter, speaker media player. | `74hc138_keypad`, `i2s_audio` |
| Waveshare ESP32-S3 Matrix | [devices/waveshare/esp32s3-matrix.yaml](devices/waveshare/esp32s3-matrix.yaml) | QMI8658 IMU sensors and 8x8 WS2812B matrix display/brightness demo. | `qmi8658` |
| Waveshare ESP32-S3 Touch AMOLED 2.41 | [devices/waveshare/esp32s3-touch-amoled-2.41.yaml](devices/waveshare/esp32s3-touch-amoled-2.41.yaml) | 450x600 AMOLED display, touch, RTC, QMI8658 IMU sensors, LVGL UI, Bluetooth proxy, diagnostics. | `qmi8658` |
| Waveshare ESP32-S3 Touch LCD 1.85C | [devices/waveshare/esp32s3-touch-lcd-1.85c.yaml](devices/waveshare/esp32s3-touch-lcd-1.85c.yaml) | QSPI LCD, touch, RTC, microphone/speaker voice assistant, wake word selection, Bluetooth proxy, battery and audio diagnostics. | None currently. |
| Waveshare ESP32-S3 ETH | [devices/waveshare/esp32s3-eth.yaml](devices/waveshare/esp32s3-eth.yaml) | W5500 Ethernet, camera, Ethernet info sensors, Bluetooth proxy, boot button, diagnostics. | None. |
| Waveshare ESP32-S3 GEEK | [devices/waveshare/esp32s3-geek.yaml](devices/waveshare/esp32s3-geek.yaml) | ST7789 display, backlight control, WiFi info, Bluetooth proxy, diagnostics. | None. |
| Guition ESP32-S3-4848S040 | TBD | Display, touch, relays. | TBD |
| Guition JC3636K518 | TBD | QSPI display, touch, audio, encoder. | TBD |
| Guition JC3636W518 | TBD | QSPI display, touch, audio. | TBD |
| Waveshare ESP32-S3-ETH-8DI-8RO | TBD | Ethernet, digital inputs, relay outputs, RS485. | TBD |
| Waveshare ESP32-S3-Relay-6CH | TBD | Relay outputs, RS485. | TBD |
| Wireless-Tag WT32-SC01 / Plus | TBD | Display, touch, RS485, audio. | TBD |

`secrets.yaml` files beside the demo YAMLs are local secret material and are not
device examples.
