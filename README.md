# ESPHome Components

Custom ESPHome components and demo YAMLs.

## Components

Components live under [components/](components/).

| Component | Purpose | Demo usage |
| --- | --- | --- |
| `amg8833` | Panasonic AMG8833 8×8 thermal camera: iron false-color JPEG camera entity, avg/min/max/thermistor temperature sensors. | [devices/test/amg8833_poc.yaml](devices/test/amg8833_poc.yaml) |
| `mcu90640` | GY-MCU90640 32×24 thermal camera module (**MLX90640 sensor + onboard STM32**) over **UART**: iron false-color JPEG camera entity, avg/min/max/ambient temperature, heat centroid, presence/hot-spot binary sensors. ⚠️ Not for bare MLX90640 I2C breakouts — this module's STM32 owns the sensor's I2C bus (driving it as a second I2C master corrupts reads and locks the bus) and streams pre-computed temperatures over UART instead. | [devices/test/mcu90640_poc.yaml](devices/test/mcu90640_poc.yaml) |
| `wled_bridge` | WLED v16-compatible bridge for ESPHome addressable LED strips: fixed 220 WLED mode slots with supported-effect mapping, 54 palettes, Web UI, HA entities, UDP sync, DDP/E1.31/Art-Net, and sound reactive options. | [devices/test/wled_bridge_poc.yaml](devices/test/wled_bridge_poc.yaml), [devices/test/wled_bridge_multibus_poc.yaml](devices/test/wled_bridge_multibus_poc.yaml) |
| `74hc138_keypad` | Keyboard matrix scanner for 74HC138 decoder based designs. | M5Stack Cardputer keyboard input. |
| `aw9523` | I2C GPIO expander with GPIO and LED-driver support. | M5Stack StamPLC relay/input expander. |
| `bmi270` | Bosch BMI270 6-axis IMU sensor. | M5Stack Paper S3 motion sensors. |
| `ed047tc1` | ED047TC1 4.7 inch e-paper display driver using EPDIY. | M5Stack Paper S3 LVGL e-paper UI. |
| `esp_sr_wake_word` | Experimental ESP-SR wake word component. | CoreS3 SE ESP-SR voice assistant variant. |
| `i2s_audio` | Local override of ESPHome `i2s_audio` speaker support. | M5Stack Cardputer speaker `output_gain`. |
| `qmi8658` | QMI8658 6-axis IMU sensor. | Waveshare ESP32-S3 Matrix and Touch AMOLED demos. |
| `ys_irtm_uart` | YS-IRTM UART infrared transceiver. NEC TX/RX, HA IR/RF Proxy bridge. | Generic ESP32 demo. |

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
| Waveshare ESP32-P4 WIFI6 Touch LCD 10.1 | [devices/waveshare/esp32p4-wifi6-touch-lcd-10.1.yaml](devices/waveshare/esp32p4-wifi6-touch-lcd-10.1.yaml) | ESP32-P4 with ESP32-C6 hosted Wi-Fi, 800x1280 MIPI-DSI display, GT911 touch, ES8311/ES7210 audio, LVGL status UI, voice assistant, microphone level diagnostics. | None. |
| AMG8833 PoC | [devices/test/amg8833_poc.yaml](devices/test/amg8833_poc.yaml) | ESP32-S3, AMG8833 thermal camera — JPEG camera entity (64×64 false-color), avg/min/max/thermistor sensors. | `amg8833` |
| MCU90640 PoC | [devices/test/mcu90640_poc.yaml](devices/test/mcu90640_poc.yaml) | ESP32-C3, GY-MCU90640 thermal camera (MLX90640 via onboard STM32, UART 115200) — JPEG camera entity (64×48 false-color), avg/min/max/ambient sensors, centroid, presence/hot-spot. | `mcu90640` |
| WLED Bridge PoC | [devices/test/wled_bridge_poc.yaml](devices/test/wled_bridge_poc.yaml) | ESP32-S3, 16MB flash, 8MB PSRAM, WS2812 strip — WLED v16 JSON UI compatibility, fixed 220 mode slots, supported-effect mapping, multi-segment, presets/playlists, and nightlight. | `wled_bridge` |
| WLED Bridge Multi-Bus PoC | [devices/test/wled_bridge_multibus_poc.yaml](devices/test/wled_bridge_multibus_poc.yaml) | ESP32-S3, two ESPHome RMT LED strips exposed as one WLED virtual LED space with per-bus current limits. | `wled_bridge` |
| WLED Bridge Realtime PoC | [devices/test/wled_bridge_realtime_poc.yaml](devices/test/wled_bridge_realtime_poc.yaml) | ESP32-S3 realtime receiver coverage for WLED notifier, DDP, E1.31, and Art-Net with boot preset and brightness factor. | `wled_bridge` |
| WLED Bridge Audio PoC | [devices/test/wled_bridge_audio_poc.yaml](devices/test/wled_bridge_audio_poc.yaml) | ESP32-S3, I2S microphone, sound reactive effects — volume + FFT GEQ + beat detection + AGC. | `wled_bridge` |
| WLED Bridge Entities PoC | [devices/test/wled_bridge_entities_poc.yaml](devices/test/wled_bridge_entities_poc.yaml) | ESP32-S3, HA select/number entities — palette select, effect select, speed and intensity numbers for dashboard control. | `wled_bridge` |
| Custom Components Smoke | [devices/test/custom_components_smoke.yaml](devices/test/custom_components_smoke.yaml) | ESP32-S3 compile smoke coverage for IMU sensors, I2C expanders, keypad scanner, and YS-IRTM UART actions. | `74hc138_keypad`, `aw9523`, `bmi270`, `qmi8658`, `ys_irtm_uart` |
| YS-IRTM UART Bridge | [devices/generic/ys-irtm-demo.yaml](devices/generic/ys-irtm-demo.yaml) | NEC IR transmit/receive, HA IR/RF Proxy bridge service, raw hex send. | `ys_irtm_uart` |
| Guition ESP32-S3-4848S040 | TBD | Display, touch, relays. | TBD |
| Guition JC3636K518 | TBD | QSPI display, touch, audio, encoder. | TBD |
| Guition JC3636W518 | TBD | QSPI display, touch, audio. | TBD |
| Waveshare ESP32-S3-ETH-8DI-8RO | TBD | Ethernet, digital inputs, relay outputs, RS485. | TBD |
| Waveshare ESP32-S3-Relay-6CH | TBD | Relay outputs, RS485. | TBD |
| Wireless-Tag WT32-SC01 | TBD | Display, touch, RS485, audio. | TBD |

`secrets.yaml` files beside the demo YAMLs are local secret material and are not
device examples.
