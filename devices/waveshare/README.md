# Waveshare Devices

Device YAMLs for Waveshare ESP32 hardware. All use ESP-IDF framework and the shared
[common package](../../packages/common.yaml) for WiFi, OTA, API, and logger.

## Secrets

Place a `secrets.yaml` beside the device YAML (gitignored):

```yaml
wifi_ssid: "MyNetwork"
wifi_password: "password"
api_key: "abcdef..."
ota_password: "..."
```

## Devices

### ESP32-S3 Matrix

**File:** [esp32s3-matrix.yaml](esp32s3-matrix.yaml)

ESP32-S3 with onboard 8×8 WS2812B LED matrix. QMI8658 IMU sensors and matrix
brightness/animation demo.

Custom components: `qmi8658`

---

### ESP32-S3 Touch AMOLED 2.41

**File:** [esp32s3-touch-amoled-2.41.yaml](esp32s3-touch-amoled-2.41.yaml)

450×600 AMOLED display, capacitive touch, RTC (PCF85063), QMI8658 IMU, LVGL UI,
Bluetooth proxy, and diagnostics.

Custom components: `qmi8658`

---

### ESP32-S3 Touch LCD 1.85C

**File:** [esp32s3-touch-lcd-1.85c.yaml](esp32s3-touch-lcd-1.85c.yaml)

QSPI LCD 240×280, capacitive touch, RTC, I2S microphone and speaker for voice
assistant with wake word selection, Bluetooth proxy, battery and audio diagnostics.

Custom components: none

---

### ESP32-S3 ETH

**File:** [esp32s3-eth.yaml](esp32s3-eth.yaml)

W5500 SPI Ethernet, OV2640 camera, Ethernet info sensors, Bluetooth proxy, boot
button, diagnostics.

Custom components: none

---

### ESP32-S3 GEEK

**File:** [esp32s3-geek.yaml](esp32s3-geek.yaml)

ESP32-S3 with ST7789 1.14" display, backlight control, WiFi diagnostics, Bluetooth
proxy.

Custom components: none

---

### ESP32-P4 WiFi6 Touch LCD 10.1

**File:** [esp32p4-wifi6-touch-lcd-10.1.yaml](esp32p4-wifi6-touch-lcd-10.1.yaml)

ESP32-P4 SoC with ESP32-C6 hosted Wi-Fi 6. 800×1280 MIPI-DSI display, GT911 touch,
ES8311 speaker + ES7210 microphone, LVGL status UI, voice assistant, microphone
level diagnostics.

Custom components: none
