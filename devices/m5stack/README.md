# M5Stack Devices

Device YAMLs for M5Stack hardware. All use ESP-IDF framework and the shared
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

### M5Stack Paper S3

**File:** [m5papers3.yaml](m5papers3.yaml)

ESP32-S3, 16 MB flash, 8 MB PSRAM. ED047TC1 4.7" e-paper display driven by EPDIY.
LVGL UI with clock, status, BMI270 IMU, and debug pages. Touch navigation, battery
and charging status via AXP2101 PMU. RTTTL buzzer service.

Custom components: `ed047tc1`, `bmi270`

Compile:
```powershell
.\dev-compile.ps1 -Device devices/m5stack/m5papers3.yaml
```

---

### M5Stack StamPLC

**File:** [m5stamplc.yaml](m5stamplc.yaml)

ESP32-S3 PLC board. 8 relay outputs and 8 digital inputs via AW9523 I2C expander.
Board temperature and current sensors. ST7789 status display, RTC sync, RTTTL service.

Custom components: `aw9523`

---

### M5Stack CoreS3 SE

**File:** [m5cores3se.yaml](m5cores3se.yaml)

ESP32-S3, 16 MB flash, 8 MB PSRAM. Voice assistant with on-device wake word via
ESPHome `micro_wake_word`. Speaker media player, microphone mute, AXP2101 PMU/battery
sensors, LCD/touch LVGL UI.

Uses M5Stack official components (`github://m5stack/esphome-yaml`) for AXP2101,
AW9523B, and AW88298.

Local custom components: `i2s_audio`

---

### M5Stack CoreS3 SE (ESP-SR variant)

**File:** [m5cores3se-espsr.yaml](m5cores3se-espsr.yaml)

Same hardware as CoreS3 SE but uses the `esp_sr_wake_word` component
for Espressif ESP-SR WakeNet models instead of `micro_wake_word`. Requires flashing
an `srmodels.bin` model partition separately — see the
[esp_sr_wake_word README](../../components/esp_sr_wake_word/README.md).

Local custom components: `esp_sr_wake_word`, `i2s_audio`

Also uses M5Stack official components for AXP2101, AW9523B, and AW88298.

---

### M5Stack Cardputer

**File:** [m5cardputer.yaml](m5cardputer.yaml)

ESP32-S3FN8, **no PSRAM**. 74HC138 keyboard matrix (full QWERTY). Keyboard-driven
HA conversation flow, push-to-talk voice assistant. ST7789 1.14" display (status +
debug pages). RTTTL buzzer, IR transmitter, speaker media player (RTTTL only — no
`speaker_media_player` without PSRAM).

Custom components: `74hc138_keypad`, `i2s_audio`

> GPIO 33–37 are the LCD bus — do not use as general GPIO.
