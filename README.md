# ESPHome Components

Custom ESPHome components and device YAMLs for this workspace.

This README is intentionally short: it tracks the YAMLs that exist, which
non-official components are needed, and which external component sources each
device uses.

## Status Legend

| Symbol | Meaning |
| --- | --- |
| 🆕 | Planned, not implemented yet |
| 🔄 | Existing code, pending ESPHome 2026.x verification or rewrite |
| 🧪 | Compiles on current ESPHome, pending hardware verification |
| ✅ | Verified on hardware with current ESPHome |
| ⚠️ | Present in YAML, but known/pending migration issue |

## Device YAMLs

| Device | YAML | Status | Extra components used |
| --- | --- | --- | --- |
| M5Stack Paper S3 | [devices/m5stack/m5papers3.yaml](devices/m5stack/m5papers3.yaml) | 🧪 Compile OK; hardware verification pending | Our `bmi270`, `ed047tc1`; EPDIY library fork |
| M5Stack StamPLC | [devices/m5stack/m5stamplc.yaml](devices/m5stack/m5stamplc.yaml) | 🔄 Needs cleanup for ESPHome 2026.x | Our `aw9523`; ⚠️ `pi4ioe5v6408` is archived/pending replacement |
| M5Stack CoreS3 SE | [devices/m5stack/m5cores3se.yaml](devices/m5stack/m5cores3se.yaml) | ✅ Voice Assistant working | M5Stack official `axp2101`, `aw88298`, `aw9523b` |
| M5Stack CoreS3 SE ESP-SR variant | [devices/m5stack/m5cores3se-espsr.yaml](devices/m5stack/m5cores3se-espsr.yaml) | 🔄 Experimental/local validation pending | M5Stack official `axp2101`, `aw88298`, `aw9523b`; our `esp_sr_wake_word` |
| M5Stack Cardputer | [devices/m5stack/m5cardputer.yaml](devices/m5stack/m5cardputer.yaml) | 🧪 Keyboard/display verified; speaker direct/fork tone verified; Media Player/VA pending | ESPHome built-ins; our `74hc138_keypad`, `m5cardputer_i2s_audio`, `m5cardputer_i2s_speaker` |
| Waveshare ESP32-S3-Touch-AMOLED-2.41 | [devices/waveshare/esp32s3-touch-amoled-2.41.yaml](devices/waveshare/esp32s3-touch-amoled-2.41.yaml) | 🧪 Display/touch/IMU/RTC verified on hardware; QMI8658 SA0 high (0x6B); LVGL UI running | Our `qmi8658`; display/touch/RTC/GPIO expander via ESPHome built-ins (`qspi_dbi`, `ft63x6`, `pcf85063`, `pca9554`) |
| Waveshare ESP32-S3-Matrix | [devices/waveshare/esp32s3-matrix.yaml](devices/waveshare/esp32s3-matrix.yaml) | ✅ Verified on hardware | Our `qmi8658`; 8×8 WS2812B matrix via ESPHome built-ins |
| Waveshare ESP32-S3-ETH | [devices/waveshare/esp32s3-eth.yaml](devices/waveshare/esp32s3-eth.yaml) | 🧪 Compile OK; hardware verification pending | ESPHome built-ins only: `ethernet` (W5500), `esp32_camera` (OV2640/OV5640), `ethernet_info`, `bluetooth_proxy` |
| Waveshare ESP32-S3-GEEK | [devices/waveshare/esp32s3-geek.yaml](devices/waveshare/esp32s3-geek.yaml) | ✅ Verified on hardware | ESPHome built-ins only: `st7789v` display, `ledc` backlight, `bluetooth_proxy` |

`devices/m5stack/secrets.yaml` is local secret material and is not a device
configuration.

## Candidate Hardware

Reference notes live under [../ref/hardware/](../ref/hardware/). These devices
do not have baseline YAMLs in this repository yet.

| Vendor | Device | Reference | ESPHome notes |
| --- | --- | --- | --- |
| Guition | ESP32-S3-4848S040 | [hardware](../ref/hardware/guition/esp32-s3-4848s040.md) | ESPHome built-ins should cover display/touch/relays; relay GPIOs still need confirmation. |
| Guition | JC3636K518 | [hardware](../ref/hardware/guition/jc3636k518.md) | Built-ins cover QSPI display, touch, audio, encoder; note Quad PSRAM and dual-MCU audio path. |
| Guition | JC3636W518 | [hardware](../ref/hardware/guition/jc3636w518.md) | Built-ins cover QSPI display, touch, audio; note Octal PSRAM and V1/V2 mic differences. |
| Waveshare | ESP32-S3-ETH | [hardware](../ref/hardware/waveshare/esp32-s3-eth/esp32-s3-eth.md) | Built-in W5500 Ethernet; GPIO33-37 reserved by Octal PSRAM. |
| Waveshare | ESP32-S3-ETH-8DI-8RO | [hardware](../ref/hardware/waveshare/esp32-s3-eth-8di-8ro/esp32-s3-eth-8di-8ro.md) | Built-in W5500 Ethernet, `pca9554` for TCA9554 relay expander, RS485 via `uart`. |
| ~~Waveshare~~ | ~~ESP32-S3-GEEK~~ | ~~[hardware](../ref/hardware/waveshare/esp32-s3-geek/esp32-s3-geek.md)~~ | Graduated to Device YAMLs above. |
| Waveshare | ESP32-S3-Relay-6CH | [hardware](../ref/hardware/waveshare/esp32-s3-relay-6ch/esp32-s3-relay-6ch.md) | Built-in GPIO relays and RS485; several relay pins are strapping pins. |
| ~~Waveshare~~ | ~~ESP32-S3-Touch-AMOLED-2.41~~ | ~~[hardware](../ref/hardware/waveshare/esp32-s3-touch-amoled-2.41/esp32-s3-touch-amoled-2.41.md)~~ | Graduated to Device YAMLs above. |
| Waveshare | ESP32-S3-Touch-LCD-1.85C | [hardware](../ref/hardware/waveshare/esp32-s3-touch-lcd-1.85c/esp32-s3-touch-lcd-1.85c.md) | `qmi8658` available; display/touch/RTC/audio/GPIO expander should use ESPHome built-ins. |
| Wireless-Tag | WT32-SC01 / Plus | [hardware](../ref/hardware/wireless-tag/wt32-sc01/wt32-sc01.md) | Built-in display/touch/RS485/audio support; Plus has tight 2MB PSRAM for LVGL. |

## Components Not In ESPHome Official

### Active / Local Components

These live under [components/](components/) and are provided by this repository.

| Component | Type | Used by | Status | Notes |
| --- | --- | --- | :---: | --- |
| `aw9523` | I2C GPIO expander | StamPLC | 🧪 | Compiles on ESPHome 2026.4.5; hardware verification pending. Adds LED driver mode, INTENABLE disable, `imax_divider`, and `latch_inputs`. |
| `bmi270` | 6-axis IMU | Paper S3 | 🧪 | Native I2C rewrite landed on `new`; compiles with Paper S3 on ESPHome 2026.4.5. |
| `ed047tc1` | 4.7 inch e-paper display | Paper S3 | 🧪 | Supports `partial_update()` and ESPHome 2026.x EPDIY board init compatibility. Requires EPDIY fork below. |
| `esp_sr_wake_word` | ESP-SR wake word integration | CoreS3 SE ESP-SR variant | 🔄 | Local experimental component used only by `m5cores3se-espsr.yaml`. |
| `74hc138_keypad` | 3-to-8 decoder keyboard matrix | M5Stack Cardputer V1.1 | 🧪 | First-pass ESP-IDF component using M5 official IO-matrix scan mapping; compile/OTA OK and basic key input verified. Modifier keys remain unhandled in YAML. |
| `m5cardputer_i2s_audio` | Renamed fork of ESPHome `i2s_audio` speaker path | M5Stack Cardputer V1.1 | 🧪 | Verification fork avoids the upstream first-buffer `i2s_channel_preload_data()` path and writes initial audio with `i2s_channel_write()`. Direct `speaker::Speaker` tone is audible on hardware; Media Player/Voice Assistant integration remains pending. |
| `m5cardputer_i2s_speaker` | ESP-IDF new-driver I2S speaker test | M5Stack Cardputer V1.1 | 🧪 | Minimal direct `i2s_channel_write()` tone generator on I2S port 1, GPIO41/43/42. Verified audible and used as the reference implementation for the fork. |
| `m5cardputer_legacy_speaker` | ESP-IDF legacy I2S speaker test | M5Stack Cardputer V1.1 | 🧪 | Legacy-driver comparison component. Produces audible test tones, but cannot coexist with ESPHome's new I2S driver stack. |
| `pca9505` | I2C GPIO expander | None currently | 🔄 | Existing local component; pending ESPHome 2026.x verification/rewrite. |
| `qmi8658` | 6-axis IMU | Waveshare Matrix, Touch AMOLED 2.41, Touch LCD 1.85C | ✅ | Written from scratch following ESPHome BMI160 pattern. Verified on hardware with ESP32-S3-Matrix. |

### Planned Components

None currently.

### Replaced / Archived

These should not be used for new device YAMLs unless there is a deliberate
reason to revive them.

| Component | Replacement | Notes |
| --- | --- | --- |
| `axp2101` | M5Stack official `axp2101` | Used by CoreS3 SE from `github://m5stack/esphome-yaml/components`. |
| `aw88298` | M5Stack official `aw88298` | Used by CoreS3 SE from `github://m5stack/esphome-yaml/components`. |
| `aw9523b` | M5Stack official `aw9523b` | Used by CoreS3 SE from `github://m5stack/esphome-yaml/components`. |
| `lm75` | ESPHome official `lm75b` | Built-in replacement. |
| `pi4ioe5v6408` | Pending migration | Archived because ESPHome 2026.x pin schema/API changed; still appears in StamPLC YAML and needs cleanup. |
| `rx8130ce` | ESPHome official `rx8130` | Built-in replacement. |

See [../DECISIONS.md](../DECISIONS.md) for the CoreS3 SE decision to use
M5Stack official `axp2101`, `aw88298`, and `aw9523b`.

## External Component Sources

| Source | Components | Where used |
| --- | --- | --- |
| `github://patrick3399/esphome_components` | Local repository components via [packages/common.yaml](packages/common.yaml) | Paper S3, StamPLC, and any YAML including the common package |
| `github://patrick3399/esphome_components` | `esp_sr_wake_word` | CoreS3 SE ESP-SR variant |
| `github://m5stack/esphome-yaml/components` | `axp2101`, `aw88298`, `aw9523b` | CoreS3 SE YAMLs |
| `https://github.com/patrick3399/epdiy` | EPDIY display library fork | Paper S3 `ed047tc1` |

EPDIY-side dependency: [`patrick3399/epdiy@fix/platformio-srcfilter`](https://github.com/patrick3399/epdiy/tree/fix/platformio-srcfilter)
contains the PlatformIO `srcFilter` and weak C fallback fixes needed for the
current `ed047tc1` build path.
