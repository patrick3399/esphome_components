# ESPHome Components

Custom ESPHome components and device configurations.

## Status Legend

| Symbol | Meaning |
|--------|---------|
| ✅ | Verified working with current ESPHome |
| ⚠️ | Needs verification with ESPHome 2026.x |
| 🚧 | In development (unmerged branch) |
| ❌ | Archived — known broken or replaced |
| 📋 | Reference only — no config yet |

---

## Components

### Active (`components/`)

| Component | Type | ESPHome 2026.x | Notes |
|-----------|------|:--------------:|-------|
| `aw9523` | I2C GPIO expander (16-pin) | ⚠️ | Used on StamPLC |
| `bmi270` | IMU (accel + gyro) | ⚠️ 🚧 | Native I2C rewrite pending (`fix/bmi270-native-i2c`) |
| `ed047tc1` | 4.7" e-paper display | ⚠️ 🚧 | `partial_update()` pending (`feat/ed047tc1-partial-update`) |
| `pca9505` | I2C GPIO expander (40-pin) | ⚠️ | Used on StamPLC |

### Archived (`archived/`)

| Component | Type | Reason |
|-----------|------|--------|
| `lm75` | I2C temperature sensor | Replaced by built-in ESPHome `lm75` |
| `pi4ioe5v6408` | I2C GPIO expander (8-pin) | `pins.gpio_base_schema` API broken in ESPHome 2026.x |
| `rx8130ce` | I2C RTC | Replaced by built-in ESPHome `rx8130ce` |

---

## Device × Component Matrix

| Vendor | Device | Config | `aw9523` | `bmi270` | `ed047tc1` | `pca9505` | ESPHome |
|--------|--------|--------|:--------:|:--------:|:----------:|:---------:|---------|
| **M5Stack** | StamPLC | `devices/m5stack/m5stamplc.yaml` | ⚠️ | — | — | ⚠️ | ⚠️ needs verify |
| **M5Stack** | Paper S3 | `devices/m5stack/m5papers3.yaml` | — | ⚠️ 🚧 | ⚠️ 🚧 | — | ⚠️ needs verify |
| **M5Stack** | CoreS3 SE | 🏠 HA server | — | ⚠️ 🚧 | — | — | 🏠 in use |
| **M5Stack** | Cardputer V1.1 | — | — | — | — | — | 📋 no config |
| **Guition** | ESP32-S3-4848S040 | — | — | — | — | — | 📋 no config |
| **Guition** | JC3636K518 | — | — | — | — | — | 📋 no config |
| **Guition** | JC3636W518 | — | — | — | — | — | 📋 no config |
| **Waveshare** | ESP32-S3-ETH | — | — | — | — | — | 📋 no config |
| **Waveshare** | ESP32-S3-ETH-8DI-8RO | — | — | — | — | — | 📋 no config |
| **Waveshare** | ESP32-S3-GEEK | — | — | — | — | — | 📋 no config |
| **Waveshare** | ESP32-S3-Matrix | — | — | — | — | — | 📋 no config |
| **Waveshare** | ESP32-S3-Relay-6CH | — | — | — | — | — | 📋 no config |
| **Waveshare** | ESP32-S3-Touch-AMOLED-2.41 | — | — | — | — | — | 📋 no config |
| **Waveshare** | ESP32-S3-Touch-LCD-1.85C | — | — | — | — | — | 📋 no config |
| **Wireless-Tag** | WT32-SC01 / Plus | — | — | — | — | — | 📋 no config |

---

## Open Branches

| Branch | Description |
|--------|-------------|
| `feat/ed047tc1-partial-update` | Adds `partial_update()` to reduce e-ink panel wear |
| `fix/bmi270-native-i2c` | Rewrites bmi270 using ESPHome native I2C, removes Bosch SDK dependency |
