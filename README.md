# ESPHome Components

Custom ESPHome components and device configurations.

## Status Legend

| Symbol | Meaning |
|--------|---------|
| 🆕 | 待開發 — Planned, no implementation yet |
| 🚧 | In progress — active branch (see Open Branches) |
| 🔄 | 待更新/重構 — Existing code, pending verification or rewrite for ESPHome 2026.x |
| 🧪 | Compiles cleanly on current ESPHome — awaiting hardware verification |
| ✅ | Verified working on real hardware with current ESPHome |
| 🏠 | In use on HA server, config not in repo |
| 📋 | Reference only — no config yet |
| ❌ | Archived — known broken or replaced |

---

## 待開發 / Planned Components

Top-priority custom components to add for new target devices. Driven by the
[Coverage Gaps Summary](#coverage-gaps-summary) below.

| Component | Type | Devices Blocked | Priority |
|-----------|------|-----------------|----------|
| `qmi8658` | 6-axis IMU | Waveshare Matrix · AMOLED-2.41 · LCD-1.85C | Medium — community fork `dala318/esphome-qmi8658` available to vendor |
| `74hc138_keypad` | 3-to-8 decoder keyboard matrix | M5Stack Cardputer V1.1 | Low — `matrix_keypad` cannot model decoder topology |

> **axp2101 / aw88298 / aw9523b (CoreS3 SE):** Deferred — using
> `github://m5stack/esphome-yaml/components` instead. See [DECISIONS.md](../DECISIONS.md).

---

## 待更新/重構 / Legacy Components (`components/`)

All existing components were last validated on pre-2026.x ESPHome and are
pending verification or rewrite. Do not assume any of these compile cleanly
on the current ESPHome version without re-testing.

| Component | Type | Status | Notes |
|-----------|------|:------:|-------|
| `aw9523` | I2C GPIO expander (16-pin) | 🧪 | Used on StamPLC. Compiles on ESPHome 2026.4.5. Adds LED driver mode, INTENABLE disable, `imax_divider`, `latch_inputs`. HW verification pending. |
| `bmi270` | IMU (accel + gyro) | 🧪 | Native I2C rewrite landed in `new` (`b0366b7`, cherry-picked from `fix/bmi270-native-i2c`); compiles on m5papers3 + ESPHome 2026.4.5 |
| `ed047tc1` | 4.7" e-paper display | 🧪 | `partial_update()` (`8ca1c24`) and EPDIY 2026.x board-init compat (`ecdead1`) both in `new`; compiles on m5papers3. Requires [`patrick3399/epdiy@fix/platformio-srcfilter`](https://github.com/patrick3399/epdiy/tree/fix/platformio-srcfilter) |
| `pca9505` | I2C GPIO expander (40-pin) | 🔄 | Used on StamPLC |

### Archived (`archived/`)

| Component | Type | Reason |
|-----------|------|--------|
| `axp2101` | PMU / power rail control | Replaced by `github://m5stack/esphome-yaml/components` — vendor-validated, crash-free. Our version had identical C++ but incompatible aw9523b interaction. See [DECISIONS.md](../DECISIONS.md). |
| `aw88298` | I2C audio amplifier (1W) | Replaced by `github://m5stack/esphome-yaml/components` — our version used `return -1.0f` magic number for `setup_priority` instead of `setup_priority::DATA`. |
| `aw9523b` | I2C GPIO expander (16-pin, CoreS3 SE) | Replaced by `github://m5stack/esphome-yaml/components` — pin key schema (`aw9523b:` vs `aw9523b_id:`) and `mode_mask_` init difference caused OTA rollback crashes. |
| `lm75` | I2C temperature sensor | Replaced by built-in ESPHome `lm75b` |
| `pi4ioe5v6408` | I2C GPIO expander (8-pin) | `pins.gpio_base_schema` API broken in ESPHome 2026.x |
| `rx8130ce` | I2C RTC | Replaced by built-in ESPHome `rx8130` |

---

## Device × Component Matrix

Custom component columns track which devices depend on components in **this repo**.
ESPHome column reflects overall config readiness (not just custom components).

| Vendor | Device | Config | `aw9523` | `bmi270` | `ed047tc1` | `pca9505` | ESPHome |
|--------|--------|--------|:--------:|:--------:|:----------:|:---------:|---------|
| **M5Stack** | StamPLC | `devices/m5stack/m5stamplc.yaml` | 🧪 | — | — | 🔄 | 🔄 待更新 · pi4ioe5v6408 still broken |
| **M5Stack** | Paper S3 | `devices/m5stack/m5papers3.yaml` | — | 🧪 | 🧪 | — | 🧪 compile OK · HW pending |
| **M5Stack** | CoreS3 SE | `devices/m5stack/m5cores3se.yaml` | — | — | — | — | ✅ VA working · M5Stack official axp2101/aw88298/aw9523b |
| **M5Stack** | Cardputer V1.1 | — | — | — | — | — | 📋 · 🆕 74HC138 keyboard gap |
| **Guition** | ESP32-S3-4848S040 | — | — | — | — | — | 📋 |
| **Guition** | JC3636K518 | — | — | — | — | — | 📋 |
| **Guition** | JC3636W518 | — | — | — | — | — | 📋 |
| **Waveshare** | ESP32-S3-ETH | — | — | — | — | — | 📋 |
| **Waveshare** | ESP32-S3-ETH-8DI-8RO | — | — | — | — | — | 📋 |
| **Waveshare** | ESP32-S3-GEEK | — | — | — | — | — | 📋 |
| **Waveshare** | ESP32-S3-Matrix | — | — | — | — | — | 📋 · 🆕 QMI8658 gap |
| **Waveshare** | ESP32-S3-Relay-6CH | — | — | — | — | — | 📋 |
| **Waveshare** | ESP32-S3-Touch-AMOLED-2.41 | — | — | — | — | — | 📋 · 🆕 QMI8658 gap |
| **Waveshare** | ESP32-S3-Touch-LCD-1.85C | — | — | — | — | — | 📋 · 🆕 QMI8658 gap |
| **Wireless-Tag** | WT32-SC01 / Plus | — | — | — | — | — | 📋 |

---

## ESPHome Official Coverage by Device

Full breakdown of which ICs are covered by ESPHome built-ins vs. requiring custom components.

### M5Stack StamPLC

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| ST7789V2 display (SPI) | ✅ `ili9xxx` | `model: ST7789V` |
| AW9523 GPIO expander | ❌ → our `aw9523` | 16-pin I2C expander |
| PCA9505 GPIO expander | ❌ → our `pca9505` | 40-pin I2C expander |
| LM75 temperature | ✅ `lm75b` | Register-compatible with LM75 |
| INA226 power monitor | ✅ `ina226` | |
| RX8130CE RTC | ✅ `rx8130` | Built-in since ESPHome 2025.x |
| RS485 / Modbus | ✅ `uart` + `modbus_controller` | |

### M5Stack Paper S3

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| ED047TC1 e-ink (960×540) | ❌ → our `ed047tc1` | Requires EPDIY library |
| GT911 touch | ✅ `gt911` | |
| BMI270 IMU | ❌ → our `bmi270` | 6-axis IMU |
| BM8563 RTC | ✅ `bm8563` | Added in ESPHome 2025.12.0 |
| LGS4056H charger | ⬜ no driver | Battery charger; monitor via ADC |

### M5Stack CoreS3 SE

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| ILI9342C display (SPI) | ✅ `ili9xxx` | `model: ILI9342` |
| FT6336U touch | ✅ `ft63x6` | |
| ES7210 mic ADC | ✅ `es7210` | `audio_adc` platform |
| AW88298 amplifier | ✅ M5Stack official | `github://m5stack/esphome-yaml/components` — see [DECISIONS.md](../DECISIONS.md) |
| AXP2101 PMU | ✅ M5Stack official | `github://m5stack/esphome-yaml/components` — see [DECISIONS.md](../DECISIONS.md) |
| BM8563 RTC | ✅ `bm8563` | Added in ESPHome 2025.12.0 |

### M5Stack Cardputer V1.1

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| ST7789V2 display (SPI) | ✅ `ili9xxx` | |
| 74HC138 keyboard matrix | 🆕 `74hc138_keypad` | `matrix_keypad` cannot model 3-to-8 decoder topology; planned for this repo |
| SPM1423 mic (PDM) | ✅ `i2s_audio` | `pdm: true` |
| NS4168 speaker (I2S) | ✅ `i2s_audio` | `dac_type: external` |
| IR TX | ✅ `remote_transmitter` | |

### Guition ESP32-S3-4848S040

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| ST7701S RGB display | ✅ `st7701s` | RGB 16-bit parallel |
| GT911 touch | ✅ `gt911` | |
| 4ch relay (GPIO) | ✅ `gpio` switch | |

### Guition JC3636K518 / JC3636W518

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| ST77916 QSPI display | ✅ `qspi_dbi` | K518=Quad PSRAM · W518=Octal PSRAM |
| CST816S / CST816T touch | ✅ `cst816` | |
| PDM mic | ✅ `i2s_audio` | `pdm: true` (K518) / `pdm: false` (W518 V1) |
| PCM5100A DAC | ✅ `i2s_audio` | Passive I2S DAC, no I2C control needed |
| Rotary encoder | ✅ `rotary_encoder` | |

### Waveshare ESP32-S3-ETH

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| W5500 Ethernet | ✅ `ethernet` | `type: W5500`; GPIO33–37 unavailable (Octal PSRAM) |

### Waveshare ESP32-S3-ETH-8DI-8RO

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| W5500 Ethernet | ✅ `ethernet` | |
| TCA9554 GPIO expander (relays) | ✅ `pca9554` | TCA9554 is register-identical to PCA9554 |
| RS485 | ✅ `uart` | |

### Waveshare ESP32-S3-GEEK

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| ST7789 display (SPI) | ✅ `ili9xxx` | |

### Waveshare ESP32-S3-Matrix

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| WS2812B 8×8 matrix | ✅ `esp32_rmt_led_strip` | |
| QMI8658 IMU | 🆕 `qmi8658` | Planned; community fork `dala318/esphome-qmi8658` to vendor |

### Waveshare ESP32-S3-Relay-6CH

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| 6ch relay (GPIO) | ✅ `gpio` switch | GPIO1/2/45/46 → `ignore_strapping_warning: true` |
| RS485 | ✅ `uart` | |

### Waveshare ESP32-S3-Touch-AMOLED-2.41

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| RM690B0 AMOLED display (QSPI) | ✅ `qspi_amoled` | Being superseded by `mipi_spi` in future ESPHome |
| FT6336 touch | ✅ `ft63x6` | |
| TCA9554 GPIO expander | ✅ `pca9554` | Controls AMOLED_EN, TP_INT, IMU_INT — required for display init |
| QMI8658 IMU | 🆕 `qmi8658` | Planned; community fork `dala318/esphome-qmi8658` to vendor |
| PCF85063 RTC | ✅ `pcf85063` | |

### Waveshare ESP32-S3-Touch-LCD-1.85C

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| ST77916 QSPI display | ✅ `qspi_dbi` | |
| CST816S touch | ✅ `cst816` | |
| TCA9554 GPIO expander | ✅ `pca9554` | Controls TP_RST, LCD_RST, SD_D3, RTC_INT |
| PCF85063 RTC | ✅ `pcf85063` | |
| QMI8658A IMU | 🆕 `qmi8658` | Planned; community fork `dala318/esphome-qmi8658` to vendor |
| PCM5101A + NS8002 (V1 audio) | ✅ `i2s_audio` | Passive DAC; no I2C control |
| ES8311 codec (V2) | ✅ `es8311` | `audio_dac` platform |
| ES7210 mic ADC (V2) | ✅ `es7210` | `audio_adc` platform |

### Wireless-Tag WT32-SC01 / Plus

| IC / Peripheral | Platform | Notes |
|-----------------|----------|-------|
| ST7796 display — SC01 (SPI) | ✅ `ili9xxx` | `model: ST7796` |
| ST7796 display — Plus (8-bit parallel) | ✅ `ili9xxx` | `data_width: 8` |
| FT6336U touch | ✅ `ft63x6` | |
| RS485 — Plus | ✅ `uart` | |
| I2S speaker — Plus | ✅ `i2s_audio` | |

---

## Coverage Gaps Summary

ICs with no official ESPHome support across all candidate devices — these
map directly to the [Planned Components](#待開發--planned-components) table above.

| IC | Function | Devices | Action |
|----|----------|---------|--------|
| **AXP2101** | PMU / power rail control | CoreS3 SE | ✅ Resolved — using M5Stack official component |
| **AW88298** | I2C audio amplifier (1W) | CoreS3 SE | ✅ Resolved — using M5Stack official component |
| **QMI8658 / QMI8658A** | 6-axis IMU | ESP32-S3-Matrix, AMOLED-2.41, LCD-1.85C | 🆕 Vendor community component `dala318/esphome-qmi8658` |
| **74HC138 keyboard** | 3-to-8 decoder keyboard matrix | Cardputer V1.1 | 🆕 Custom scanning logic — `matrix_keypad` cannot model decoder topology |

---

## Open Branches

_None active — all in-flight work landed on `new` (2026-05-16):_

| Former branch | Landed as | Status |
|---------------|-----------|--------|
| `feat/ed047tc1-partial-update` | `8ca1c24` on `new` | merged via reorg; branch can be deleted |
| `fix/bmi270-native-i2c` | `b0366b7` on `new` (cherry-pick of `f02e8d9`) | merged; branch can be deleted |

EPDIY-side dependency: [`patrick3399/epdiy@fix/platformio-srcfilter`](https://github.com/patrick3399/epdiy/tree/fix/platformio-srcfilter)
— two commits (`6c3f9f7` lut.S srcFilter exclude, `e8a2e00` weak C fallback) that together let
EPDIY 2.0.0 link cleanly when consumed via PlatformIO's `libraries:` directive (as ESPHome
external components do). PR candidate to send upstream to `vroland/epdiy` later.
