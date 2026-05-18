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

`devices/m5stack/secrets.yaml` is local secret material and is not a device
configuration.

## Components Not In ESPHome Official

### Active / Local Components

These live under [components/](components/) and are provided by this repository.

| Component | Type | Used by | Status | Notes |
| --- | --- | --- | :---: | --- |
| `aw9523` | I2C GPIO expander | StamPLC | 🧪 | Compiles on ESPHome 2026.4.5; hardware verification pending. Adds LED driver mode, INTENABLE disable, `imax_divider`, and `latch_inputs`. |
| `bmi270` | 6-axis IMU | Paper S3 | 🧪 | Native I2C rewrite landed on `new`; compiles with Paper S3 on ESPHome 2026.4.5. |
| `ed047tc1` | 4.7 inch e-paper display | Paper S3 | 🧪 | Supports `partial_update()` and ESPHome 2026.x EPDIY board init compatibility. Requires EPDIY fork below. |
| `esp_sr_wake_word` | ESP-SR wake word integration | CoreS3 SE ESP-SR variant | 🔄 | Local experimental component used only by `m5cores3se-espsr.yaml`. |
| `pca9505` | I2C GPIO expander | None currently | 🔄 | Existing local component; pending ESPHome 2026.x verification/rewrite. |

### Planned Components

| Component | Type | Target devices | Status | Notes |
| --- | --- | --- | :---: | --- |
| `qmi8658` | 6-axis IMU | Waveshare Matrix, Touch AMOLED 2.41, Touch LCD 1.85C | 🆕 | Prefer vendoring/adapting active community work such as `dala318/esphome-qmi8658` before writing from scratch. |
| `74hc138_keypad` | 3-to-8 decoder keyboard matrix | M5Stack Cardputer V1.1 | 🆕 | Needed because ESPHome `matrix_keypad` cannot model this decoder topology. |

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
