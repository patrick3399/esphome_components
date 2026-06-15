# ESP-SR Wake Word

ESPHome external component for Espressif ESP-SR WakeNet wake-word
detection on ESP32-S3 / ESP-IDF.

## Configuration

```yaml
esp_sr_wake_word:
  id: wake_engine
  microphone: i2s_mic
  stop_after_detection: true
  models:
    - id: esp_sr_nihaoxiaozhi
      model: wn9_nihaoxiaozhi_tts
      wake_word: "Ni Hao Xiao Zhi"
      internal: false
  on_wake_word_detected:
    - voice_assistant.start:
        wake_word: !lambda return wake_word;
```

The `models` list mirrors ESPHome's `micro_wake_word` shape. A model can be
referenced directly by name or expanded to set an ID and display wake word:

```yaml
models:
  - wn9_nihaoxiaozhi_tts
  - id: esp_sr_hiesp_small
    model: wn9s_hiesp
    wake_word: "Hi ESP"
    internal: false
```

All configured models are included in `srmodels.bin`. At runtime the component
loads one enabled ESP-SR WakeNet model at a time. This is intentionally different
from `micro_wake_word`: ESP-SR WakeNet instances are large enough that loading
several at once can exhaust internal RAM on ESP32-S3.

The first configured model is enabled by default. Calling `enable_model` selects
that model and disables the others. If detection is already running, the
component stops the inference task, unloads the current model, enables the new
model, and starts detection again. `disable_model` disables the selected model;
detection cannot start while no model is enabled.

### Actions And Conditions

```yaml
button:
  - platform: template
    name: "Start Wake Word"
    on_press:
      - esp_sr_wake_word.start: wake_engine

  - platform: template
    name: "Stop Wake Word"
    on_press:
      - esp_sr_wake_word.stop: wake_engine

  - platform: template
    name: "Enable Hi ESP"
    on_press:
      - esp_sr_wake_word.enable_model: esp_sr_hiesp_small

  - platform: template
    name: "Disable Hi ESP"
    on_press:
      - esp_sr_wake_word.disable_model: esp_sr_hiesp_small

binary_sensor:
  - platform: template
    name: "Wake Word Running"
    lambda: return id(wake_engine).is_running();
    # Automation conditions are also available:
    # - esp_sr_wake_word.is_running: wake_engine
    # - esp_sr_wake_word.model_is_enabled: esp_sr_hiesp_small
```

```yaml
select:
  - platform: template
    name: "Wake Word Model"
    optimistic: true
    options:
      - "Hi Lily"
      - "Hi ESP"
    on_value:
      - if:
          condition:
            lambda: return x == "Hi Lily";
          then:
            - esp_sr_wake_word.enable_model: esp_sr_hilili
      - if:
          condition:
            lambda: return x == "Hi ESP";
          then:
            - esp_sr_wake_word.enable_model: esp_sr_hiesp
```

## Partition

ESP-SR model blobs are stored in a data partition named `model`. ESPHome 2026.4
can append this partition to the generated ESP32 partition table:

```yaml
esp32:
  partitions:
    - name: model
      type: data
      subtype: undefined
      size: 0x600000
```

The partition name must be exactly `model`; ESP-SR looks it up at runtime with
`esp_srmodel_init("model")`.

## Model Image

The `espressif/esp-sr` IDF component adds an `srmodels_bin` CMake build target
when it finds the `model` partition. ESPHome's `compile` action runs this target
automatically, but only if the IDF configure step has already completed. If
`srmodels.bin` is missing after a compile, generate it manually with `movemodel.py`
(no cmake invocation required):

```powershell
# Paths for ws-185c — adjust <vendor>/<device> for other targets
$comp = "esphome_components\.dev\devices\<vendor>\.esphome\build\<device>\managed_components\espressif__esp-sr"
$build = "esphome_components\.dev\devices\<vendor>\.esphome\build\<device>\.pioenvs\<device>"
$sdk  = "esphome_components\.dev\devices\<vendor>\.esphome\build\<device>\sdkconfig.<device>"

.\venv\Scripts\python.exe "$comp\model\movemodel.py" -d1 $sdk -d2 $comp -d3 $build
# Output: $build\srmodels\srmodels.bin
```

The model partition offset is in `flasher_args.json` (same directory as `firmware.bin`):

```powershell
Get-Content "$build\flasher_args.json" | ConvertFrom-Json | Select-Object -ExpandProperty flash_files
# "model": { "offset": "0xa00000", "file": "srmodels/srmodels.bin" }
```

Flash with esptool — device can be running normally (no bootloader mode needed):

```powershell
.\esptool-windows-amd64\esptool.exe --chip esp32s3 --port COM<N> --baud 921600 `
    write-flash <offset> "$build\srmodels\srmodels.bin"
```

**OTA updates replace only the application image; they do not populate or update
the `model` data partition.** Re-flash `srmodels.bin` only when the esp-sr component
version changes (i.e. different models are included) — not on every firmware update.
