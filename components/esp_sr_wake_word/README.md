# ESP-SR Wake Word

Experimental ESPHome external component for Espressif ESP-SR WakeNet wake-word
detection on ESP32-S3 / ESP-IDF.

## Configuration

```yaml
esp_sr_wake_word:
  microphone: i2s_mic
  stop_after_detection: true
  models:
    - id: esp_sr_nihaoxiaozhi
      model: wn9_nihaoxiaozhi_tts
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
when it finds the `model` partition. The target writes:

```text
<build>/srmodels/srmodels.bin
```

For the local staging helper, the CoreS3 SE ESP-SR model path is:

```text
esphome_components/.dev/devices/m5stack/.esphome/build/m5cores3se-espsr/.pio/build/m5cores3se-espsr/srmodels/srmodels.bin
```

If a normal ESPHome compile does not leave that file behind, build the target
from the generated CMake build tree:

```powershell
C:\Users\Yuni\.platformio\packages\tool-cmake\bin\cmake.exe --build esphome_components\.dev\devices\m5stack\.esphome\build\m5cores3se-espsr\.pio\build\m5cores3se-espsr --target srmodels_bin
```

For a first serial flash, flash the normal factory firmware and the model image.
Use the generated partition table to determine the current `model` offset, or
let PlatformIO/ESP-IDF flash by partition name through its `flash` target.

OTA updates replace only the application image; they do not populate or update
the `model` data partition.
