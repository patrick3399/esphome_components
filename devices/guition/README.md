# Guition Devices

Device YAMLs for Guition ESP32 hardware. All use ESP-IDF framework and the shared
[common packages](../../packages/) for WiFi, OTA, API, and diagnostics.

## Secrets

Place a `secrets.yaml` beside the device YAML (gitignored):

```yaml
wifi_ssid: "MyNetwork"
wifi_password: "password"
api_key: "abcdef..."
ota_password: "..."
```

## Devices

### ESP32-S3-4848S040

**File:** [esp32-s3-4848s040.yaml](esp32-s3-4848s040.yaml)

4" 480×480 RGB LCD (ST7701S), GT911 capacitive touch, 8 MB Octal PSRAM, 4-ch relay
(GPIO unconfirmed — see `ref/guition/esp32-s3-4848s040.md`), LVGL UI.

ST7701S init sequence is from HA community post; includes `[0xCD, 0x00]` MDT batch
colour-correction. If colours look shifted, verify the sequence against the panel batch.

Custom components: none

---

### JC3636K518

**File:** [jc3636k518.yaml](jc3636k518.yaml)

1.8" round 360×360 QSPI display (ST77916), CST816S capacitive touch, rotary encoder,
I2S MEMS microphone, 8 MB Quad PSRAM.

> **Dual-MCU note:** GPIO16/17/18/21 are shared between the QSPI display bus (D1–D3,
> RST) and the on-board PCM5100A I2S DAC lines (WS, DOUT, BCLK, MCLK). ESPHome
> cannot drive both peripherals simultaneously on the same pins — the speaker/DAC
> section is omitted. Microphone (GPIO42/45/46) works independently.

Custom components: none

---

### JC3636W518

**File:** [jc3636w518.yaml](jc3636w518.yaml)

1.8" round 360×360 QSPI display (ST77916), CST816S capacitive touch, I2S MEMS
microphone, PCM5100A DAC (3.5 mm headphone output), TF card (1-bit MMC), wireless
charging (Qi), 8 MB Octal PSRAM.

> **V1 vs V2 microphone:** V1 uses standard I2S (`pdm: false`); V2 uses PDM
> (`pdm: true`). The YAML defaults to V1. If there is no audio, flip to `pdm: true`.

Custom components: none
