# i2s_audio

Local override of ESPHome's built-in `i2s_audio` component. Adds `volume_multiplier`
support to the speaker platform, which upstream did not expose at the time this
override was written.

## When to Use

Use this component only on devices that need `volume_multiplier` on the speaker (e.g.
M5Stack Cardputer with its NS4168 amplifier). For devices that don't need it,
remove this from `external_components` and let ESPHome load the upstream version.

## Configuration

Identical to upstream `i2s_audio` — see the
[ESPHome i2s_audio docs](https://esphome.io/components/i2s_audio.html). The
additional `volume_multiplier` key is available on the `speaker` platform:

```yaml
speaker:
  - platform: i2s_audio
    i2s_audio_id: i2s_bus
    dac_type: external
    i2s_dout_pin: GPIO6
    volume_multiplier: 3.0   # added by this override
```

## Notes

- This override shadows the upstream component by name. When ESPHome upstream adds
  equivalent volume multiplication, remove this directory and its
  `external_components` entries.
- Port allocation logic (`_assign_ports`) prioritises PDM/ADC microphones to I2S
  port 0, which is the only port that supports PDM on most ESP32 variants.

## Demo

[devices/m5stack/m5cardputer.yaml](../../devices/m5stack/m5cardputer.yaml)
