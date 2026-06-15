# i2s_audio

Local override of ESPHome's built-in `i2s_audio` component. Adds `volume_multiplier`
support to the speaker platform, which upstream did not expose at the time this
override was written.

## When to Use

Use this component only on devices that need `volume_multiplier` on the speaker (e.g.
M5Stack Cardputer with its NS4168 amplifier). For devices that don't need it,
remove this from `external_components` and let ESPHome load the upstream version.

## Complete Configuration

The example below shows every non-inherited YAML control exposed by this local
override. Entity/component base options such as `id`, `on_*`, and speaker or
microphone framework options are omitted unless they select I2S behavior.

```yaml
i2s_audio:
  - id: i2s_bus
    i2s_lrclk_pin: GPIO5     # optional at bus level
    i2s_bclk_pin: GPIO4      # optional at bus level
    i2s_mclk_pin: GPIO0      # optional at bus level

microphone:
  - platform: i2s_audio
    id: i2s_mic
    i2s_audio_id: i2s_bus
    adc_type: external       # internal is rejected by final validation
    i2s_din_pin: GPIO6
    pdm: false
    channel: right           # left, right, or stereo; mono is rejected
    sample_rate: 16000
    bits_per_sample: 32bit   # 8bit, 16bit, 24bit, or 32bit
    i2s_mode: primary        # primary or secondary
    use_apll: false
    mclk_multiple: 256       # 128, 256, 384, or 512
    correct_dc_offset: false

speaker:
  - platform: i2s_audio
    id: i2s_speaker
    i2s_audio_id: i2s_bus
    dac_type: external       # internal is rejected by final validation
    i2s_dout_pin: GPIO7
    channel: mono            # mono, left, right, or stereo
    sample_rate: 16000
    bits_per_sample: 16bit
    i2s_mode: primary
    use_apll: false
    mclk_multiple: 256
    i2s_comm_fmt: stand_i2s  # stand_i2s, stand_msb, stand_pcm_short,
                             # stand_pcm_long, i2s_msb, i2s_lsb,
                             # pcm, pcm_short, or pcm_long
    spdif_mode: false
    buffer_duration: 500ms
    timeout: 500ms           # or never
    volume_multiplier: 3.0  # local extension; range 0.0–8.0
```

For S/PDIF output, set `spdif_mode: true`, `channel: stereo`,
`sample_rate: 44100` or `48000`, `use_apll: true`, `i2s_mode: primary`,
`i2s_comm_fmt: stand_i2s`, and `mclk_multiple: 256`.

The former `media_player: platform: i2s_audio` platform is intentionally
rejected. Use ESPHome's speaker media player with the speaker entity instead.

## Notes

- This override shadows the upstream component by name. When ESPHome upstream adds
  equivalent volume multiplication, remove this directory and its
  `external_components` entries.
- Port allocation logic (`_assign_ports`) prioritises PDM/ADC microphones to I2S
  port 0, which is the only port that supports PDM on most ESP32 variants.

## Demo

[devices/m5stack/m5cardputer.yaml](../../devices/m5stack/m5cardputer.yaml)
