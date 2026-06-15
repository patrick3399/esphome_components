# wled_bridge

ESPHome external component that exposes an ESPHome addressable LED strip as a
WLED-compatible bridge.

## Compatibility

- Target ESPHome: 2026.5.x and ESP-IDF on ESP32 / ESP32-S3.
- WLED API surface: based on WLED v16.0.0 JSON, UDP notifier, DDP, E1.31, and
  Art-Net conventions.
- Effect API IDs: JSON and presets use WLED's upstream `MODE_COUNT = 220`
  numbering. Internally implemented effects are mapped to those fixed WLED mode
  IDs. Slots that are not implemented by this component are exposed as
  `Unsupported` in `/json/effects` and have empty `/json/fxdata` metadata.
- The component is not a full WLED firmware replacement. Filesystem features,
  OTA UI management, WiFi provisioning, DMX output, usermods, and unsupported
  particle effects are intentionally out of scope.

## Configuration Notes

Use either `light_id` for a single ESPHome addressable light, or `buses:` for a
virtual strip composed from multiple ESPHome addressable lights. They are
mutually exclusive.

### Complete Configuration

This canonical example exposes every current non-inherited component control.
The deprecated flat realtime keys are documented separately below and must not
be mixed with `realtime:`.

```yaml
wled_bridge:
  id: wled

  # Use this for one addressable ESPHome light:
  light_id: led_strip
  max_ma: 5000
  led_ma: 55

  # Or replace light_id/max_ma/led_ma with a buses list:
  # buses:
  #   - light_id: led_strip
  #     type: addressable
  #     max_ma: 5000
  #     led_ma: 55
  #   - light_id: rgbw_light
  #     type: rgbw
  #     max_ma: 2000
  #     led_ma: 55

  use_task: false             # true is currently rejected
  auto_white: none            # none, brighter, accurate, or max

  matrix_width: 0             # width and height must both be zero or positive
  matrix_height: 0
  matrix_serpentine: false

  udp_port: 21324
  udp_port2: 65506
  udp_send: false
  udp_receive: false

  boot_preset: 0              # 0 restores NVS; 1–16 loads that preset
  brightness_factor: 100      # 0–100 percent
  web_ui: true

  realtime:
    ddp: false
    e131: false
    e131_universe: 1
    e131_universes: 1
    artnet: false
    artnet_universe: 0
    artnet_universes: 1

  audio:
    microphone: i2s_mic
    passive: false
    fft: true
    agc: true

  palette_select:
    name: "WLED Palette"
  effect_select:
    name: "WLED Effect"
  speed_number:
    name: "WLED Speed"
  intensity_number:
    name: "WLED Intensity"
  preset_select:
    name: "WLED Preset"
  nightlight_switch:
    name: "WLED Nightlight"
  sync_send_switch:
    name: "WLED Sync Send"
  sync_receive_switch:
    name: "WLED Sync Receive"
  estimated_current:
    name: "WLED Estimated Current"
```

Supported `buses[].type` values are `addressable`, `mono`/`monochromatic`,
`on_off`, `rgb`, `rgbw`, `rgbww`, `rgbct`, and
`cct`/`cwww`/`cold_warm_white`.

For backward compatibility, the top level also accepts `ddp_receive`,
`e131_receive`, `e131_universe`, `e131_universe_count`, `artnet_receive`,
`artnet_universe`, and `artnet_universe_count`. New configurations should use
the nested `realtime:` form.

### Automation Actions

```yaml
script:
  - id: exercise_wled_controls
    then:
      - wled_bridge.power_on: wled
      - wled_bridge.power_off: wled
      - wled_bridge.power_toggle: wled

      - wled_bridge.set_brightness:
          id: wled
          brightness: 128
      - wled_bridge.set_color:
          id: wled
          red: 255
          green: 64
          blue: 0
          white: 0
      - wled_bridge.set_effect:
          id: wled
          effect: 0
      - wled_bridge.set_palette:
          id: wled
          palette: 0
      - wled_bridge.set_speed:
          id: wled
          speed: 128
      - wled_bridge.set_intensity:
          id: wled
          intensity: 128

      - wled_bridge.save_preset:
          id: wled
          preset: 1
      - wled_bridge.load_preset:
          id: wled
          preset: 1
      - wled_bridge.delete_preset:
          id: wled
          preset: 1

      - wled_bridge.start_nightlight:
          id: wled
          duration: 60
          target_brightness: 0
          mode: 1
      - wled_bridge.stop_nightlight: wled

      - wled_bridge.set_segment_bounds:
          id: wled
          start: 0
          stop: 60
      - wled_bridge.set_segment_reverse:
          id: wled
          reverse: false
      - wled_bridge.set_segment_mirror:
          id: wled
          mirror: false
      - wled_bridge.set_transition:
          id: wled
          transition: 7
      - wled_bridge.stop_playlist: wled
```

Current validation rules intentionally reject unsafe or ambiguous settings:

- `use_task: true` is rejected until the render path is made thread-safe.
- `matrix_width` and `matrix_height` must both be zero, or both greater than
  zero.
- E1.31 universe ranges must stay within 1..63999.
- Art-Net universe ranges must stay within 0..32767.
- When UDP sync receive is enabled, `udp_port` and `udp_port2` must not collide
  with enabled realtime receiver ports: DDP 4048, E1.31 5568, or Art-Net 6454.
  Runtime `/win` updates to those ports are also ignored when the corresponding
  realtime receiver is enabled.
- `max_ma: 0` means unlimited current for that bus. Positive `max_ma` values
  enable software brightness limiting.

## Runtime Safety

- JSON POST bodies are capped at 8192 bytes and return HTTP 413 when exceeded.
- Realtime receivers are non-blocking UDP sockets and process a bounded number
  of packets per loop tick.
- DDP, E1.31, and Art-Net each lock to the first valid source IP for the active
  stream. Other source IPs are ignored with a throttled warning until the stream
  times out.
- Pixel override updates batch state changes per realtime packet to avoid state
  churn.
- Automatic brightness limiting uses the same auto-white conversion path as the
  actual LED output, so RGBW current estimates include derived white channel
  output.

## Home Assistant Entities

Optional entities can be enabled with `palette_select`, `effect_select`,
`speed_number`, `intensity_number`, `preset_select`, `nightlight_switch`,
`sync_send_switch`, `sync_receive_switch`, and `estimated_current`.

The effect select only lists supported effects. JSON clients still see WLED's
full 220 mode slots for ID compatibility.

## Version Strategy

Treat this component as a WLED v16 compatibility bridge. Any future change to
WLED effect IDs, JSON fields, UDP packet layout, or palette/effect metadata
should update the compatibility note above and the source-level contract tests
under `tests/test_wled_bridge_json_contract.py`.
