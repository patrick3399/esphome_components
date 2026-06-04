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
