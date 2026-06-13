# 74hc138_keypad

Keyboard matrix scanner for designs using a 74HC138 3-to-8 decoder. Integrates
with ESPHome's `key_provider` so any `matrix_keypad`-compatible consumer (e.g.
`keyboard_keypad`) can receive key events.

## Hardware

The 74HC138 drives 8 column lines from 3 address pins. Seven row sense lines are
read simultaneously each scan cycle. With one decoder chip the maximum matrix size
is 8 columns × 7 rows = 56 keys. The M5Stack Cardputer uses exactly this layout.

## Configuration

```yaml
74hc138_keypad:
  address_pins:
    - pin: GPIO8   # A0 (LSB)
    - pin: GPIO9   # A1
    - pin: GPIO11  # A2 (MSB)
  input_pins:
    - pin: GPIO1
    - pin: GPIO2
    - pin: GPIO3
    - pin: GPIO4
    - pin: GPIO5
    - pin: GPIO6
    - pin: GPIO7
  debounce_time: 5ms   # optional, default 5 ms
  on_key:
    - lambda: ESP_LOGD("key", "key %d", x);
```

| Option | Required | Default | Notes |
|---|---|---|---|
| `address_pins` | yes | — | Exactly 3 GPIO output pins (A0, A1, A2) |
| `input_pins` | yes | — | Exactly 7 GPIO input pins (rows) |
| `debounce_time` | no | `5ms` | Minimum stable time before key event fires |
| `on_key` | no | — | Automation trigger; `x` is `uint8_t` key index (0–55) |

`MULTI_CONF: true` — multiple instances allowed.

## Demo

[devices/m5stack/m5cardputer.yaml](../../devices/m5stack/m5cardputer.yaml)
