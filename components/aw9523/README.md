# aw9523

AW9523B I2C GPIO expander with 16 GPIO pins and per-pin LED-driver mode.
Registered in ESPHome's pin schema so it works wherever a native GPIO pin is
accepted (`output`, `binary_sensor`, `switch`, etc.).

## Hardware

- Interface: I2C (address `0x58`–`0x5B` via AD0/AD1, default `0x59`)
- 16 GPIO pins (P0_0–P0_7, P1_0–P1_7)
- Each pin configurable as GPIO input, GPIO output, or constant-current LED sink
- `imax_divider`: scales the full-scale LED current — 0=full, 1=¾, 2=½, 3=¼

## Configuration

```yaml
aw9523:
  id: aw
  address: 0x59           # optional, default 0x59
  update_interval: 60s    # optional — polling interval for input refresh
  imax_divider: 0         # optional, default 0 (0=full, 1=3/4, 2=1/2, 3=1/4)
  latch_inputs: true      # optional, default true

# Use as output (relay, LED, etc.)
switch:
  - platform: gpio
    pin:
      aw9523: aw
      number: 0           # 0–15
      mode:
        output: true
    name: "Relay 1"

# Use as input (digital input, button)
binary_sensor:
  - platform: gpio
    pin:
      aw9523: aw
      number: 8
      mode:
        input: true
      inverted: true
    name: "Input 1"
```

`MULTI_CONF: true` — multiple expanders allowed.

## Notes

- `latch_inputs: true` holds the last read value between polling cycles, avoiding
  spurious state changes. Set to `false` only if you need raw real-time reads.
- I2C bus must be initialized before this component.
- Known issue: output shadow register must be pre-loaded from hardware on `setup()`
  to avoid clobbering pins not managed by this component (see commit a800ca9).

## Demo

[devices/m5stack/m5stamplc.yaml](../../devices/m5stack/m5stamplc.yaml)
