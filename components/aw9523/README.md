# aw9523

AW9523B I2C GPIO expander with 16 GPIO pins. Registered in ESPHome's pin schema
so it works wherever a native GPIO pin is accepted (`output`,
`binary_sensor`, `switch`, etc.).

## Hardware

- Interface: I2C (address `0x58`–`0x5B` via AD0/AD1, default `0x59`)
- 16 GPIO pins (P0_0–P0_7, P1_0–P1_7)
- Each pin is configurable from YAML as a GPIO input or GPIO output
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
- During `setup()`, the component soft-resets the expander, initializes all pins
  as GPIO inputs, and reads `OUTPUT0`/`OUTPUT1` into its output shadow. Subsequent
  per-pin writes therefore preserve the other output-latch bits.
- The soft reset applies to the whole expander. Do not share the same AW9523 with
  another driver that expects its previous direction, LED-mode, or output state
  to remain intact.
- The C++ driver contains constant-current LED register support, but no ESPHome
  `output` platform currently exposes that mode to YAML. `imax_divider` only
  preconfigures the chip-wide current scale for future LED-mode use.

## Demo

[devices/m5stack/m5stamplc.yaml](../../devices/m5stack/m5stamplc.yaml)
