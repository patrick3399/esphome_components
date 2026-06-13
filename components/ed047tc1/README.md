# ed047tc1

ED047TC1 4.7-inch e-paper display driver built on the
[EPDIY](https://github.com/vroland/epdiy) library. Exposes a standard ESPHome
`display` entity compatible with LVGL.

## Hardware

Resolution: 960×540, 16-level greyscale (4-bit). The panel is driven by a
parallel 8-bit data bus plus several control signals — no SPI or I2C is involved.

Requires `esp32` target with `esp-idf` framework. The M5Stack Paper S3 is the
primary target (ESP32-S3 with 16 MB flash, 8 MB PSRAM).

## Configuration

```yaml
display:
  - platform: ed047tc1
    id: epaper
    pwr_pin: GPIO45
    bst_en_pin: GPIO46
    xstl_pin: GPIO47
    xle_pin: GPIO48
    spv_pin: GPIO1
    ckv_pin: GPIO2
    d0_pin: GPIO8
    d1_pin: GPIO9
    d2_pin: GPIO10
    d3_pin: GPIO11
    d4_pin: GPIO12
    d5_pin: GPIO13
    d6_pin: GPIO14
    d7_pin: GPIO15
    # pclk_pin: GPIO0  # optional pixel clock override
    lambda: |-
      it.fill(COLOR_OFF);
```

All `d0_pin`–`d7_pin` and most control pins are required. `pclk_pin` is optional
and defaults to a hardware-derived clock.

## Notes

- Partial refresh / differential updates are handled by EPDIY internally.
- LVGL integration: use `lvgl:` component with `display_id: epaper` and
  `color_depth: 1` or `4`. See [m5papers3.yaml](../../devices/m5stack/m5papers3.yaml)
  for a full working example.
- EPDIY is checked out at `epdiy/` in the workspace root and referenced via
  `external_components` in the device YAML.

## Demo

[devices/m5stack/m5papers3.yaml](../../devices/m5stack/m5papers3.yaml)
