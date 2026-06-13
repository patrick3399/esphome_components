# mcu90640

UART driver for the **GY-MCU90640** module — a MLX90640 32×24 thermal array sensor
with an onboard STM32 microcontroller. Exposes a HA camera entity (iron false-color
JPEG), temperature sensors, and presence/hot-spot binary sensors.

## Hardware

- Interface: UART at **115200 baud** (fixed — the STM32 owns the MLX's I2C bus)
- Module: GY-MCU90640 (grey PCB with MLX90640 + STM32)
- Sensor array: 32×24 pixels, range −40 °C to 300 °C
- Requires `esp32` target (uses `espressif/esp32-camera` for JPEG encoding)

> **Not for bare MLX90640 I2C breakouts.** This component speaks the GY-MCU90640
> 5A5A UART frame protocol. Driving a bare MLX90640 over I2C while the STM32 is
> also present will corrupt reads and can lock the I2C bus.

## Configuration

```yaml
uart:
  id: thermal_uart
  tx_pin: GPIO4
  rx_pin: GPIO5
  baud_rate: 115200

mcu90640:
  id: thermal
  uart_id: thermal_uart
  name: "Thermal Camera"
  update_interval: 1s        # optional, default 1 s
  idle_update_interval: 60s  # optional, default 60 s
  jpeg_quality: 80           # optional, default 80 (range 6–63)
  output_width: 32           # source-oriented width before rotation
  output_height: 24          # source-oriented height before rotation
  rotation: 90               # JPEG output becomes 24x32
  mirror_horizontal: false   # optional, default false
  mirror_vertical: false     # optional, default false
  emissivity: 1.0            # optional, default 1.0 (range 0.1–1.0)

sensor:
  - platform: mcu90640
    mcu90640_id: thermal
    avg_temperature:
      name: "Avg Temp"
    min_temperature:
      name: "Min Temp"
    max_temperature:
      name: "Max Temp"
    ambient_temperature:
      name: "Ambient Temp"    # STM32-reported ambient
    centroid_x:
      name: "Hot Centroid X"
    centroid_y:
      name: "Hot Centroid Y"

binary_sensor:
  - platform: mcu90640
    mcu90640_id: thermal
    presence:
      name: "Presence"
      threshold: 3.0     # optional, default 3.0 °C delta above background
      min_pixels: 5      # optional, default 5 pixels above threshold
    hot_spot:
      name: "Hot Spot"
      threshold: 50.0    # optional, default 50.0 °C absolute
```

## Notes

- `emissivity` adjusts temperature readings for non-blackbody surfaces (e.g.
  shiny metal ≈ 0.1, human skin ≈ 0.98).
- `idle_update_interval` reduces UART traffic when HA is not subscribed to the
  camera stream.
- A 90° or 270° rotation swaps the JPEG dimensions. The native 32x24 image
  becomes 24x32; a configured 64x48 output becomes 48x64.
- Centroid sensors report the pixel-coordinate center of mass of the hottest region.

## Demo

[devices/generic/mcu90640_poc.yaml](../../devices/generic/mcu90640_poc.yaml)
