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
  performance_profile: auto
  update_interval: 1s        # optional, default 1 s
  idle_update_interval: 60s  # optional, default 60 s
  # jpeg_quality/output_* are optional profile overrides
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

- `performance_profile` controls image defaults and memory placement:
  - `low_memory`: 64×48, quality 75, internal RAM, maximum 4800 output pixels.
  - `balanced`: 96×72, quality 80; uses PSRAM when it is guaranteed.
  - `high_quality`: 160×120, quality 85 and PSRAM-backed image buffers.
  - `auto` selects `high_quality` only when `psram:` sets
    `ignore_not_found: false`; otherwise it selects `low_memory`.
- Explicit `output_width`, `output_height`, and `jpeg_quality` override the
  profile defaults. JPEG quality accepts 1–100.
- `high_quality` is rejected unless PSRAM is guaranteed. ESP32-S3 alone does not
  imply that PSRAM is fitted or configured.
- `emissivity` adjusts temperature readings for non-blackbody surfaces (e.g.
  shiny metal ≈ 0.1, human skin ≈ 0.98).
- The module continues streaming UART data at 4 Hz. `idle_update_interval` only
  reduces analytics publication and image rendering when there are no consumers.
- A 90° or 270° rotation swaps the JPEG dimensions. The native 32x24 image
  becomes 24x32; a configured 64x48 output becomes 48x64.
- Centroid sensors report the pixel-coordinate center of mass of the hottest region.
- **Increase the UART RX buffer.** The module streams ~6.2 kB/s; ESPHome's default
  `rx_buffer_size` (256 B) can overflow if the main loop is busy, producing checksum
  failures and dropped frames. Set `rx_buffer_size: 2048` on the `uart:` bus.
- **Stream-loss recovery:** if the module goes silent for >5 s after streaming is
  commanded, including before the first valid frame, the component re-arms
  streaming and raises a dedicated warning until frames resume.
- Checksum failure searches the buffered frame for the next `5A 5A` marker,
  reducing recovery time after UART overflow or byte loss.

## Limitations & hardware notes

- **One camera per device.** ESPHome's camera core is a singleton — this component
  cannot coexist with `esp32_camera` or with the `amg8833` thermal camera on the same
  node. A second camera marks itself failed at boot.
- **Large outputs cost loop time.** JPEG is encoded synchronously in the RX path.
  The default output is cheap, but large `output_*` values raise both the RGB buffer
  and the encode time past the 10 ms loop budget. Large image buffers use external
  RAM only when PSRAM is guaranteed; UART and temperature data stay internal.
- Only one published frame may be outstanding. Slow clients cause newer camera
  frames to be skipped instead of allowing allocations to accumulate.
- The JPEG buffer grows up to the selected profile limit and retries encoding.
  Oversized frames are dropped safely.
- **Variant support:** validated on ESP32-C3 and ESP32-S3. The `espressif/esp32-camera`
  dependency is pulled in only for its software JPEG encoder.
- **No `on_image` / `on_stream_*` automations.** Only the HA camera entity and the
  sensors are exposed; image-event triggers and the new core `Encoder` abstraction are
  not wired up yet.

## Demo

[devices/generic/mcu90640_poc.yaml](../../devices/generic/mcu90640_poc.yaml)
