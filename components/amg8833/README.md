# amg8833

Panasonic AMG8833 8×8 thermal camera driver. Exposes a HA camera entity
(iron false-color JPEG), temperature sensors, and presence/hot-spot binary sensors.

## Hardware

- Interface: I2C (address `0x68` or `0x69`, default `0x69`)
- Sensor array: 8×8 pixels, range −20 °C to 100 °C
- Requires `esp32` target (uses `espressif/esp32-camera` for JPEG encoding)

## Configuration

```yaml
amg8833:
  id: amg
  address: 0x69           # optional, default 0x69
  name: "Thermal Camera"
  update_interval: 1s     # optional, default 1 s — active capture rate
  idle_update_interval: 60s  # optional, default 60 s — rate when HA not subscribed
  jpeg_quality: 80        # optional, default 80 (range 1–100)
  output_width: 32        # optional, default 32 (range 8–320)
  output_height: 32       # optional, default 32 (range 8–320)
  rotation: 0             # optional: 0, 90, 180, 270

sensor:
  - platform: amg8833
    amg8833_id: amg
    avg_temperature:
      name: "Avg Temp"
    min_temperature:
      name: "Min Temp"
    max_temperature:
      name: "Max Temp"
    thermistor_temperature:
      name: "Thermistor"
    centroid_x:
      name: "Hot Centroid X"
    centroid_y:
      name: "Hot Centroid Y"

binary_sensor:
  - platform: amg8833
    amg8833_id: amg
    presence:
      name: "Presence"
      threshold: 3.0      # optional, default 3.0 °C delta above background
      min_pixels: 2       # optional, default 2 pixels above threshold
    hot_spot:
      name: "Hot Spot"
      threshold: 50.0     # optional, default 50.0 °C absolute
```

## Notes

- JPEG output is upscaled from 8×8 to `output_width`×`output_height` with iron
  palette colorization — the image is useful for debugging, not precision.
- A 90°/270° rotation swaps the output dimensions, so an `output_width`≠`output_height`
  image keeps the correct aspect ratio after rotation.
- `idle_update_interval` reduces bus traffic when no HA client is subscribed to
  the camera stream.
- Centroid sensors report the pixel-coordinate center of mass of the hottest region.

## Limitations & hardware notes

- **One camera per device.** ESPHome's camera core is a singleton — this component
  cannot coexist with `esp32_camera` or with the `mcu90640` thermal camera on the
  same node. A second camera marks itself failed at boot.
- **Large outputs need PSRAM / cost loop time.** JPEG is encoded synchronously in
  `loop()`. The 32×32 default is cheap, but pushing `output_*` toward 320 raises the
  RGB buffer (e.g. 320×320 ≈ 300 KB) and the encode time well past the 10 ms loop
  budget. Buffers use `RAMAllocator` (PSRAM-capable); on PSRAM-less boards keep the
  output small.
- **Variant support:** validated on ESP32-C3 and ESP32-S3. The `espressif/esp32-camera`
  dependency is pulled in only for its software JPEG encoder (no DVP camera needed).
- **No `on_image` / `on_stream_*` automations.** Unlike `esp32_camera`, this component
  exposes only the HA camera entity and the sensors; image-event triggers and the new
  core `Encoder` abstraction are not wired up yet.

## Demo

[devices/generic/amg8833_poc.yaml](../../devices/generic/amg8833_poc.yaml)
