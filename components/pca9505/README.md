## ESPHome PCA9505 Component

Usage:  
```yaml
external_components:
  - source: github://patrick3399/esphome_components
    components: [ pca9505 ]

i2c:
  sda: 41
  scl: 40
  scan: true

pca9505:
  - id: 'my_pca9505'
    address: 0x20 # Ensure this matches your PCA9505's I2C address (A0-A2 jumpers)

switch:
  - platform: gpio
    name: "PCA9505 LED 1 (Pin 0)"
    pin:
      pca9505: my_pca9505
      number: 0
      mode:
        output: true
      inverted: false # Or true, depending on your LED circuit
  - platform: gpio
    name: "PCA9505 Output Pin 15"
    pin:
      pca9505: my_pca9505
      number: 15
      mode:
        output: true

binary_sensor:
  - platform: gpio
    name: "PCA9505 Button 1 (Pin 39)"
    pin:
      pca9505: my_pca9505
      number: 39
      mode:
        input: true # PCA9505 has internal pull-ups, PCA9506 does not [cite: 5, 6]
                   # If using PCA9506, an external pull-up might be needed for buttons.
      # inverted: true # If your button pulls LOW when pressed and you
```
