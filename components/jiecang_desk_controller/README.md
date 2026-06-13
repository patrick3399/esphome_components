# jiecang_desk_controller

UART driver for Jiecang (捷昌) sit-stand desk controllers. Decodes the proprietary
serial protocol to expose desk height, memory positions, and control buttons as
ESPHome entities.

## Hardware

- Interface: UART (TX+RX required, no parity, 1 stop bit)
- Compatible with Jiecang linear actuator controllers that expose a UART port
  (typically via the RJ45 handset connector)

## Configuration

```yaml
uart:
  id: desk_uart
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600   # check your controller's documented baud rate

jiecang_desk_controller:
  id: desk
  uart_id: desk_uart

sensor:
  - platform: jiecang_desk_controller
    jiecang_desk_controller_id: desk
    height:
      name: "Desk Height"         # cm
    height_min:
      name: "Min Height"          # cm
    height_max:
      name: "Max Height"          # cm
    height_pct:
      name: "Height %"            # %
    m1:
      name: "Memory 1"            # cm
    m2:
      name: "Memory 2"
    m3:
      name: "Memory 3"
    m4:
      name: "Memory 4"

number:
  - platform: jiecang_desk_controller
    jiecang_desk_controller_id: desk
    height:
      name: "Target Height"       # set height in cm
    height_pct:
      name: "Target Height %"     # set height as %

button:
  - platform: jiecang_desk_controller
    jiecang_desk_controller_id: desk
    step_up:
      name: "Step Up"
    step_down:
      name: "Step Down"
    stop:
      name: "Stop"
    move_up:
      name: "Move Up"
    move_down:
      name: "Move Down"
    goto_m1:
      name: "Go to M1"
    goto_m2:
      name: "Go to M2"
    goto_m3:
      name: "Go to M3"
    goto_m4:
      name: "Go to M4"
    save_m1:
      name: "Save M1"
    save_m2:
      name: "Save M2"
    save_m3:
      name: "Save M3"
    save_m4:
      name: "Save M4"
```

All sensor, number, and button keys are optional — only declare what you need.

## Notes

- Baud rate varies by controller model. Check the Jiecang documentation or sniff
  the handset communication to confirm.
- The UART requires both TX and RX. TX sends commands; RX receives height updates.
