# ys_irtm_uart

UART driver for the YS-IRTM infrared transceiver module. Supports NEC IR
transmission, raw hex transmission, HA IR/RF Proxy bridge mode, and IR receive
trigger.

## Hardware

- Interface: UART (TX+RX; baud rate 4800/9600/19200/57600, configurable at runtime)
- Module: YS-IRTM (small breakout with IR LED + IR receiver)
- Protocols: NEC TX/RX; raw hex for other protocols via HA Proxy

## Configuration

```yaml
uart:
  id: ir_uart
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600

ys_irtm_uart:
  id: ir
  uart_id: ir_uart
  on_ir_receive:
    - lambda: |-
        ESP_LOGD("ir", "received hi=%02X lo=%02X key=%02X",
                 user_code_hi, user_code_lo, key_code);
```

### Actions

```yaml
# Send NEC code
- ys_irtm_uart.send_nec:
    id: ir
    user_code_hi: 0x00
    user_code_lo: 0xFF
    key_code: 0x48
    repeats: 0           # optional, default 0

# Send raw hex string (e.g. from HA IR Proxy)
- ys_irtm_uart.send_raw:
    id: ir
    data: "AA BB 01 02"

# Send HA IR/RF Proxy packet (protocol name + address + command)
- ys_irtm_uart.send_proxy_packet:
    id: ir
    protocol: "NEC"
    address: 0x00FF
    command: 0x48B7
    repeats: 0

# Change module address (multi-module bus)
- ys_irtm_uart.set_address:
    id: ir
    address: 0x00

# Change baud rate (persisted by module)
- ys_irtm_uart.set_baudrate:
    id: ir
    baud_rate: 9600      # 4800, 9600, 19200, or 57600
```

### `on_ir_receive` trigger variables

| Variable | Type | Description |
|---|---|---|
| `user_code_hi` | `uint8_t` | NEC user code high byte |
| `user_code_lo` | `uint8_t` | NEC user code low byte |
| `key_code` | `uint8_t` | NEC key code byte |

## Demo

[devices/generic/ys-irtm-demo.yaml](../../devices/generic/ys-irtm-demo.yaml)
