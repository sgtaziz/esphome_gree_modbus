# Gree AC Modbus RTU ESPHome Component

ESPHome external component for controlling Gree commercial AC units via RS485 Modbus RTU.

## Tested Hardware

- **AC Unit**: Gree GU-Match series duct unit
- **Controller**: ESP32 / ESP8266 with RS485 transceiver (MAX485 or similar)
- **Connection**: COM-BMS (CN3) daughter board on the indoor unit

## Features

- Full climate control (On/Off, Mode, Temperature, Fan Speed)
- Temperature reading from indoor unit sensor
- Outdoor temperature sensor (optional)
- Vertical and horizontal swing control (via selects for granular positions)
- Sleep mode, Turbo mode, Fresh air valve switches
- 5 fan speeds + Auto + Turbo

## Wiring

Connect your RS485 transceiver to the COM-BMS / CN3 board:
- A+ (Data+)
- B- (Data-)
- GND (I used the metal chassis of the indoor unit)

UART settings: 9600 baud, 8N1

## Installation

Add to your ESPHome YAML:

```yaml
external_components:
  - source: github://sgtaziz/esphome_gree_modbus
    components: [gree_ac]
```

## Configuration

See [example.yaml](example.yaml) for a complete configuration example.

### Minimal Configuration

```yaml
uart:
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600

climate:
  - platform: gree_ac
    name: "Gree AC"
```

### Full Configuration

```yaml
uart:
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600

climate:
  - platform: gree_ac
    name: "Gree AC"
    slave_id: 1
    update_interval: 5s
    outdoor_temperature:
      name: "Outdoor Temperature"
    vertical_swing_select:
      name: "Vertical Swing"
    horizontal_swing_select:
      name: "Horizontal Swing"
    sleep_switch:
      name: "Sleep Mode"
    turbo_switch:
      name: "Turbo Mode"
    fresh_air_switch:
      name: "Fresh Air"
```

## Modbus Registers

Key registers used:
| Register | Description |
|----------|-------------|
| 2 | On/Off (85=Off, 170=On) |
| 3 | Current temperature × 10 (This may not always be available - register 4 can be used instead if not) |
| 17 | Mode (1=Cool, 2=Heat, 3=Dry, 4=Fan, 5=Auto) |
| 19 | Fan Speed (0=Auto, 1-5=Speed, 6=Turbo) |
| 20 | Set Temperature (whole degrees) |
| 22 | Vertical Swing |
| 23 | Horizontal Swing |
| 24 | Fresh Air Valve |
| 25 | Sleep Mode |
| 49 | Outdoor Temperature |

