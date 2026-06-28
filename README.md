# Gree AC Modbus RTU ESPHome Component

ESPHome external component for controlling Gree commercial AC units via RS485 Modbus RTU.

## Tested Hardware

- **AC Unit**: Gree U-Match series duct unit
- **Controller**: ESP32 / ESP8266 with RS485 transceiver (MAX485 or similar)
- **Connection**: COM-BMS (CN3) daughter board on the indoor unit

## Features

- Full climate control (On/Off, Mode, Temperature, Fan Speed)
- Temperature reading from indoor unit sensor
- Outdoor temperature sensor (optional)
- Vertical and horizontal swing control (via selects for granular positions)
- Sleep mode, Turbo mode, Fresh air valve switches
- 5 fan speeds + Auto + Turbo
- **Expose Sensors** toggle — publishes every state value as its own sensor
- **Current Temp Source** select — choose which register feeds the climate's
  current temperature (wired controller, IDU return air, return-air port, light board)
- **Debug mode** — dumps all registers 0–92 as a JSON text sensor for reverse
  engineering, plus an optional register read/write interface

## Wiring

Connect your RS485 transceiver to the COM-BMS / CN3 board:
- A+ (Data+)
- B- (Data-)
- GND (I used the metal chassis of the indoor unit)

UART settings: 9600 baud, 8N1

### RS485 Module Types

**Auto-direction modules** (recommended): These modules automatically switch between TX and RX modes. Just connect VCC, GND, TX, and RX - no additional GPIO needed.

**MAX485 modules** (with DE/RE pins): These require manual direction control. Wire DE and RE together to a GPIO pin and configure `flow_control_pin` in your YAML:

```
ESP32          MAX485
GPIO17  -----> DI
GPIO16  <----- RO
GPIO4   -----> DE + RE (directly connect or jumper together. Try only RE first - might work)
3.3V    -----> VCC
GND     -----> GND
```

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
    flow_control_pin: GPIO4  # Optional: only for MAX485 modules
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

### Expose Sensors

Set `expose_sensors: true` to publish each state value as a separate sensor
(named `"<climate name> <label>"`). Useful for logging and troubleshooting.

```yaml
climate:
  - platform: gree_ac
    name: "Gree AC"
    expose_sensors: true
```

This creates sensors for: Set Point, Current Temperature, Mode, Fan Speed,
Power State, Sleep, Turbo, Fresh Air, Contamination, Set Temperature Precise,
Ambient Return Air, and Ambient Light Board. (Outdoor temperature already has
its own `outdoor_temperature:` key, so it is not duplicated here.)

### Current Temperature Source

Some installations have multiple temperature sensors. Use `current_temp_source`
to pick which register feeds the climate's `current_temperature`:

```yaml
climate:
  - platform: gree_ac
    name: "Gree AC"
    current_temp_source:
      name: "Current Temp Source"
```

Options (selectable from Home Assistant):

| Option | Register | Scaling |
|--------|----------|---------|
| Wired Controller | 3 | ÷10, signed (default) |
| IDU Return Air | 4 | ÷1 |
| Return Air Port | 82 | ÷10, signed |
| Light Board | 83 | ÷10, signed |

Switching the source updates `current_temperature` instantly from the cached
register values (no waiting for the next poll).

### Debug Mode

Enable `debug_mode: true` to sweep every register 0–92 and publish the values
as a JSON text sensor (e.g. `{"0":1234,"1":null,...}`). Registers the unit
does not support show as `null`, which makes it easy to map out what exists.

```yaml
climate:
  - platform: gree_ac
    name: "Gree AC"
    debug_mode: true
```

To also **write** registers (e.g. to probe writeable addresses), enable
`debug_write: true`. This adds three entities: a register-address number
(0–92), a register-value number (0–65535), and a write button. Setting both
numbers and pressing the button writes that value to the unit using Modbus
function 0x10. `debug_write` implies `debug_mode`.

> ⚠️ Debug writes go straight to the AC with no guardrails. They are logged at
> WARNING level. Use at your own risk — writing the wrong register can change
> unit behavior.

## Modbus Registers

Key registers used:
| Register | Description |
|----------|-------------|
| 2 | On/Off (85=Off, 170=On) |
| 3 | Current temperature × 10 (wired controller sensor; default current-temp source) |
| 4 | Ambient temperature (IDU return air sensor) |
| 5 | IDU Address |
| 17 | Mode (1=Cool, 2=Heat, 3=Dry, 4=Fan, 5=Auto) |
| 19 | Fan Speed (0=Auto, 1-5=Speed, 6=Turbo) |
| 20 | Set Temperature (whole degrees) |
| 22 | Vertical Swing |
| 23 | Horizontal Swing |
| 24 | Fresh Air Valve |
| 25 | Sleep Mode |
| 34 | Contamination grade |
| 39 | Ambient temp sensor selection |
| 42 | Set Temperature × 10 (0.5°C precision, read-only) |
| 49 | Outdoor Temperature |
| 77 | DRED function |
| 82 | Ambient temp at return air port × 10 |
| 83 | Ambient temp of light board × 10 |

Use `debug_mode: true` to sweep and inspect all registers 0–92.

