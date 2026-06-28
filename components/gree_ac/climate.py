"""
Gree AC Modbus RTU ESPHome Component - Climate Platform

Controls Gree commercial AC units via RS485 Modbus RTU.
Tested with: Gree GFH36K3FI / GUHD36NK3FO
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import climate, uart, sensor, select, switch, text_sensor, number, button
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

AUTO_LOAD = ["switch", "sensor", "select", "text_sensor", "number", "button"]
DEPENDENCIES = ["uart"]

# Namespace and class definitions
gree_ac_ns = cg.esphome_ns.namespace("gree_ac")
GreeAC = gree_ac_ns.class_("GreeAC", cg.Component, uart.UARTDevice, climate.Climate)

GreeACSwitch = gree_ac_ns.class_(
    "GreeACSwitch", switch.Switch, cg.Component
)
GreeACSelect = gree_ac_ns.class_(
    "GreeACSelect", select.Select, cg.Component
)
GreeACNumber = gree_ac_ns.class_(
    "GreeACNumber", number.Number, cg.Component
)
GreeACButton = gree_ac_ns.class_(
    "GreeACButton", button.Button, cg.Component
)
# Debug register dump uses ESPHome's built-in TextSensor directly (only
# publish_state is needed), so no custom subclass is required.

# Configuration keys
CONF_SLAVE_ID = "slave_id"
CONF_FLOW_CONTROL_PIN = "flow_control_pin"
CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_VERTICAL_SWING_SELECT = "vertical_swing_select"
CONF_HORIZONTAL_SWING_SELECT = "horizontal_swing_select"
CONF_SLEEP_SWITCH = "sleep_switch"
CONF_TURBO_SWITCH = "turbo_switch"
CONF_FRESH_AIR_SWITCH = "fresh_air_switch"
CONF_EXPOSE_SENSORS = "expose_sensors"
CONF_CURRENT_TEMP_SOURCE = "current_temp_source"
CONF_DEBUG_MODE = "debug_mode"
CONF_DEBUG_WRITE = "debug_write"

# Options for the current-temp source select (must match C++ CURRENT_TEMP_SOURCE_OPTIONS)
CURRENT_TEMP_SOURCE_OPTIONS = [
    "Wired Controller",
    "IDU Return Air",
    "Return Air Port",
    "Light Board",
]

# Exposed-sensor definitions: (setter_attr, label_suffix, unit, decimals, device_class, state_class)
# Each becomes a sensor named "${climate_name} <label_suffix>" when expose_sensors is true.
EXPOSED_SENSOR_DEFS = [
    ("set_point", "Set Point", UNIT_CELSIUS, 0, DEVICE_CLASS_TEMPERATURE, STATE_CLASS_MEASUREMENT),
    ("current_temp", "Current Temperature", UNIT_CELSIUS, 1, DEVICE_CLASS_TEMPERATURE, STATE_CLASS_MEASUREMENT),
    ("mode", "Mode", None, 0, None, None),
    ("fan_speed", "Fan Speed", None, 0, None, None),
    ("on_off", "Power State", None, 0, None, None),
    ("sleep", "Sleep", None, 0, None, None),
    ("turbo", "Turbo", None, 0, None, None),
    ("fresh_air", "Fresh Air", None, 0, None, None),
    ("contamination", "Contamination", None, 0, None, None),
    ("set_temp_precise", "Set Temperature Precise", UNIT_CELSIUS, 1, DEVICE_CLASS_TEMPERATURE, STATE_CLASS_MEASUREMENT),
    ("ambient_return_air", "Ambient Return Air", UNIT_CELSIUS, 1, DEVICE_CLASS_TEMPERATURE, STATE_CLASS_MEASUREMENT),
    ("ambient_light_board", "Ambient Light Board", UNIT_CELSIUS, 1, DEVICE_CLASS_TEMPERATURE, STATE_CLASS_MEASUREMENT),
    # Note: outdoor temperature is intentionally NOT in this list — it already
    # has its own dedicated `outdoor_temperature:` config key to avoid a name clash.
]

# Swing options (must match C++ code)
VERTICAL_SWING_OPTIONS = [
    "Off",
    "Full Swing",
    "Position 1 (Up)",
    "Position 2",
    "Position 3 (Middle)",
    "Position 4",
    "Position 5 (Down)",
    "Swing Upper",
    "Swing Middle",
    "Swing Lower",
    "Swing Upper-Middle",
    "Swing Middle-Lower",
]

HORIZONTAL_SWING_OPTIONS = [
    "Off",
    "Full Swing",
    "Position 1 (Left)",
    "Position 2",
    "Position 3 (Middle)",
    "Position 4",
    "Position 5 (Right)",
    "Swing Left",
    "Swing Right",
]

# Switch schema (matching sinclair_ac pattern)
switch_schema = switch.switch_schema(switch.Switch).extend(cv.COMPONENT_SCHEMA).extend(
    {cv.GenerateID(): cv.declare_id(GreeACSwitch)}
)

# Select schema (matching sinclair_ac pattern)
select_schema = select.select_schema(select.Select).extend(
    {cv.GenerateID(CONF_ID): cv.declare_id(GreeACSelect)}
)

# Sensor schema for outdoor temperature
outdoor_temp_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = (
    climate.climate_schema(GreeAC)
    .extend(
        {
            cv.Optional(CONF_SLAVE_ID, default=1): cv.int_range(min=1, max=247),
            cv.Optional(
                CONF_UPDATE_INTERVAL, default="5s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_OUTDOOR_TEMPERATURE): outdoor_temp_schema,
            cv.Optional(CONF_VERTICAL_SWING_SELECT): select_schema,
            cv.Optional(CONF_HORIZONTAL_SWING_SELECT): select_schema,
            cv.Optional(CONF_SLEEP_SWITCH): switch_schema,
            cv.Optional(CONF_TURBO_SWITCH): switch_schema,
            cv.Optional(CONF_FRESH_AIR_SWITCH): switch_schema,
            cv.Optional(CONF_EXPOSE_SENSORS, default=False): cv.boolean,
            cv.Optional(CONF_CURRENT_TEMP_SOURCE): select_schema,
            cv.Optional(CONF_DEBUG_MODE, default=False): cv.boolean,
            cv.Optional(CONF_DEBUG_WRITE, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await climate.register_climate(var, config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))

    # Flow control pin (DE/RE for MAX485 modules)
    if CONF_FLOW_CONTROL_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))

    # Outdoor temperature sensor
    if CONF_OUTDOOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_TEMPERATURE])
        cg.add(var.set_outdoor_temperature_sensor(sens))

    # Vertical swing select
    if CONF_VERTICAL_SWING_SELECT in config:
        conf = config[CONF_VERTICAL_SWING_SELECT]
        sel = await select.new_select(conf, options=VERTICAL_SWING_OPTIONS)
        await cg.register_component(sel, conf)
        cg.add(var.set_vertical_swing_select(sel))

    # Horizontal swing select
    if CONF_HORIZONTAL_SWING_SELECT in config:
        conf = config[CONF_HORIZONTAL_SWING_SELECT]
        sel = await select.new_select(conf, options=HORIZONTAL_SWING_OPTIONS)
        await cg.register_component(sel, conf)
        cg.add(var.set_horizontal_swing_select(sel))

    # Sleep switch
    if CONF_SLEEP_SWITCH in config:
        conf = config[CONF_SLEEP_SWITCH]
        sw = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(sw, conf)
        await switch.register_switch(sw, conf)
        cg.add(var.set_sleep_switch(sw))

    # Turbo switch
    if CONF_TURBO_SWITCH in config:
        conf = config[CONF_TURBO_SWITCH]
        sw = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(sw, conf)
        await switch.register_switch(sw, conf)
        cg.add(var.set_turbo_switch(sw))

    # Fresh air switch
    if CONF_FRESH_AIR_SWITCH in config:
        conf = config[CONF_FRESH_AIR_SWITCH]
        sw = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(sw, conf)
        await switch.register_switch(sw, conf)
        cg.add(var.set_fresh_air_switch(sw))

    # Current temperature source select (reg 3/4/82/83)
    if CONF_CURRENT_TEMP_SOURCE in config:
        conf = config[CONF_CURRENT_TEMP_SOURCE]
        sel = await select.new_select(conf, options=CURRENT_TEMP_SOURCE_OPTIONS)
        await cg.register_component(sel, conf)
        cg.add(var.set_current_temp_source_select(sel))

    # Expose Sensors toggle: spawn a sensor per definition, named "<climate> <label>"
    if config.get(CONF_EXPOSE_SENSORS):
        cg.add(var.set_expose_sensors(True))
        climate_name = config.get(CONF_NAME, "")
        for attr, label, unit, decimals, device_class, state_class in EXPOSED_SENSOR_DEFS:
            sens_conf = {
                CONF_NAME: f"{climate_name} {label}".strip(),
                CONF_ID: f"gree_exposed_{attr}",
            }
            # Build schema kwargs, omitting None values (they aren't valid for
            # unit_of_measurement/device_class/state_class).
            schema_kwargs = {"accuracy_decimals": decimals}
            if unit is not None:
                schema_kwargs["unit_of_measurement"] = unit
            if device_class is not None:
                schema_kwargs["device_class"] = device_class
            if state_class is not None:
                schema_kwargs["state_class"] = state_class
            sens = await sensor.new_sensor(sensor.sensor_schema(**schema_kwargs)(sens_conf))
            cg.add(getattr(var, f"set_{attr}_sensor")(sens))

    # Debug mode: a text sensor dumping every register 0..92 as JSON.
    # debug_write implies debug_mode (the dump is needed to see what changed).
    if config.get(CONF_DEBUG_MODE) or config.get(CONF_DEBUG_WRITE):
        cg.add(var.set_debug_mode(True))
        base = config.get(CONF_NAME, "")
        ts_conf = text_sensor.text_sensor_schema(text_sensor.TextSensor)(
            {CONF_NAME: f"{base} Debug Registers".strip(), CONF_ID: "gree_debug_registers"}
        )
        ts = cg.new_Pvariable(ts_conf[CONF_ID])
        await text_sensor.register_text_sensor(ts, ts_conf)
        cg.add(var.set_debug_text_sensor(ts))

    # Debug write: address + value numbers and a write button
    if config.get(CONF_DEBUG_WRITE):
        base = config.get(CONF_NAME, "")

        addr_conf = number.number_schema(GreeACNumber)(
            {CONF_NAME: f"{base} Debug Register Address".strip(), CONF_ID: "gree_debug_address"}
        )
        addr = cg.new_Pvariable(addr_conf[CONF_ID])
        await number.register_number(addr, addr_conf, min_value=0, max_value=92, step=1)
        cg.add(var.set_debug_address_number(addr))

        val_conf = number.number_schema(GreeACNumber)(
            {CONF_NAME: f"{base} Debug Register Value".strip(), CONF_ID: "gree_debug_value"}
        )
        val = cg.new_Pvariable(val_conf[CONF_ID])
        await number.register_number(val, val_conf, min_value=0, max_value=65535, step=1)
        cg.add(var.set_debug_value_number(val))

        btn_conf = button.button_schema(GreeACButton)(
            {CONF_NAME: f"{base} Debug Write".strip(), CONF_ID: "gree_debug_write"}
        )
        btn = cg.new_Pvariable(btn_conf[CONF_ID])
        await button.register_button(btn, btn_conf)
        cg.add(var.set_debug_write_button(btn))
