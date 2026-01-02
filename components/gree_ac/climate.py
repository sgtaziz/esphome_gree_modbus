"""
Gree AC Modbus RTU ESPHome Component - Climate Platform

Controls Gree commercial AC units via RS485 Modbus RTU.
Tested with: Gree GFH36K3FI / GUHD36NK3FO
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart, sensor, select, switch
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

AUTO_LOAD = ["switch", "sensor", "select"]
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

# Configuration keys
CONF_SLAVE_ID = "slave_id"
CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_VERTICAL_SWING_SELECT = "vertical_swing_select"
CONF_HORIZONTAL_SWING_SELECT = "horizontal_swing_select"
CONF_SLEEP_SWITCH = "sleep_switch"
CONF_TURBO_SWITCH = "turbo_switch"
CONF_FRESH_AIR_SWITCH = "fresh_air_switch"

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
            cv.Optional(CONF_OUTDOOR_TEMPERATURE): outdoor_temp_schema,
            cv.Optional(CONF_VERTICAL_SWING_SELECT): select_schema,
            cv.Optional(CONF_HORIZONTAL_SWING_SELECT): select_schema,
            cv.Optional(CONF_SLEEP_SWITCH): switch_schema,
            cv.Optional(CONF_TURBO_SWITCH): switch_schema,
            cv.Optional(CONF_FRESH_AIR_SWITCH): switch_schema,
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
