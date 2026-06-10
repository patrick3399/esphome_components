import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

from . import AMG8833BaseSchema, CONF_AMG8833_ID

DEPENDENCIES = ["amg8833"]

CONF_AVG_TEMPERATURE = "avg_temperature"
CONF_MIN_TEMPERATURE = "min_temperature"
CONF_MAX_TEMPERATURE = "max_temperature"
CONF_THERMISTOR_TEMPERATURE = "thermistor_temperature"

_temp_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = AMG8833BaseSchema.extend(
    {
        cv.Optional(CONF_AVG_TEMPERATURE): _temp_schema,
        cv.Optional(CONF_MIN_TEMPERATURE): _temp_schema,
        cv.Optional(CONF_MAX_TEMPERATURE): _temp_schema,
        cv.Optional(CONF_THERMISTOR_TEMPERATURE): _temp_schema,
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_AMG8833_ID])

    for key, setter in (
        (CONF_AVG_TEMPERATURE, "set_avg_temperature_sensor"),
        (CONF_MIN_TEMPERATURE, "set_min_temperature_sensor"),
        (CONF_MAX_TEMPERATURE, "set_max_temperature_sensor"),
        (CONF_THERMISTOR_TEMPERATURE, "set_thermistor_sensor"),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(hub, setter)(sens))
