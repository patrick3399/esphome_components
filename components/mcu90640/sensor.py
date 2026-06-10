import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

from . import CONF_MCU90640_ID, MCU90640BaseSchema

DEPENDENCIES = ["mcu90640"]

CONF_AVG_TEMPERATURE = "avg_temperature"
CONF_MIN_TEMPERATURE = "min_temperature"
CONF_MAX_TEMPERATURE = "max_temperature"
CONF_AMBIENT_TEMPERATURE = "ambient_temperature"
CONF_CENTROID_X = "centroid_x"
CONF_CENTROID_Y = "centroid_y"

_temp_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

_coord_schema = sensor.sensor_schema(
    unit_of_measurement="px",
    accuracy_decimals=2,
    icon="mdi:crosshairs",
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = MCU90640BaseSchema.extend(
    {
        cv.Optional(CONF_AVG_TEMPERATURE): _temp_schema,
        cv.Optional(CONF_MIN_TEMPERATURE): _temp_schema,
        cv.Optional(CONF_MAX_TEMPERATURE): _temp_schema,
        cv.Optional(CONF_AMBIENT_TEMPERATURE): _temp_schema,
        cv.Optional(CONF_CENTROID_X): _coord_schema,
        cv.Optional(CONF_CENTROID_Y): _coord_schema,
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MCU90640_ID])

    for key, setter in (
        (CONF_AVG_TEMPERATURE, "set_avg_temperature_sensor"),
        (CONF_MIN_TEMPERATURE, "set_min_temperature_sensor"),
        (CONF_MAX_TEMPERATURE, "set_max_temperature_sensor"),
        (CONF_AMBIENT_TEMPERATURE, "set_ambient_temperature_sensor"),
        (CONF_CENTROID_X, "set_centroid_x_sensor"),
        (CONF_CENTROID_Y, "set_centroid_y_sensor"),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(hub, setter)(sens))
