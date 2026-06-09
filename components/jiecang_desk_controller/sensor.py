import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_HEIGHT,
    CONF_ID,
    UNIT_CENTIMETER,
    UNIT_PERCENT,
)

from . import CONF_JIECANG_DESK_CONTROLLER_ID, JiecangDeskController

DEPENDENCIES = ["jiecang_desk_controller"]

CONF_HEIGHT_MIN = "height_min"
CONF_HEIGHT_MAX = "height_max"
CONF_HEIGHT_PCT = "height_pct"
CONF_M1 = "m1"
CONF_M2 = "m2"
CONF_M3 = "m3"
CONF_M4 = "m4"

_SENSOR_CM = sensor.sensor_schema(
    unit_of_measurement=UNIT_CENTIMETER,
    accuracy_decimals=1,
)
_SENSOR_PCT = sensor.sensor_schema(
    unit_of_measurement=UNIT_PERCENT,
    accuracy_decimals=1,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
        cv.GenerateID(CONF_JIECANG_DESK_CONTROLLER_ID): cv.use_id(
            JiecangDeskController
        ),
        cv.Optional(CONF_HEIGHT): _SENSOR_CM,
        cv.Optional(CONF_HEIGHT_MIN): _SENSOR_CM,
        cv.Optional(CONF_HEIGHT_MAX): _SENSOR_CM,
        cv.Optional(CONF_HEIGHT_PCT): _SENSOR_PCT,
        cv.Optional(CONF_M1): _SENSOR_CM,
        cv.Optional(CONF_M2): _SENSOR_CM,
        cv.Optional(CONF_M3): _SENSOR_CM,
        cv.Optional(CONF_M4): _SENSOR_CM,
    }
)

_SENSOR_MAP = {
    CONF_HEIGHT: "set_height_sensor",
    CONF_HEIGHT_MIN: "set_height_min_sensor",
    CONF_HEIGHT_MAX: "set_height_max_sensor",
    CONF_HEIGHT_PCT: "set_height_pct_sensor",
    CONF_M1: "set_m1_sensor",
    CONF_M2: "set_m2_sensor",
    CONF_M3: "set_m3_sensor",
    CONF_M4: "set_m4_sensor",
}


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JIECANG_DESK_CONTROLLER_ID])
    for key, setter in _SENSOR_MAP.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(hub, setter)(sens))
