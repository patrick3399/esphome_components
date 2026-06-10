import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.components.esp32 import add_idf_component
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)
from esphome.core.entity_helpers import setup_entity

AUTO_LOAD = ["camera", "sensor"]
DEPENDENCIES = ["i2c", "esp32"]
CODEOWNERS = ["@patrick3399"]

CONF_JPEG_QUALITY = "jpeg_quality"
CONF_OUTPUT_WIDTH = "output_width"
CONF_OUTPUT_HEIGHT = "output_height"
CONF_IDLE_UPDATE_INTERVAL = "idle_update_interval"
CONF_AVG_TEMPERATURE = "avg_temperature"
CONF_MIN_TEMPERATURE = "min_temperature"
CONF_MAX_TEMPERATURE = "max_temperature"
CONF_THERMISTOR_TEMPERATURE = "thermistor_temperature"

amg8833_ns = cg.esphome_ns.namespace("amg8833")
AMG8833Component = amg8833_ns.class_("AMG8833Component", cg.Component, cg.EntityBase)

_temp_sensor_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = (
    cv.ENTITY_BASE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(AMG8833Component),
            cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.update_interval,
            cv.Optional(CONF_IDLE_UPDATE_INTERVAL, default="15s"): cv.update_interval,
            cv.Optional(CONF_JPEG_QUALITY, default=80): cv.int_range(min=6, max=63),
            cv.Optional(CONF_OUTPUT_WIDTH, default=64): cv.int_range(min=8, max=320),
            cv.Optional(CONF_OUTPUT_HEIGHT, default=64): cv.int_range(min=8, max=320),
            cv.Optional(CONF_AVG_TEMPERATURE): _temp_sensor_schema,
            cv.Optional(CONF_MIN_TEMPERATURE): _temp_sensor_schema,
            cv.Optional(CONF_MAX_TEMPERATURE): _temp_sensor_schema,
            cv.Optional(CONF_THERMISTOR_TEMPERATURE): _temp_sensor_schema,
        }
    )
    .extend(i2c.i2c_device_schema(0x69))
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await setup_entity(var, config, "camera")
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_idle_update_interval(config[CONF_IDLE_UPDATE_INTERVAL]))
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_output_size(config[CONF_OUTPUT_WIDTH], config[CONF_OUTPUT_HEIGHT]))

    for key, setter in (
        (CONF_AVG_TEMPERATURE, "set_avg_temperature_sensor"),
        (CONF_MIN_TEMPERATURE, "set_min_temperature_sensor"),
        (CONF_MAX_TEMPERATURE, "set_max_temperature_sensor"),
        (CONF_THERMISTOR_TEMPERATURE, "set_thermistor_sensor"),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(sens))

    add_idf_component(name="espressif/esp32-camera", ref="2.1.5")
    cg.add_define("USE_CAMERA")
