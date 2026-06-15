from typing import Final

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.components.esp32 import add_idf_component
from esphome.const import CONF_ID, CONF_ROTATION, CONF_UPDATE_INTERVAL
from esphome.core.entity_helpers import setup_entity

AUTO_LOAD = ["camera"]
DEPENDENCIES = ["i2c", "esp32"]
CODEOWNERS = ["@patrick3399"]

CONF_AMG8833_ID: Final = "amg8833_id"
CONF_JPEG_QUALITY: Final = "jpeg_quality"
CONF_OUTPUT_WIDTH: Final = "output_width"
CONF_OUTPUT_HEIGHT: Final = "output_height"
CONF_IDLE_UPDATE_INTERVAL: Final = "idle_update_interval"

amg8833_ns = cg.esphome_ns.namespace("amg8833")
AMG8833Component = amg8833_ns.class_("AMG8833Component", cg.Component, cg.EntityBase)

AMG8833BaseSchema = cv.Schema(
    {
        cv.GenerateID(CONF_AMG8833_ID): cv.use_id(AMG8833Component),
    }
)

CONFIG_SCHEMA = (
    cv.ENTITY_BASE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(AMG8833Component),
            cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.update_interval,
            cv.Optional(CONF_IDLE_UPDATE_INTERVAL, default="60s"): cv.update_interval,
            cv.Optional(CONF_JPEG_QUALITY, default=80): cv.int_range(min=1, max=100),
            cv.Optional(CONF_OUTPUT_WIDTH, default=32): cv.int_range(min=8, max=320),
            cv.Optional(CONF_OUTPUT_HEIGHT, default=32): cv.int_range(min=8, max=320),
            cv.Optional(CONF_ROTATION, default=0): cv.one_of(0, 90, 180, 270, int=True),
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
    cg.add(var.set_rotation(config[CONF_ROTATION]))

    add_idf_component(name="espressif/esp32-camera", ref="2.1.5")
    cg.add_define("USE_CAMERA")
