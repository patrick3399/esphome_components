from typing import Final

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, psram
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
CONF_PERFORMANCE_PROFILE: Final = "performance_profile"

PROFILE_AUTO = "auto"
PROFILE_LOW_MEMORY = "low_memory"
PROFILE_BALANCED = "balanced"
PROFILE_HIGH_QUALITY = "high_quality"

PROFILE_DEFAULTS = {
    PROFILE_LOW_MEMORY: (32, 32, 75, 8192, 16384),
    PROFILE_BALANCED: (64, 64, 80, 16384, 65536),
    PROFILE_HIGH_QUALITY: (128, 128, 85, 32768, 262144),
}

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
            cv.Optional(CONF_PERFORMANCE_PROFILE, default=PROFILE_AUTO): cv.one_of(
                PROFILE_AUTO,
                PROFILE_LOW_MEMORY,
                PROFILE_BALANCED,
                PROFILE_HIGH_QUALITY,
                lower=True,
            ),
            cv.Optional(CONF_JPEG_QUALITY): cv.int_range(min=1, max=100),
            cv.Optional(CONF_OUTPUT_WIDTH): cv.int_range(min=8, max=320),
            cv.Optional(CONF_OUTPUT_HEIGHT): cv.int_range(min=8, max=320),
            cv.Optional(CONF_ROTATION, default=0): cv.one_of(0, 90, 180, 270, int=True),
        }
    )
    .extend(i2c.i2c_device_schema(0x69))
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    psram_guaranteed = psram.is_guaranteed()
    profile = config[CONF_PERFORMANCE_PROFILE]
    if profile == PROFILE_AUTO:
        profile = PROFILE_HIGH_QUALITY if psram_guaranteed else PROFILE_LOW_MEMORY
    if profile == PROFILE_HIGH_QUALITY and not psram_guaranteed:
        raise cv.Invalid(
            "performance_profile high_quality requires a guaranteed psram: configuration "
            "(set ignore_not_found: false)"
        )

    width_default, height_default, quality_default, jpeg_initial, jpeg_max = (
        PROFILE_DEFAULTS[profile]
    )
    width = config.get(CONF_OUTPUT_WIDTH, width_default)
    height = config.get(CONF_OUTPUT_HEIGHT, height_default)
    quality = config.get(CONF_JPEG_QUALITY, quality_default)
    pixels = width * height
    if profile == PROFILE_LOW_MEMORY and pixels > 4096:
        raise cv.Invalid(
            "low_memory profile limits output_width * output_height to 4096 pixels"
        )
    if profile == PROFILE_BALANCED and not psram_guaranteed and pixels > 8192:
        raise cv.Invalid(
            "balanced profile without guaranteed PSRAM limits output size to 8192 pixels"
        )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await setup_entity(var, config, "camera")
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_idle_update_interval(config[CONF_IDLE_UPDATE_INTERVAL]))
    cg.add(var.set_jpeg_quality(quality))
    cg.add(var.set_output_size(width, height))
    cg.add(var.set_rotation(config[CONF_ROTATION]))
    cg.add(var.set_use_psram(psram_guaranteed and profile != PROFILE_LOW_MEMORY))
    cg.add(var.set_jpeg_buffer_sizes(jpeg_initial, jpeg_max))

    add_idf_component(name="espressif/esp32-camera", ref="2.1.5")
    cg.add_define("USE_CAMERA")
