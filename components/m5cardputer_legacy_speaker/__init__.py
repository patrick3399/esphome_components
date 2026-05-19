import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
from esphome.const import CONF_ID

CODEOWNERS = ["@patrick3399"]

m5cardputer_legacy_speaker_ns = cg.esphome_ns.namespace("m5cardputer_legacy_speaker")
M5CardputerLegacySpeaker = m5cardputer_legacy_speaker_ns.class_(
    "M5CardputerLegacySpeaker", cg.Component
)

CONFIG_SCHEMA = cv.COMPONENT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(M5CardputerLegacySpeaker),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # ESPHome 2026.x excludes unused ESP-IDF components by default. The legacy
    # M5Stack speaker path uses the deprecated i2s_driver_install/i2s_write API.
    esp32.include_builtin_idf_component("driver")
    esp32.include_builtin_idf_component("esp_driver_i2s")
