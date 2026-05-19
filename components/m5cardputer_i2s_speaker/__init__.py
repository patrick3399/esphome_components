import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32"]
CODEOWNERS = ["@patrick3399"]

m5cardputer_i2s_speaker_ns = cg.esphome_ns.namespace("m5cardputer_i2s_speaker")
M5CardputerI2SSpeaker = m5cardputer_i2s_speaker_ns.class_(
    "M5CardputerI2SSpeaker", cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(M5CardputerI2SSpeaker),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    esp32.include_builtin_idf_component("esp_driver_i2s")
