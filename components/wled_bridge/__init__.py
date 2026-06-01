import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import light, web_server_base
from esphome.const import CONF_ID
from esphome.core import CORE

DEPENDENCIES = ["light"]
AUTO_LOAD = ["web_server_base", "json"]

wled_bridge_ns = cg.esphome_ns.namespace("wled_bridge")
WLEDBridgeComponent = wled_bridge_ns.class_("WLEDBridgeComponent", cg.Component)

CONF_LIGHT_ID = "light_id"
CONF_MAX_MA = "max_ma"
CONF_LED_MA = "led_ma"
CONF_USE_TASK = "use_task"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WLEDBridgeComponent),
        cv.Required(CONF_LIGHT_ID): cv.use_id(light.AddressableLightState),
        cv.Optional(CONF_MAX_MA, default=5000): cv.positive_int,
        cv.Optional(CONF_LED_MA, default=55): cv.positive_int,
        cv.Optional(CONF_USE_TASK, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    light_state = await cg.get_variable(config[CONF_LIGHT_ID])
    cg.add(var.set_light_state(light_state))
    cg.add(var.set_max_ma(config[CONF_MAX_MA]))
    cg.add(var.set_led_ma(config[CONF_LED_MA]))
    cg.add(var.set_use_task(config[CONF_USE_TASK]))
