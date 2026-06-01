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
CONF_BUSES = "buses"
CONF_MAX_MA = "max_ma"
CONF_LED_MA = "led_ma"
CONF_USE_TASK = "use_task"

# Per-bus config block
BUS_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_LIGHT_ID): cv.use_id(light.AddressableLightState),
        cv.Optional(CONF_MAX_MA, default=5000): cv.positive_int,
        cv.Optional(CONF_LED_MA, default=55): cv.positive_int,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WLEDBridgeComponent),
        # Accept either a single light_id (backward compat) or a buses: list.
        cv.Optional(CONF_LIGHT_ID): cv.use_id(light.AddressableLightState),
        cv.Optional(CONF_BUSES): cv.All(cv.ensure_list(BUS_SCHEMA), cv.Length(min=1)),
        # Global defaults used when light_id (not buses:) is specified
        cv.Optional(CONF_MAX_MA, default=5000): cv.positive_int,
        cv.Optional(CONF_LED_MA, default=55): cv.positive_int,
        cv.Optional(CONF_USE_TASK, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


def _validate(config):
    if CONF_LIGHT_ID not in config and CONF_BUSES not in config:
        raise cv.Invalid("Either 'light_id' or 'buses' must be specified")
    if CONF_LIGHT_ID in config and CONF_BUSES in config:
        raise cv.Invalid("'light_id' and 'buses' are mutually exclusive")
    return config


CONFIG_SCHEMA = cv.All(CONFIG_SCHEMA, _validate)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_use_task(config[CONF_USE_TASK]))

    if CONF_BUSES in config:
        # Multi-bus path: call add_bus() for each entry in order.
        for bus_cfg in config[CONF_BUSES]:
            light_state = await cg.get_variable(bus_cfg[CONF_LIGHT_ID])
            cg.add(var.add_bus(light_state, bus_cfg[CONF_MAX_MA], bus_cfg[CONF_LED_MA]))
    else:
        # Single light_id backward-compat path: one bus, global max_ma/led_ma.
        light_state = await cg.get_variable(config[CONF_LIGHT_ID])
        cg.add(var.add_bus(light_state, config[CONF_MAX_MA], config[CONF_LED_MA]))
