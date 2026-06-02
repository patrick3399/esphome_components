import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import light, web_server_base
from esphome.const import CONF_ID
from esphome.core import CORE

try:
    from esphome.components.esp32 import add_idf_sdkconfig_option
    _HAS_SDKCONFIG = True
except ImportError:
    _HAS_SDKCONFIG = False

DEPENDENCIES = ["light"]
AUTO_LOAD = ["web_server_base", "json"]

wled_bridge_ns = cg.esphome_ns.namespace("wled_bridge")
WLEDBridgeComponent = wled_bridge_ns.class_("WLEDBridgeComponent", cg.Component)

CONF_LIGHT_ID = "light_id"
CONF_BUSES = "buses"
CONF_MAX_MA = "max_ma"
CONF_LED_MA = "led_ma"
CONF_USE_TASK = "use_task"
CONF_AUTO_WHITE = "auto_white"
CONF_MATRIX_WIDTH = "matrix_width"
CONF_MATRIX_HEIGHT = "matrix_height"
CONF_MATRIX_SERPENTINE = "matrix_serpentine"
CONF_UDP_PORT = "udp_port"
CONF_UDP_PORT2 = "udp_port2"
CONF_UDP_SEND = "udp_send"
CONF_UDP_RECEIVE = "udp_receive"

# Auto-white modes for RGBW strips (derive W channel from RGB).
AUTO_WHITE_MODES = {
    "none": 0,
    "brighter": 1,
    "accurate": 2,
    "max": 4,
}

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
        # Global defaults used when light_id (not buses:) is specified.
        # Keep defaults in codegen so buses: configs don't show unused globals.
        cv.Optional(CONF_MAX_MA): cv.positive_int,
        cv.Optional(CONF_LED_MA): cv.positive_int,
        cv.Optional(CONF_USE_TASK, default=False): cv.boolean,
        cv.Optional(CONF_AUTO_WHITE, default="none"): cv.enum(
            AUTO_WHITE_MODES, lower=True
        ),
        # 2D matrix geometry — required for 2D effects
        cv.Optional(CONF_MATRIX_WIDTH, default=0): cv.int_range(min=0, max=256),
        cv.Optional(CONF_MATRIX_HEIGHT, default=0): cv.int_range(min=0, max=256),
        cv.Optional(CONF_MATRIX_SERPENTINE, default=False): cv.boolean,
        # UDP WLED notifier sync
        cv.Optional(CONF_UDP_PORT, default=21324): cv.port,
        cv.Optional(CONF_UDP_PORT2, default=65506): cv.port,
        cv.Optional(CONF_UDP_SEND, default=False): cv.boolean,
        cv.Optional(CONF_UDP_RECEIVE, default=False): cv.boolean,
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
    cg.add(var.set_auto_white_mode(config[CONF_AUTO_WHITE]))

    # 2D matrix
    cg.add(var.set_matrix_width(config[CONF_MATRIX_WIDTH]))
    cg.add(var.set_matrix_height(config[CONF_MATRIX_HEIGHT]))
    cg.add(var.set_matrix_serpentine(config[CONF_MATRIX_SERPENTINE]))

    # UDP sync
    cg.add(var.set_udp_port(config[CONF_UDP_PORT]))
    cg.add(var.set_udp_port2(config[CONF_UDP_PORT2]))
    cg.add(var.set_udp_send(config[CONF_UDP_SEND]))
    cg.add(var.set_udp_receive(config[CONF_UDP_RECEIVE]))

    # Enable IDF WebSocket support so /ws endpoint works
    if _HAS_SDKCONFIG and CORE.is_esp32:
        add_idf_sdkconfig_option("CONFIG_HTTPD_WS_SUPPORT", True)

    if CONF_BUSES in config:
        # Multi-bus path: call add_bus() for each entry in order.
        for bus_cfg in config[CONF_BUSES]:
            light_state = await cg.get_variable(bus_cfg[CONF_LIGHT_ID])
            cg.add(var.add_bus(light_state, bus_cfg[CONF_MAX_MA], bus_cfg[CONF_LED_MA]))
    else:
        # Single light_id backward-compat path: one bus, global max_ma/led_ma.
        light_state = await cg.get_variable(config[CONF_LIGHT_ID])
        cg.add(
            var.add_bus(
                light_state, config.get(CONF_MAX_MA, 5000), config.get(CONF_LED_MA, 55)
            )
        )
