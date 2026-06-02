from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import light, web_server_base
from esphome.const import (
    CONF_BLUE,
    CONF_BRIGHTNESS,
    CONF_GREEN,
    CONF_ID,
    CONF_RED,
    CONF_STATE,
    CONF_WHITE,
)
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

# Automation action classes
LoadPresetAction = wled_bridge_ns.class_("LoadPresetAction", automation.Action)
SavePresetAction = wled_bridge_ns.class_("SavePresetAction", automation.Action)
SetEffectAction = wled_bridge_ns.class_("SetEffectAction", automation.Action)
SetPaletteAction = wled_bridge_ns.class_("SetPaletteAction", automation.Action)
SetBrightnessAction = wled_bridge_ns.class_("SetBrightnessAction", automation.Action)
SetColorAction = wled_bridge_ns.class_("SetColorAction", automation.Action)
PowerAction = wled_bridge_ns.class_("PowerAction", automation.Action)
StopPlaylistAction = wled_bridge_ns.class_("StopPlaylistAction", automation.Action)

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
CONF_DDP_RECEIVE = "ddp_receive"

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
        # DDP realtime pixel receiver (Hyperion / Prismatik / xLights)
        cv.Optional(CONF_DDP_RECEIVE, default=False): cv.boolean,
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

    # DDP realtime receiver
    cg.add(var.set_ddp_enabled(config[CONF_DDP_RECEIVE]))

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


CONF_PRESET = "preset"
CONF_EFFECT = "effect"
CONF_PALETTE = "palette"


@automation.register_action(
    "wled_bridge.load_preset",
    LoadPresetAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_PRESET): cv.templatable(cv.int_range(min=1, max=16)),
        }
    ),
    synchronous=True,
)
async def wled_bridge_load_preset_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_PRESET], args, cg.uint8)
    cg.add(var.set_preset(template_))
    return var


@automation.register_action(
    "wled_bridge.save_preset",
    SavePresetAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_PRESET): cv.templatable(cv.int_range(min=1, max=16)),
        }
    ),
    synchronous=True,
)
async def wled_bridge_save_preset_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_PRESET], args, cg.uint8)
    cg.add(var.set_preset(template_))
    return var


@automation.register_action(
    "wled_bridge.set_effect",
    SetEffectAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_EFFECT): cv.templatable(cv.int_range(min=0, max=255)),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_effect_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_EFFECT], args, cg.uint8)
    cg.add(var.set_effect(template_))
    return var


@automation.register_action(
    "wled_bridge.set_palette",
    SetPaletteAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_PALETTE): cv.templatable(cv.int_range(min=0, max=255)),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_palette_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_PALETTE], args, cg.uint8)
    cg.add(var.set_palette(template_))
    return var


@automation.register_action(
    "wled_bridge.set_brightness",
    SetBrightnessAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_BRIGHTNESS): cv.templatable(
                cv.int_range(min=0, max=255)
            ),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_brightness_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_BRIGHTNESS], args, cg.uint8)
    cg.add(var.set_brightness(template_))
    return var


@automation.register_action(
    "wled_bridge.set_color",
    SetColorAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_RED): cv.templatable(cv.int_range(min=0, max=255)),
            cv.Required(CONF_GREEN): cv.templatable(cv.int_range(min=0, max=255)),
            cv.Required(CONF_BLUE): cv.templatable(cv.int_range(min=0, max=255)),
            cv.Optional(CONF_WHITE, default=0): cv.templatable(
                cv.int_range(min=0, max=255)
            ),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_color_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_red(await cg.templatable(config[CONF_RED], args, cg.uint8)))
    cg.add(var.set_green(await cg.templatable(config[CONF_GREEN], args, cg.uint8)))
    cg.add(var.set_blue(await cg.templatable(config[CONF_BLUE], args, cg.uint8)))
    cg.add(var.set_white(await cg.templatable(config[CONF_WHITE], args, cg.uint8)))
    return var


@automation.register_action(
    "wled_bridge.power_on",
    PowerAction,
    cv.Schema({cv.GenerateID(): cv.use_id(WLEDBridgeComponent)}),
    synchronous=True,
)
async def wled_bridge_power_on_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_state(True))
    return var


@automation.register_action(
    "wled_bridge.power_off",
    PowerAction,
    cv.Schema({cv.GenerateID(): cv.use_id(WLEDBridgeComponent)}),
    synchronous=True,
)
async def wled_bridge_power_off_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_state(False))
    return var


@automation.register_action(
    "wled_bridge.power_toggle",
    PowerAction,
    cv.Schema({cv.GenerateID(): cv.use_id(WLEDBridgeComponent)}),
    synchronous=True,
)
async def wled_bridge_power_toggle_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_toggle(True))
    return var


@automation.register_action(
    "wled_bridge.stop_playlist",
    StopPlaylistAction,
    cv.Schema({cv.GenerateID(): cv.use_id(WLEDBridgeComponent)}),
    synchronous=True,
)
async def wled_bridge_stop_playlist_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
