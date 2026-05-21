import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart
from esphome.const import (
    CONF_ID,
    CONF_TRIGGER_ID,
    CONF_DATA,
    CONF_PROTOCOL,
    CONF_ADDRESS,
    CONF_COMMAND,
    CONF_BAUD_RATE,
)

CONF_REPEATS = "repeats"
CONF_USER_CODE_HI = "user_code_hi"
CONF_USER_CODE_LO = "user_code_lo"
CONF_KEY_CODE = "key_code"
CONF_ON_IR_RECEIVE = "on_ir_receive"

BAUD_RATE_OPTIONS = {
    4800: 1,
    9600: 2,
    19200: 3,
    57600: 4,
}

DEPENDENCIES = ["uart"]

ys_irtm_uart_ns = cg.esphome_ns.namespace("ys_irtm_uart")

YSIrtmUartComponent = ys_irtm_uart_ns.class_("YSIrtmUartComponent", cg.Component, uart.UARTDevice)
SendRawAction = ys_irtm_uart_ns.class_("SendRawAction", automation.Action)
SendNecAction = ys_irtm_uart_ns.class_("SendNecAction", automation.Action)
SendProxyPacketAction = ys_irtm_uart_ns.class_("SendProxyPacketAction", automation.Action)
SetModuleAddressAction = ys_irtm_uart_ns.class_("SetModuleAddressAction", automation.Action)
SetBaudrateAction = ys_irtm_uart_ns.class_("SetBaudrateAction", automation.Action)
IrReceiveTrigger = ys_irtm_uart_ns.class_(
    "IrReceiveTrigger", automation.Trigger.template(cg.uint8, cg.uint8, cg.uint8)
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(YSIrtmUartComponent),
            cv.Optional(CONF_ON_IR_RECEIVE): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(IrReceiveTrigger)}
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    for conf in config.get(CONF_ON_IR_RECEIVE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trigger,
            [(cg.uint8, "user_code_hi"), (cg.uint8, "user_code_lo"), (cg.uint8, "key_code")],
            conf,
        )


@automation.register_action(
    "ys_irtm_uart.send_raw",
    SendRawAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(YSIrtmUartComponent),
            cv.Required(CONF_DATA): cv.templatable(cv.string),
        }
    ),
    synchronous=True,
)
async def ys_irtm_send_raw_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_data(await cg.templatable(config[CONF_DATA], args, cg.std_string)))
    return var


@automation.register_action(
    "ys_irtm_uart.send_nec",
    SendNecAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(YSIrtmUartComponent),
            cv.Required(CONF_USER_CODE_HI): cv.templatable(cv.uint8_t),
            cv.Required(CONF_USER_CODE_LO): cv.templatable(cv.uint8_t),
            cv.Required(CONF_KEY_CODE): cv.templatable(cv.uint8_t),
            cv.Optional(CONF_REPEATS, default=0): cv.templatable(cv.uint8_t),
        }
    ),
    synchronous=True,
)
async def ys_irtm_send_nec_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_user_code_hi(await cg.templatable(config[CONF_USER_CODE_HI], args, cg.uint8)))
    cg.add(var.set_user_code_lo(await cg.templatable(config[CONF_USER_CODE_LO], args, cg.uint8)))
    cg.add(var.set_key_code(await cg.templatable(config[CONF_KEY_CODE], args, cg.uint8)))
    cg.add(var.set_repeats(await cg.templatable(config[CONF_REPEATS], args, cg.uint8)))
    return var


@automation.register_action(
    "ys_irtm_uart.send_proxy_packet",
    SendProxyPacketAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(YSIrtmUartComponent),
            cv.Required(CONF_PROTOCOL): cv.templatable(cv.string),
            cv.Required(CONF_ADDRESS): cv.templatable(cv.uint32_t),
            cv.Required(CONF_COMMAND): cv.templatable(cv.uint32_t),
            cv.Optional(CONF_REPEATS, default=0): cv.templatable(cv.uint8_t),
        }
    ),
    synchronous=True,
)
async def ys_irtm_send_proxy_packet_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_protocol(await cg.templatable(config[CONF_PROTOCOL], args, cg.std_string)))
    cg.add(var.set_address(await cg.templatable(config[CONF_ADDRESS], args, cg.uint32)))
    cg.add(var.set_command(await cg.templatable(config[CONF_COMMAND], args, cg.uint32)))
    cg.add(var.set_repeats(await cg.templatable(config[CONF_REPEATS], args, cg.uint8)))
    return var


@automation.register_action(
    "ys_irtm_uart.set_address",
    SetModuleAddressAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(YSIrtmUartComponent),
            cv.Required(CONF_ADDRESS): cv.templatable(cv.uint8_t),
        }
    ),
    synchronous=True,
)
async def ys_irtm_set_address_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_address(await cg.templatable(config[CONF_ADDRESS], args, cg.uint8)))
    return var


@automation.register_action(
    "ys_irtm_uart.set_baudrate",
    SetBaudrateAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(YSIrtmUartComponent),
            cv.Required(CONF_BAUD_RATE): cv.one_of(4800, 9600, 19200, 57600, int=True),
        }
    ),
    synchronous=True,
)
async def ys_irtm_set_baudrate_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_baud_id(BAUD_RATE_OPTIONS[config[CONF_BAUD_RATE]]))
    return var
