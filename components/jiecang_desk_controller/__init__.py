import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
CODEOWNERS = ["@patrick3399"]

jiecang_ns = cg.esphome_ns.namespace("jiecang_desk_controller")

JiecangDeskController = jiecang_ns.class_(
    "JiecangDeskController", cg.Component, uart.UARTDevice
)

CONF_JIECANG_DESK_CONTROLLER_ID = "jiecang_desk_controller_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(JiecangDeskController),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "jiecang_desk_controller",
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
