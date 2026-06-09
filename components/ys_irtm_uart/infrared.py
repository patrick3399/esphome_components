"""YS-IRTM UART platform for the infrared component (TX only, NEC protocol)."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import infrared

from . import YSIrtmUartComponent, ys_irtm_uart_ns

CODEOWNERS = ["@patrick3399"]
DEPENDENCIES = ["infrared"]

CONF_YS_IRTM_UART_ID = "ys_irtm_uart_id"

YsIrtmInfrared = ys_irtm_uart_ns.class_("YsIrtmInfrared", infrared.Infrared)

CONFIG_SCHEMA = infrared.infrared_schema(YsIrtmInfrared).extend(
    {
        cv.Required(CONF_YS_IRTM_UART_ID): cv.use_id(YSIrtmUartComponent),
    }
)


async def to_code(config):
    var = await infrared.new_infrared(config)
    ys = await cg.get_variable(config[CONF_YS_IRTM_UART_ID])
    cg.add(var.set_ys_irtm(ys))
