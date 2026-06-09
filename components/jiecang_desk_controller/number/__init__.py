import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import CONF_HEIGHT, CONF_ID

from .. import (
    CONF_JIECANG_DESK_CONTROLLER_ID,
    JiecangDeskController,
    jiecang_ns,
)

DEPENDENCIES = ["jiecang_desk_controller"]

CONF_HEIGHT_PCT = "height_pct"

JiecangDeskNumber = jiecang_ns.class_(
    "JiecangDeskNumber",
    number.Number,
    cg.Parented.template(JiecangDeskController),
)

_NUM_HEIGHT_CM = 0
_NUM_HEIGHT_PCT = 1

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
        cv.GenerateID(CONF_JIECANG_DESK_CONTROLLER_ID): cv.use_id(
            JiecangDeskController
        ),
        cv.Optional(CONF_HEIGHT): number.number_schema(
            JiecangDeskNumber
        ),
        cv.Optional(CONF_HEIGHT_PCT): number.number_schema(
            JiecangDeskNumber
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(
        config[CONF_JIECANG_DESK_CONTROLLER_ID]
    )
    if CONF_HEIGHT in config:
        num = await number.new_number(
            config[CONF_HEIGHT],
            min_value=0,
            max_value=300,
            step=0.1,
        )
        await cg.register_parented(
            num, config[CONF_JIECANG_DESK_CONTROLLER_ID]
        )
        cg.add(num.set_type(_NUM_HEIGHT_CM))
        cg.add(hub.set_height_number(num))

    if CONF_HEIGHT_PCT in config:
        num = await number.new_number(
            config[CONF_HEIGHT_PCT],
            min_value=0,
            max_value=100,
            step=1,
        )
        await cg.register_parented(
            num, config[CONF_JIECANG_DESK_CONTROLLER_ID]
        )
        cg.add(num.set_type(_NUM_HEIGHT_PCT))
        cg.add(hub.set_height_pct_number(num))
