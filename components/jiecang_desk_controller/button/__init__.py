import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import CONF_ID

from .. import (
    CONF_JIECANG_DESK_CONTROLLER_ID,
    JiecangDeskController,
    jiecang_ns,
)

DEPENDENCIES = ["jiecang_desk_controller"]

JiecangDeskButton = jiecang_ns.class_(
    "JiecangDeskButton",
    button.Button,
    cg.Parented.template(JiecangDeskController),
)

_BTN_STEP_UP = 0
_BTN_STEP_DOWN = 1
_BTN_STOP = 2
_BTN_MOVE_UP = 3
_BTN_MOVE_DOWN = 4
_BTN_GOTO_M1 = 5
_BTN_GOTO_M2 = 6
_BTN_GOTO_M3 = 7
_BTN_GOTO_M4 = 8
_BTN_SAVE_M1 = 9
_BTN_SAVE_M2 = 10
_BTN_SAVE_M3 = 11
_BTN_SAVE_M4 = 12

CONF_STEP_UP = "step_up"
CONF_STEP_DOWN = "step_down"
CONF_STOP = "stop"
CONF_MOVE_UP = "move_up"
CONF_MOVE_DOWN = "move_down"
CONF_GOTO_M1 = "goto_m1"
CONF_GOTO_M2 = "goto_m2"
CONF_GOTO_M3 = "goto_m3"
CONF_GOTO_M4 = "goto_m4"
CONF_SAVE_M1 = "save_m1"
CONF_SAVE_M2 = "save_m2"
CONF_SAVE_M3 = "save_m3"
CONF_SAVE_M4 = "save_m4"

_BUTTON_SCHEMA = button.button_schema(JiecangDeskButton)

_BUTTON_MAP = {
    CONF_STEP_UP: _BTN_STEP_UP,
    CONF_STEP_DOWN: _BTN_STEP_DOWN,
    CONF_STOP: _BTN_STOP,
    CONF_MOVE_UP: _BTN_MOVE_UP,
    CONF_MOVE_DOWN: _BTN_MOVE_DOWN,
    CONF_GOTO_M1: _BTN_GOTO_M1,
    CONF_GOTO_M2: _BTN_GOTO_M2,
    CONF_GOTO_M3: _BTN_GOTO_M3,
    CONF_GOTO_M4: _BTN_GOTO_M4,
    CONF_SAVE_M1: _BTN_SAVE_M1,
    CONF_SAVE_M2: _BTN_SAVE_M2,
    CONF_SAVE_M3: _BTN_SAVE_M3,
    CONF_SAVE_M4: _BTN_SAVE_M4,
}

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_ID): cv.declare_id(cg.EntityBase),
    cv.GenerateID(CONF_JIECANG_DESK_CONTROLLER_ID): cv.use_id(
        JiecangDeskController
    ),
    **{cv.Optional(k): _BUTTON_SCHEMA for k in _BUTTON_MAP},
}


async def to_code(config):
    for key, action in _BUTTON_MAP.items():
        if key in config:
            btn = await button.new_button(config[key])
            await cg.register_parented(
                btn, config[CONF_JIECANG_DESK_CONTROLLER_ID]
            )
            cg.add(btn.set_action(action))
