import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import CONF_ID, CONF_PIN

from .. import aw9523b_ns, CONF_AW9523B, AW9523BComponent

DEPENDENCIES = ["aw9523b"]

AW9523BFloatOutput = aw9523b_ns.class_("AW9523BFloatOutput", output.FloatOutput, cg.Component)

CONFIG_SCHEMA = output.FLOAT_OUTPUT_SCHEMA.extend(
    {
        cv.Required(CONF_ID): cv.declare_id(AW9523BFloatOutput),
        cv.Required(CONF_AW9523B): cv.use_id(AW9523BComponent),
        cv.Required(CONF_PIN): cv.int_range(min=0, max=15),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_AW9523B])
    cg.add(var.set_parent(parent))
    await output.register_output(var, config)
    await cg.register_component(var, config)
    cg.add(var.set_pin(config[CONF_PIN]))
