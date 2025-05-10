import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_OUTPUT,
    CONF_INVERTED,
    CONF_MODE,
    CONF_NUMBER,
    CONF_UPDATE_INTERVAL, # Added
)

DEPENDENCIES = ["i2c"]
MULTI_CONF = True

pi4ioe5v6408_ns = cg.esphome_ns.namespace("pi4ioe5v6408")
PI4IOE5V6408 = pi4ioe5v6408_ns.class_("PI4IOE5V6408", cg.PollingComponent, i2c.I2CDevice)
PI4IOE5V6408GPIOPin = pi4ioe5v6408_ns.class_("PI4IOE5V6408GPIOPin", cg.GPIOPin)

# Configuration schema for the main component
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.declare_id(PI4IOE5V6408),
            # Remove CONF_UPDATE_INTERVAL from here if inheriting PollingComponent's default
        }
    )
    .extend(cv.polling_component_schema("60s")) # Inherit polling component schema, default 60s
    .extend(i2c.i2c_device_schema(0x58)) # Default I2C address from your YAML
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    # PollingComponent handles update_interval automatically if CONF_UPDATE_INTERVAL is in its schema

def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    return value

CONF_PI4IOE5V6408 = "pi4ioe5v6408"
# Schema for individual pins
PI4IOE5V6408_PIN_SCHEMA = pins.gpio_base_schema(
    PI4IOE5V6408GPIOPin,
    cv.int_range(min=0, max=7), # PI4IOE5V6408 has 8 pins (0-7)
    modes=[CONF_INPUT, CONF_OUTPUT],
    mode_validator=validate_mode,
    invertable=True,
).extend(
    {
        cv.Required(CONF_PI4IOE5V6408): cv.use_id(PI4IOE5V6408),
    }
)

@pins.PIN_SCHEMA_REGISTRY.register(CONF_PI4IOE5V6408, PI4IOE5V6408_PIN_SCHEMA)
async def pi4ioe5v6408_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_PI4IOE5V6408])

    cg.add(var.set_parent(parent))
    cg.add(var.set_pin(config[CONF_NUMBER]))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    return var