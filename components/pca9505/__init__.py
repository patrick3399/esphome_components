from esphome import pins
import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_INVERTED,
    CONF_MODE,
    CONF_NUMBER,
    CONF_OUTPUT,
)

DEPENDENCIES = ["i2c"]
MULTI_CONF = True

pca9505_ns = cg.esphome_ns.namespace("pca9505")

PCA9505Component = pca9505_ns.class_("PCA9505Component", cg.Component, i2c.I2CDevice)
PCA9505GPIOPin = pca9505_ns.class_("PCA9505GPIOPin", cg.GPIOPin)

CONF_PCA9505 = "pca9505"

# Default I2C address for PCA9505 can range from 0x20 to 0x27
# based on A0, A1, A2 pins.
# Using 0x20 as a common default if A0-A2 are LOW.
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.declare_id(PCA9505Component),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x20)) # Default I2C address
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)


def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    return value


PCA9505_PIN_SCHEMA = pins.gpio_base_schema(
    PCA9505GPIOPin,
    cv.int_range(min=0, max=39), # PCA9505 has 40 pins (0-39)
    modes=[CONF_INPUT, CONF_OUTPUT],
    mode_validator=validate_mode,
    invertable=True,
).extend(
    {
        cv.Required(CONF_PCA9505): cv.use_id(PCA9505Component),
    }
)


@pins.PIN_SCHEMA_REGISTRY.register(CONF_PCA9505, PCA9505_PIN_SCHEMA)
async def pca9505_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_PCA9505])

    cg.add(var.set_parent(parent))

    num = config[CONF_NUMBER]
    cg.add(var.set_pin(num))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    return var