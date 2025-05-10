from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_INVERTED,
    CONF_MODE,
    CONF_NUMBER,
    CONF_OUTPUT,
    CONF_UPDATE_INTERVAL, # 為輪詢加入
)

DEPENDENCIES = ["i2c"]
MULTI_CONF = True

aw9523_ns = cg.esphome_ns.namespace("aw9523")

AW9523Component = aw9523_ns.class_("AW9523Component", cg.PollingComponent, i2c.I2CDevice) # 繼承自 PollingComponent
AW9523GPIOPin = aw9523_ns.class_("AW9523GPIOPin", cg.GPIOPin)

# AW9523 組件本身的 Schema
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.declare_id(AW9523Component),
        }
    )
    .extend(cv.polling_component_schema('60s'))  # 預設更新間隔 60 秒
    .extend(i2c.i2c_device_schema(0x59)) # 根據您的 YAML，預設 AW9523 I2C 位址為 0x59
)

# 驗證 pin 模式 (輸入或輸出，不能兩者皆是)
def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    return value

CONF_AW9523 = "aw9523" # 定義父組件的鍵名

# AW9523 個別 GPIO pin 的 Schema
AW9523_PIN_SCHEMA = pins.gpio_base_schema(
    AW9523GPIOPin,
    cv.int_range(min=0, max=15), # AW9523 有 16 個 GPIO (0-15)
    modes=[CONF_INPUT, CONF_OUTPUT],
    mode_validator=validate_mode,
    invertable=True,
).extend(
    {
        cv.Required(CONF_AW9523): cv.use_id(AW9523Component),
    }
)

# 註冊組件和 I2C 設備
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

# 個別 pin 的程式碼生成
@pins.PIN_SCHEMA_REGISTRY.register(CONF_AW9523, AW9523_PIN_SCHEMA)
async def aw9523_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_AW9523])
    cg.add(var.set_parent(parent))

    num = config[CONF_NUMBER]
    cg.add(var.set_pin(num))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    return var