import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, time
from esphome import automation
from esphome.const import CONF_ID, CONF_ADDRESS

CODEOWNERS = ["@your_github_username"] # 請替換
DEPENDENCIES = ["i2c"]

rx8130ce_ns = cg.esphome_ns.namespace("rx8130ce")
RX8130CEComponent = rx8130ce_ns.class_(
    "RX8130CEComponent", time.RealTimeClock, i2c.I2CDevice, cg.Component
)

RX8130CEWriteTimeAction = rx8130ce_ns.class_("RX8130CEWriteTimeAction", automation.Action)
# 定義 ReadTime Action 類別
RX8130CEReadTimeAction = rx8130ce_ns.class_("RX8130CEReadTimeAction", automation.Action)

DEFAULT_I2C_ADDRESS = 0x32

CONFIG_SCHEMA = time.TIME_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(RX8130CEComponent),
    }
).extend(i2c.i2c_device_schema(DEFAULT_I2C_ADDRESS))


@automation.register_action(
    "rx8130ce.write_time",
    RX8130CEWriteTimeAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(RX8130CEComponent),
        }
    ),
)
async def rx8130ce_write_time_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

# 註冊 read_time action
@automation.register_action(
    "rx8130ce.read_time", # YAML 中使用的 action 名稱
    RX8130CEReadTimeAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(RX8130CEComponent), # 指定要操作的 RX8130CE ID
        }
    ),
)
async def rx8130ce_read_time_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    await time.register_time(var, config)
    return var