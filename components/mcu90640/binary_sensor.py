import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_HEAT, DEVICE_CLASS_OCCUPANCY

from . import CONF_MCU90640_ID, MCU90640BaseSchema

DEPENDENCIES = ["mcu90640"]

CONF_PRESENCE = "presence"
CONF_HOT_SPOT = "hot_spot"
CONF_THRESHOLD = "threshold"
CONF_MIN_PIXELS = "min_pixels"

_presence_schema = binary_sensor.binary_sensor_schema(
    device_class=DEVICE_CLASS_OCCUPANCY,
).extend(
    {
        cv.Optional(CONF_THRESHOLD, default=3.0): cv.float_range(
            min=0.5, max=20.0
        ),
        cv.Optional(CONF_MIN_PIXELS, default=5): cv.int_range(min=1, max=768),
    }
)

_hot_spot_schema = binary_sensor.binary_sensor_schema(
    device_class=DEVICE_CLASS_HEAT,
).extend(
    {
        cv.Optional(CONF_THRESHOLD, default=50.0): cv.float_range(
            min=20.0, max=300.0
        ),
    }
)

CONFIG_SCHEMA = MCU90640BaseSchema.extend(
    {
        cv.Optional(CONF_PRESENCE): _presence_schema,
        cv.Optional(CONF_HOT_SPOT): _hot_spot_schema,
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MCU90640_ID])

    if CONF_PRESENCE in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_PRESENCE])
        cg.add(hub.set_presence_sensor(bs))
        cg.add(
            hub.set_presence_threshold(config[CONF_PRESENCE][CONF_THRESHOLD])
        )
        cg.add(
            hub.set_presence_min_pixels(config[CONF_PRESENCE][CONF_MIN_PIXELS])
        )

    if CONF_HOT_SPOT in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_HOT_SPOT])
        cg.add(hub.set_hot_spot_sensor(bs))
        cg.add(
            hub.set_hot_spot_threshold(config[CONF_HOT_SPOT][CONF_THRESHOLD])
        )
