from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import key_provider
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_ON_KEY, CONF_TRIGGER_ID

AUTO_LOAD = ["key_provider"]
MULTI_CONF = True

hc138_keypad_ns = cg.esphome_ns.namespace("hc138_keypad")
HC138Keypad = hc138_keypad_ns.class_("HC138Keypad", key_provider.KeyProvider, cg.Component)
HC138KeyTrigger = hc138_keypad_ns.class_(
    "HC138KeyTrigger", automation.Trigger.template(cg.uint8)
)

CONF_ADDRESS_PINS = "address_pins"
CONF_INPUT_PINS = "input_pins"
CONF_DEBOUNCE_TIME = "debounce_time"


CONFIG_SCHEMA = cv.COMPONENT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(HC138Keypad),
        cv.Required(CONF_ADDRESS_PINS): cv.All(
            cv.ensure_list({cv.Required("pin"): pins.gpio_output_pin_schema}),
            cv.Length(min=3, max=3),
        ),
        cv.Required(CONF_INPUT_PINS): cv.All(
            cv.ensure_list({cv.Required("pin"): pins.gpio_input_pin_schema}),
            cv.Length(min=7, max=7),
        ),
        cv.Optional(CONF_DEBOUNCE_TIME, default="5ms"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_ON_KEY): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(HC138KeyTrigger),
            }
        ),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    address_pins = []
    for conf in config[CONF_ADDRESS_PINS]:
        pin = await cg.gpio_pin_expression(conf["pin"])
        address_pins.append(pin)
    cg.add(var.set_address_pins(address_pins))

    input_pins = []
    for conf in config[CONF_INPUT_PINS]:
        pin = await cg.gpio_pin_expression(conf["pin"])
        input_pins.append(pin)
    cg.add(var.set_input_pins(input_pins))

    cg.add(var.set_debounce_time(config[CONF_DEBOUNCE_TIME]))

    for conf in config.get(CONF_ON_KEY, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_key_trigger(trigger))
        await automation.build_automation(trigger, [(cg.uint8, "x")], conf)
