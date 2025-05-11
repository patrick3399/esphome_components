import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import display
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_LAMBDA,
)

ed047tc1_ns = cg.esphome_ns.namespace("ed047tc1")
ED047TC1Display = ed047tc1_ns.class_("ED047TC1Display", cg.Component, display.DisplayBuffer)

CONF_PWR_PIN = "pwr_pin"
CONF_BST_EN_PIN = "bst_en_pin"
CONF_XSTL_PIN = "xstl_pin"
CONF_PCLK_PIN = "pclk_pin"
CONF_XLE_PIN = "xle_pin"
CONF_SPV_PIN = "spv_pin"
CONF_CKV_PIN = "ckv_pin"
CONF_D0_PIN = "d0_pin"
CONF_D1_PIN = "d1_pin"
CONF_D2_PIN = "d2_pin"
CONF_D3_PIN = "d3_pin"
CONF_D4_PIN = "d4_pin"
CONF_D5_PIN = "d5_pin"
CONF_D6_PIN = "d6_pin"
CONF_D7_PIN = "d7_pin"

CONFIG_SCHEMA = display.FULL_DISPLAY_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(ED047TC1Display),
        cv.Required(CONF_PWR_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_BST_EN_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_XSTL_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_PCLK_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_XLE_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_SPV_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_CKV_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_D0_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_D1_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_D2_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_D3_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_D4_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_D5_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_D6_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_D7_PIN): pins.gpio_output_pin_schema,
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await display.register_display(var, config)

    pwr_pin = await cg.gpio_pin_expression(config[CONF_PWR_PIN])
    cg.add(var.set_pwr_pin(pwr_pin))
    bst_en_pin = await cg.gpio_pin_expression(config[CONF_BST_EN_PIN])
    cg.add(var.set_bst_en_pin(bst_en_pin))
    xstl_pin = await cg.gpio_pin_expression(config[CONF_XSTL_PIN])
    cg.add(var.set_xstl_pin(xstl_pin))
    if CONF_PCLK_PIN in config:
        pclk_pin = await cg.gpio_pin_expression(config[CONF_PCLK_PIN])
        cg.add(var.set_pclk_pin(pclk_pin))
    xle_pin = await cg.gpio_pin_expression(config[CONF_XLE_PIN])
    cg.add(var.set_xle_pin(xle_pin))
    spv_pin = await cg.gpio_pin_expression(config[CONF_SPV_PIN])
    cg.add(var.set_spv_pin(spv_pin))
    ckv_pin = await cg.gpio_pin_expression(config[CONF_CKV_PIN])
    cg.add(var.set_ckv_pin(ckv_pin))

    data_pins_args = []
    for i in range(8):
        pin = await cg.gpio_pin_expression(config[f"d{i}_pin"])
        data_pins_args.append(pin)
    cg.add(var.set_data_pins(cg.std_vector.template(cg.GPIOPin.operator("ptr"))(data_pins_args)))

    if CONF_LAMBDA in config:
        lambda_ = await cg.process_lambda(
            config[CONF_LAMBDA], [(display.DisplayRef, "it")], return_type=cg.void
        )
        cg.add(var.set_writer(lambda_))