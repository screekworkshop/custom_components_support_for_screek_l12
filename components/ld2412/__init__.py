import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID
from esphome import automation
from esphome.automation import maybe_simple_id

# This library is heavily based on the ld2410 library by @sebcaps.
# Many thanks to @sebcaps for the foundational work.

DEPENDENCIES = ["uart"]
CODEOWNERS = ["@sebcaps", "@screek"]
MULTI_CONF = True

ld2412_ns = cg.esphome_ns.namespace("ld2412")
LD2412Component = ld2412_ns.class_("LD2412Component", cg.Component, uart.UARTDevice)
LD2412Restart = ld2412_ns.class_("LD2412Restart", automation.Action)
CONF_LD2412_ID = "ld2412_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(LD2412Component),
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2412",
    baud_rate=115200,
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

CALIBRATION_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(LD2412Component),
    }
)