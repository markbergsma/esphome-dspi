"""ESPHome component for controlling a DSPi audio DSP over its UART control interface.

The DSPi firmware exposes one command surface over USB, UART and I2C. This
component speaks the UART transport, which is the only one of the three that can
push asynchronous notifications, so state changes made anywhere (a USB host, a
knob on a DSPi control surface) reach Home Assistant without polling.

The component depends on nothing but `uart`: it is useful on any ESPHome device,
not only the Sendspin player it was first written for.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@markbergsma"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True

dspi_ns = cg.esphome_ns.namespace("dspi")
DSPiHub = dspi_ns.class_("DSPiHub", cg.Component, uart.UARTDevice)

CONF_DSPI_ID = "dspi_id"
CONF_REQUEST_TIMEOUT = "request_timeout"
CONF_FLASH_TIMEOUT = "flash_timeout"
CONF_MAX_RETRIES = "max_retries"
CONF_BACKOFF_BASE = "backoff_base"
CONF_POLL_INTERVAL = "poll_interval"
CONF_REFRESH_DEBOUNCE = "refresh_debounce"
CONF_MAX_BYTES_PER_LOOP = "max_bytes_per_loop"
CONF_EXPECT_NOTIFICATIONS = "expect_notifications"
CONF_BOOT_INPUT_SOURCE = "boot_input_source"

# Wire values of the DSPi InputSource enum. Exactly one source runs at a time;
# ADAT and S/PDIF 2-4 ship disabled and must be enabled over USB before they can
# be selected. The select platform pairs its options with these values
# explicitly rather than using the option index, since the usable set is sparse.
INPUT_SOURCES = {
    "usb": 0,
    "spdif": 1,
    "i2s": 2,
    "adat": 3,
    "spdif2": 4,
    "spdif3": 5,
    "spdif4": 6,
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DSPiHub),
            cv.Optional(
                CONF_REQUEST_TIMEOUT, default="400ms"
            ): cv.positive_time_period_milliseconds,
            # Commands that write flash leave the device deaf for roughly 45 ms
            # with interrupts off, so they are given a much longer budget.
            cv.Optional(
                CONF_FLASH_TIMEOUT, default="1500ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MAX_RETRIES, default=4): cv.int_range(min=0, max=10),
            cv.Optional(
                CONF_BACKOFF_BASE, default="60ms"
            ): cv.positive_time_period_milliseconds,
            # Zero means "never poll", which is correct when the device pushes
            # notifications. The hub warns and tells you to raise this if it
            # discovers notifications are switched off.
            cv.Optional(
                CONF_POLL_INTERVAL, default="0s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_REFRESH_DEBOUNCE, default="250ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MAX_BYTES_PER_LOOP, default=64): cv.int_range(
                min=16, max=256
            ),
            cv.Optional(CONF_EXPECT_NOTIFICATIONS, default=True): cv.boolean,
            cv.Optional(CONF_BOOT_INPUT_SOURCE): cv.enum(INPUT_SOURCES, lower=True),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

# The DSPi UART transport is fixed 8N1 (its CRC16 covers integrity end to end,
# so parity would add nothing). Pinning it here turns a mismatched `parity:`
# into a config error rather than a link that silently never works.
FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "dspi",
    require_tx=True,
    require_rx=True,
    data_bits=8,
    parity="NONE",
    stop_bits=1,
)

# Child platforms extend this to get a reference to their hub.
DSPI_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_DSPI_ID): cv.use_id(DSPiHub),
    }
)


async def register_dspi_child(var, config):
    """Wire a child entity to its hub and register it as a state listener."""
    parent = await cg.get_variable(config[CONF_DSPI_ID])
    cg.add(var.set_parent(parent))
    cg.add(parent.register_listener(var))
    return parent


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_request_timeout(config[CONF_REQUEST_TIMEOUT]))
    cg.add(var.set_flash_timeout(config[CONF_FLASH_TIMEOUT]))
    cg.add(var.set_max_retries(config[CONF_MAX_RETRIES]))
    cg.add(var.set_backoff_base(config[CONF_BACKOFF_BASE]))
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))
    cg.add(var.set_refresh_debounce(config[CONF_REFRESH_DEBOUNCE]))
    cg.add(var.set_max_bytes_per_loop(config[CONF_MAX_BYTES_PER_LOOP]))
    cg.add(var.set_expect_notifications(config[CONF_EXPECT_NOTIFICATIONS]))
    if CONF_BOOT_INPUT_SOURCE in config:
        cg.add(var.set_boot_input_source(config[CONF_BOOT_INPUT_SOURCE]))
