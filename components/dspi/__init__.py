"""ESPHome component for controlling a DSPi audio DSP over its UART control interface.

The DSPi firmware exposes one command surface over USB, UART and I2C. This
component speaks the UART transport, which is the only one of the three that can
push asynchronous notifications, so state changes made anywhere (a USB host, a
knob on a DSPi control surface) reach Home Assistant without polling.

The component depends on nothing but `uart`: it is useful on any ESPHome device,
not only the Sendspin player it was first written for.
"""

from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_INTERVAL, CONF_TRIGGER_ID

CODEOWNERS = ["@markbergsma"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True

dspi_ns = cg.esphome_ns.namespace("dspi")
DSPiHub = dspi_ns.class_("DSPiHub", cg.Component, uart.UARTDevice)
RtaConfigStruct = dspi_ns.struct("RtaConfig")
RtaBandUpdateConstRef = dspi_ns.struct("RtaBandUpdate").operator("ref").operator("const")
RtaBandFrameTrigger = dspi_ns.class_(
    "RtaBandFrameTrigger", automation.Trigger.template(RtaBandUpdateConstRef)
)

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

# Spectrum analyser. Keys mirror the firmware's RtaConfig field names so this
# block can be read against rta.h without translation.
CONF_RTA = "rta"
CONF_TAP = "tap"
CONF_CHANNEL_MASK = "channel_mask"
CONF_FFT_ORDER = "fft_order"
CONF_AVG_MS = "avg_ms"
CONF_PEAK_DECAY_DB_S = "peak_decay_db_s"
CONF_STALE_TIMEOUT = "stale_timeout"
CONF_STATUS_INTERVAL = "status_interval"
CONF_AUTO_ENABLE = "auto_enable"
CONF_ON_RTA_BAND_FRAME = "on_rta_band_frame"

# RTA_TAP_* from rta.h. The input tap sits after the per-input PEQ and before
# the matrix, so it is independent of master volume; the output tap is what the
# slots actually transmit, and therefore follows the volume control.
RTA_TAPS = {
    "input": 0,
    "output": 1,
}

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

# The analyser is transient on the device: off at boot, never persisted, and it
# switches itself off five seconds after the last band read. So there is nothing
# to restore and nothing to clean up -- polling is what keeps it alive.
#
# Refresh is `interval` x the number of channels in `channel_mask`, since one
# FFT engine rotates through them. The device itself publishes a frame every
# ~21 ms per channel at 1024 points / 48 kHz, so polling faster than that only
# re-reads a frame already seen.
#
# `interval` is also baud-sensitive: a band frame is 89 bytes on the wire, which
# is ~2 ms at 460800 but ~8 ms at 115200, where a fast interval would take a
# noticeable share of the link away from notifications and control commands.
RTA_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_TAP, default="input"): cv.enum(RTA_TAPS, lower=True),
        # Zero is rejected by the device rather than meaning "none", so it is
        # rejected here too, where the error can name the key.
        cv.Optional(CONF_CHANNEL_MASK, default=0x01): cv.int_range(
            min=1, max=0xFFFF
        ),
        # 8..10 -> 256..1024 points. Checked again against the device's own
        # reported range, which is narrower on RP2040.
        cv.Optional(CONF_FFT_ORDER, default=10): cv.int_range(min=8, max=10),
        # Plain integers, not time periods: the unit is already in the name,
        # and the firmware clamps these rather than rejecting them.
        cv.Optional(CONF_AVG_MS, default=250): cv.int_range(min=0, max=10000),
        cv.Optional(CONF_PEAK_DECAY_DB_S, default=20): cv.int_range(min=0, max=100),
        cv.Optional(
            CONF_INTERVAL, default="50ms"
        ): cv.positive_time_period_milliseconds,
        # How old a frame may be before the display is told the spectrum is no
        # longer live. Generous next to the device's frame time, so ordinary
        # jitter does not read as a dropout.
        cv.Optional(
            CONF_STALE_TIMEOUT, default="500ms"
        ): cv.positive_time_period_milliseconds,
        # Cheap insurance against a device that rebooted and lost our config:
        # its own defaults still produce frames, just from the wrong tap.
        cv.Optional(
            CONF_STATUS_INTERVAL, default="2s"
        ): cv.positive_time_period_milliseconds,
        # Off by default: a spectrum nobody is looking at costs the DSPi real
        # FFT and filter-bank time. A display should enable it when its page is
        # shown instead.
        cv.Optional(CONF_AUTO_ENABLE, default=False): cv.boolean,
    }
)

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
            # An 89-byte band-frame response plus a notification has to drain
            # in a single pass, or a frame sits half-parsed while its own
            # timeout runs down.
            cv.Optional(CONF_MAX_BYTES_PER_LOOP, default=128): cv.int_range(
                min=16, max=256
            ),
            cv.Optional(CONF_EXPECT_NOTIFICATIONS, default=True): cv.boolean,
            cv.Optional(CONF_BOOT_INPUT_SOURCE): cv.enum(INPUT_SOURCES, lower=True),
            cv.Optional(CONF_RTA): RTA_SCHEMA,
            cv.Optional(CONF_ON_RTA_BAND_FRAME): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        RtaBandFrameTrigger
                    ),
                }
            ),
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

    if rta := config.get(CONF_RTA):
        # Field order matches RtaConfig's declaration, less the reserved bytes
        # the serialiser zeroes and the flags field, which is always 0: its one
        # defined bit disables the device's idle timeout, and losing that would
        # leave the analyser running forever after a crash.
        cfg = cg.StructInitializer(
            RtaConfigStruct,
            ("version", 3),
            ("tap", rta[CONF_TAP]),
            ("channel_mask", rta[CONF_CHANNEL_MASK]),
            ("fft_order", rta[CONF_FFT_ORDER]),
            ("avg_ms", rta[CONF_AVG_MS]),
            ("peak_decay_db_s", rta[CONF_PEAK_DECAY_DB_S]),
            ("flags", 0),
        )
        cg.add(var.set_rta_config(cfg))
        cg.add(var.set_rta_interval(rta[CONF_INTERVAL]))
        cg.add(var.set_rta_stale_timeout(rta[CONF_STALE_TIMEOUT]))
        cg.add(var.set_rta_status_interval(rta[CONF_STATUS_INTERVAL]))
        cg.add(var.set_rta_auto_enable(rta[CONF_AUTO_ENABLE]))

    for conf in config.get(CONF_ON_RTA_BAND_FRAME, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trigger, [(RtaBandUpdateConstRef, "x")], conf
        )
