import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ENTITY_CATEGORY,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_MODE,
    CONF_STEP,
    CONF_TYPE,
    ENTITY_CATEGORY_CONFIG,
    UNIT_DECIBEL,
)

from .. import DSPI_COMPONENT_SCHEMA, dspi_ns, register_dspi_child

DEPENDENCIES = ["dspi"]

DSPiVolumeNumber = dspi_ns.class_("DSPiVolumeNumber", number.Number, cg.Component)
VolumeTarget = dspi_ns.enum("VolumeTarget", is_class=True)

# The DSPi has two independent gain stages, and picking the wrong one is an easy
# mistake to make silently, so `type` is required rather than defaulted:
#
#   user_volume   what a listener turns. Same field as the USB host's volume
#                 slider, and the one that drives loudness compensation.
#   master_volume the device-side output ceiling. A configuration setting --
#                 "how loud may this system ever get" -- not a listening control.
TARGETS = {
    "user_volume": VolumeTarget.USER,
    "master_volume": VolumeTarget.MASTER,
}

# The firmware clamps host volume to [-60, 0]. Master volume accepts down to
# -127, but a slider spanning that is unusable since almost all of its travel is
# inaudible; -60 is a practical floor and the C++ side still clamps to the real
# range for anything a lambda sets.
MIN_DEFAULT = -60.0
MASTER_PROTOCOL_MIN = -127.0


def _defaults_by_type(config):
    """Master volume is configuration, so categorise it as such unless told otherwise."""
    if config[CONF_TYPE] == "master_volume":
        config.setdefault(CONF_ENTITY_CATEGORY, ENTITY_CATEGORY_CONFIG)
    return config


def _validate(config):
    if config[CONF_MIN_VALUE] >= config[CONF_MAX_VALUE]:
        raise cv.Invalid(
            f"min_value ({config[CONF_MIN_VALUE]}) must be below "
            f"max_value ({config[CONF_MAX_VALUE]})"
        )
    # Asking for -80 dB of host volume looks like it works and then silently
    # gives you -60, so reject it at config time instead.
    if config[CONF_TYPE] == "user_volume" and config[CONF_MIN_VALUE] < MIN_DEFAULT:
        raise cv.Invalid(
            f"user_volume is clamped by the firmware to [{MIN_DEFAULT}, 0] dB; "
            f"min_value {config[CONF_MIN_VALUE]} is below that. Use master_volume "
            f"if you need a deeper attenuator."
        )
    return config


CONFIG_SCHEMA = cv.All(
    number.number_schema(
        DSPiVolumeNumber,
        unit_of_measurement=UNIT_DECIBEL,
    )
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TARGETS, lower=True),
            cv.Optional(CONF_MIN_VALUE, default=MIN_DEFAULT): cv.float_range(
                min=MASTER_PROTOCOL_MIN, max=0.0
            ),
            cv.Optional(CONF_MAX_VALUE, default=0.0): cv.float_range(
                min=MASTER_PROTOCOL_MIN, max=0.0
            ),
            cv.Optional(CONF_STEP, default=0.5): cv.positive_not_null_float,
            cv.Optional(CONF_MODE, default="SLIDER"): cv.enum(
                number.NUMBER_MODES, upper=True
            ),
        }
    )
    .extend(DSPI_COMPONENT_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    _defaults_by_type,
    _validate,
)


async def to_code(config):
    var = await number.new_number(
        config,
        min_value=config[CONF_MIN_VALUE],
        max_value=config[CONF_MAX_VALUE],
        step=config[CONF_STEP],
    )
    await cg.register_component(var, config)
    cg.add(var.set_target(config[CONF_TYPE]))
    await register_dspi_child(var, config)
