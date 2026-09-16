import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_TYPE

from .. import DSPI_COMPONENT_SCHEMA, INPUT_SOURCES, dspi_ns, register_dspi_child

DEPENDENCIES = ["dspi"]

CONF_SOURCES = "sources"
CONF_SLOTS = "slots"

DSPiInputSourceSelect = dspi_ns.class_(
    "DSPiInputSourceSelect", select.Select, cg.Component
)
DSPiPresetSelect = dspi_ns.class_("DSPiPresetSelect", select.Select, cg.Component)

# PRESET_SLOTS in the firmware's config.h.
PRESET_SLOTS = 10

# Default labels. The key is the protocol's own source name; the value is only
# what Home Assistant displays, so it is free to be prettier.
DEFAULT_LABELS = {
    "usb": "USB",
    "spdif": "S/PDIF",
    "i2s": "I2S",
    "adat": "ADAT",
    "spdif2": "S/PDIF 2",
    "spdif3": "S/PDIF 3",
    "spdif4": "S/PDIF 4",
}

# Which sources to expose, in order. The default is the set every DSPi can
# always select: ADAT and S/PDIF 2-4 ship disabled and have to be turned on over
# USB first, so listing them by default would offer options that do nothing.
# The hub queries the device at boot and logs what is actually selectable, and
# warns if that set and this list disagree in either direction.
DEFAULT_SOURCES = ["usb", "spdif", "i2s"]


def _validate_sources(config):
    seen = set()
    for key in config[CONF_SOURCES]:
        if key in seen:
            raise cv.Invalid(f"input source '{key}' is listed more than once")
        seen.add(key)
    labels = [config.get(k, DEFAULT_LABELS[k]) for k in config[CONF_SOURCES]]
    if len(set(labels)) != len(labels):
        raise cv.Invalid("input source labels must be unique")
    return config


def _validate_slots(config):
    labels = list(config[CONF_SLOTS].values())
    if len(set(labels)) != len(labels):
        raise cv.Invalid("preset labels must be unique")
    return config


INPUT_SOURCE_SCHEMA = (
    select.select_schema(DSPiInputSourceSelect)
    .extend(
        {
            cv.Optional(CONF_SOURCES, default=DEFAULT_SOURCES): cv.All(
                cv.ensure_list(cv.one_of(*INPUT_SOURCES, lower=True)),
                cv.Length(min=1),
            ),
            **{
                cv.Optional(key, default=label): cv.string_strict
                for key, label in DEFAULT_LABELS.items()
            },
        }
    )
    .extend(DSPI_COMPONENT_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
    .add_extra(_validate_sources)
)

# `slots:` is required rather than defaulting to all ten. A fresh DSPi names
# only slot 0 and marks none occupied, so a generated list would be ten
# indistinguishable entries -- and picking an unsaved one loads factory
# defaults, which is a surprising thing to offer by accident. Listing the
# presets actually in use is a one-time cost and makes the entity readable.
#
# Slot numbers are the mapping keys, so YAML order is the option order and a
# sparse set (0, 1, 3) needs no extra syntax. `python3 tools/dspi_setup.py`
# prints the device's own names to copy from.
PRESET_SCHEMA = (
    select.select_schema(DSPiPresetSelect)
    .extend(
        {
            cv.Required(CONF_SLOTS): cv.All(
                {cv.int_range(min=0, max=PRESET_SLOTS - 1): cv.string_strict},
                cv.Length(min=1),
            ),
        }
    )
    .extend(DSPI_COMPONENT_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
    .add_extra(_validate_slots)
)

# `type` is required, as it is on the number platform: one platform serving two
# unrelated controls should say which it is at the point of use rather than by
# omission.
CONFIG_SCHEMA = cv.typed_schema(
    {
        "input_source": INPUT_SOURCE_SCHEMA,
        "preset": PRESET_SCHEMA,
    },
    key=CONF_TYPE,
    lower=True,
)


async def to_code(config):
    if config[CONF_TYPE] == "preset":
        # Parallel lists in configured order, as below: the index is not the
        # slot number, since the set may be sparse.
        slots = list(config[CONF_SLOTS].keys())
        options = [config[CONF_SLOTS][slot] for slot in slots]

        var = await select.new_select(config, options=options)
        await cg.register_component(var, config)
        cg.add(var.set_slot_values(slots))
        await register_dspi_child(var, config)
        return

    # Options and wire values are built as parallel lists in the configured
    # order. The index is deliberately NOT used as the wire value: the set is
    # sparse once the optional sources are involved, and the order is the user's.
    keys = config[CONF_SOURCES]
    options = [config[key] for key in keys]
    values = [INPUT_SOURCES[key] for key in keys]

    var = await select.new_select(config, options=options)
    await cg.register_component(var, config)
    cg.add(var.set_source_values(values))
    await register_dspi_child(var, config)
