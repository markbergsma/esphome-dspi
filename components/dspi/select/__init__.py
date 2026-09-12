import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select

from .. import DSPI_COMPONENT_SCHEMA, INPUT_SOURCES, dspi_ns, register_dspi_child

DEPENDENCIES = ["dspi"]

CONF_SOURCES = "sources"

DSPiInputSourceSelect = dspi_ns.class_(
    "DSPiInputSourceSelect", select.Select, cg.Component
)

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


def _validate(config):
    seen = set()
    for key in config[CONF_SOURCES]:
        if key in seen:
            raise cv.Invalid(f"input source '{key}' is listed more than once")
        seen.add(key)
    labels = [config.get(k, DEFAULT_LABELS[k]) for k in config[CONF_SOURCES]]
    if len(set(labels)) != len(labels):
        raise cv.Invalid("input source labels must be unique")
    return config


CONFIG_SCHEMA = cv.All(
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
    .extend(cv.COMPONENT_SCHEMA),
    _validate,
)


async def to_code(config):
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
