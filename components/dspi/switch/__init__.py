import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch

from .. import DSPI_COMPONENT_SCHEMA, dspi_ns, register_dspi_child

DEPENDENCIES = ["dspi"]

DSPiMuteSwitch = dspi_ns.class_("DSPiMuteSwitch", switch.Switch, cg.Component)

CONFIG_SCHEMA = (
    switch.switch_schema(
        DSPiMuteSwitch,
        icon="mdi:volume-off",
        # The DSPi is the source of truth for its own mute state. Restoring a
        # cached value on boot and pushing it to the device would overwrite
        # whatever the device actually is, so restoring is disabled outright
        # rather than merely defaulted off.
        default_restore_mode="DISABLED",
    )
    .extend(DSPI_COMPONENT_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)
    await register_dspi_child(var, config)
