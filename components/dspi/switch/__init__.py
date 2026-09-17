import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ICON, CONF_TYPE

from .. import DSPI_COMPONENT_SCHEMA, dspi_ns, register_dspi_child

DEPENDENCIES = ["dspi"]

DSPiSwitch = dspi_ns.class_("DSPiSwitch", switch.Switch, cg.Component)
ToggleTarget = dspi_ns.enum("ToggleTarget", is_class=True)

# One platform serving five unrelated controls, so `type` is required rather
# than defaulted -- the same reasoning as on the number and select platforms.
#
# These keys match protocol.h's ToggleTarget rather than the opcodes those map
# to: `eq_bypass` and `leveller` say what the parameter is, where the firmware's
# REQ_SET_BYPASS and REQ_SET_LEVELLER_ENABLE would only say where it came from.
# The opcode constants keep the firmware's spellings; that translation lives in
# protocol.h and stops there.
TARGETS = {
    "user_mute": ToggleTarget.USER_MUTE,
    "loudness": ToggleTarget.LOUDNESS,
    "eq_bypass": ToggleTarget.EQ_BYPASS,
    "crossfeed": ToggleTarget.CROSSFEED,
    "leveller": ToggleTarget.LEVELLER,
}

ICONS = {
    "user_mute": "mdi:volume-off",
    "loudness": "mdi:ear-hearing",
    "eq_bypass": "mdi:equalizer-outline",
    "crossfeed": "mdi:headphones",
    "leveller": "mdi:chart-bell-curve",
}


def _default_icon(config):
    """Pick an icon per type, without overriding one the user chose.

    switch_schema only installs an icon default when passed one, and the entity
    base schema leaves `icon` absent otherwise -- which is what makes this
    setdefault able to tell a user's icon from ours.
    """
    config.setdefault(CONF_ICON, ICONS[config[CONF_TYPE]])
    return config


CONFIG_SCHEMA = cv.All(
    switch.switch_schema(
        DSPiSwitch,
        # The DSPi is the source of truth for all of these. Restoring a cached
        # value on boot and pushing it to the device would overwrite whatever
        # the device actually is, so restoring is disabled outright rather than
        # merely defaulted off.
        default_restore_mode="DISABLED",
    )
    .extend({cv.Required(CONF_TYPE): cv.enum(TARGETS, lower=True)})
    .extend(DSPI_COMPONENT_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    _default_icon,
)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)
    cg.add(var.set_target(config[CONF_TYPE]))
    parent = await register_dspi_child(var, config)
    # Ask the hub to poll this parameter. Without it the entity would never
    # publish, since the refresh burst only reads what something has asked for.
    cg.add(parent.enable_toggle_read(config[CONF_TYPE]))
