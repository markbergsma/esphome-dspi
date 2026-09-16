import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_TYPE

from .. import DSPI_COMPONENT_SCHEMA, dspi_ns, register_dspi_child

DEPENDENCIES = ["dspi"]

DSPiTextSensor = dspi_ns.class_("DSPiTextSensor", text_sensor.TextSensor, cg.Component)
TextTarget = dspi_ns.enum("TextTarget", is_class=True)

# `preset_name` is the device's own name for the slot it is currently on, read
# from its preset directory. It is the live counterpart to the fixed labels a
# preset `select` carries: those are chosen in YAML and cannot change at
# runtime, this follows the device however the preset was loaded.
TARGETS = {
    "preset_name": TextTarget.PRESET_NAME,
}

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(DSPiTextSensor)
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TARGETS, lower=True),
        }
    )
    .extend(DSPI_COMPONENT_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)
    cg.add(var.set_target(config[CONF_TYPE]))
    await register_dspi_child(var, config)
