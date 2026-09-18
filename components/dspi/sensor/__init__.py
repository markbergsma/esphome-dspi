import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_TYPE,
    DEVICE_CLASS_FREQUENCY,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_HERTZ,
)

from .. import DSPI_COMPONENT_SCHEMA, dspi_ns, register_dspi_child

DEPENDENCIES = ["dspi"]

DSPiSensor = dspi_ns.class_("DSPiSensor", sensor.Sensor, cg.PollingComponent)
SensorTarget = dspi_ns.enum("SensorTarget", is_class=True)

# `pipeline_rate` is the rate the DSP and every output are actually running at,
# named for the firmware's own description of the field (audio_state.freq, the
# "current pipeline Hz" half of REQ_GET_INPUT_RATE). The DSPi does no sample
# rate conversion, so input, DSP and outputs all share this one rate, and it
# follows the active source rather than anything configured on a controller:
# select USB and it becomes whatever the host asked for.
TARGETS = {
    "pipeline_rate": SensorTarget.PIPELINE_RATE,
}

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        DSPiSensor,
        unit_of_measurement=UNIT_HERTZ,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_FREQUENCY,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TARGETS, lower=True),
        }
    )
    .extend(DSPI_COMPONENT_SCHEMA)
    # Polled, not notified. Nothing is raised when only the rate changes, so
    # the alternative to asking is showing a stale number as though it were
    # current. 5s is cheap -- one 8-byte read, and identical queued reads
    # coalesce -- and bounds how long a wrong reading can survive.
    .extend(cv.polling_component_schema("5s"))
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    cg.add(var.set_target(config[CONF_TYPE]))
    await register_dspi_child(var, config)
