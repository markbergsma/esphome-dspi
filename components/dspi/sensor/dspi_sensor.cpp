#include "dspi_sensor.h"

#include "esphome/core/log.h"

namespace esphome {
namespace dspi {

static const char *const TAG = "dspi.sensor";

void DSPiSensor::dump_config() {
  switch (target_) {
    case SensorTarget::PIPELINE_RATE:
      LOG_SENSOR("", "DSPi Pipeline Sample Rate", this);
      break;
  }
}

}  // namespace dspi
}  // namespace esphome
