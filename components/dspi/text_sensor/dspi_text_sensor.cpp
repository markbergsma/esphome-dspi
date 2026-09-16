#include "dspi_text_sensor.h"

#include "esphome/core/log.h"

namespace esphome {
namespace dspi {

static const char *const TAG = "dspi.text_sensor";

void DSPiTextSensor::dump_config() {
  switch (target_) {
    case TextTarget::PRESET_NAME:
      LOG_TEXT_SENSOR("", "DSPi Preset Name", this);
      break;
  }
}

}  // namespace dspi
}  // namespace esphome
