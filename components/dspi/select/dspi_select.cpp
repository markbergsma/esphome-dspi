#include "dspi_select.h"

#include "esphome/core/log.h"

namespace esphome {
namespace dspi {

static const char *const TAG = "dspi.select";

void DSPiInputSourceSelect::dump_config() {
  LOG_SELECT(TAG, "DSPi Input Source", this);
  // A source the device offers but this entity omits would leave the select
  // looking stuck whenever the DSPi sat on it, so say so plainly.
  for (uint8_t src : this->unlisted_selectable_()) {
    ESP_LOGW(TAG, "  DSPi can select input source %u, but it is not in this select's `sources:` list", src);
  }
}

}  // namespace dspi
}  // namespace esphome
