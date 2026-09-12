#include "dspi_number.h"

#include "esphome/core/log.h"

namespace esphome {
namespace dspi {

static const char *const TAG = "dspi.number";

void DSPiVolumeNumber::dump_config() {
  LOG_NUMBER(TAG, target_ == VolumeTarget::USER ? "DSPi User Volume" : "DSPi Master Volume", this);
}

}  // namespace dspi
}  // namespace esphome
