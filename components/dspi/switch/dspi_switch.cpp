#include "dspi_switch.h"

#include "esphome/core/log.h"

namespace esphome {
namespace dspi {

static const char *const TAG = "dspi.switch";

void DSPiMuteSwitch::dump_config() { LOG_SWITCH(TAG, "DSPi Mute", this); }

}  // namespace dspi
}  // namespace esphome
