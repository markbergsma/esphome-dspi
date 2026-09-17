#include "dspi_switch.h"

#include "esphome/core/log.h"

namespace esphome {
namespace dspi {

static const char *const TAG = "dspi.switch";

// Named from the opcode table rather than from a literal, so adding a target
// cannot leave an entity logging under the wrong name.
//
// log_switch directly rather than through LOG_SWITCH: that macro wraps its type
// argument in LOG_STR_LITERAL, which is PSTR() on ESP8266 and so requires a
// compile-time literal.
void DSPiSwitch::dump_config() { switch_::log_switch(TAG, "", toggle_name(target_), this); }

}  // namespace dspi
}  // namespace esphome
