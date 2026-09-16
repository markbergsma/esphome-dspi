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

void DSPiPresetSelect::dump_config() {
  LOG_SELECT(TAG, "DSPi Preset", this);
  const DSPiState &state = this->parent_->state();
  if (!state.preset_dir_valid)
    return;
  // A saved slot this select omits is not an error -- most systems use a
  // handful of the ten -- but it is the likely explanation if the entity ever
  // looks stuck, so it is worth naming at config level rather than in a debug
  // log nobody will be watching when it happens.
  for (uint8_t slot = 0; slot < PRESET_SLOTS; slot++) {
    if (((state.slot_occupied >> slot) & 1u) && this->index_for_slot_(slot) < 0) {
      ESP_LOGCONFIG(TAG, "  DSPi slot %u holds a saved preset but is not in this select's `slots:` list", slot);
    }
  }
}

}  // namespace dspi
}  // namespace esphome
