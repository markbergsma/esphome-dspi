#pragma once

#include "esphome/core/automation.h"

#include "dspi.h"

namespace esphome {
namespace dspi {

// Fires once per new band frame, and once more on the edge into "not live" so
// a display can dim what it is showing rather than leaving a frozen spectrum
// that looks live.
//
// The hub compares each frame's sequence number against the last one it saw
// for that channel and only calls back when it has actually changed, so a
// consumer's redraw is driven by the device's frame rate rather than by our
// poll rate, with no bookkeeping of its own.
class RtaBandFrameTrigger : public Trigger<const RtaBandUpdate &> {
 public:
  explicit RtaBandFrameTrigger(DSPiHub *hub) {
    hub->add_on_rta_band_frame_callback([this](const RtaBandUpdate &update) { this->trigger(update); });
  }
};

}  // namespace dspi
}  // namespace esphome
