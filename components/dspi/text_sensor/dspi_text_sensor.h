#pragma once

#include <string>

#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

#include "../dspi.h"

namespace esphome {
namespace dspi {

// Which piece of device-provided text this entity publishes.
enum class TextTarget : uint8_t {
  PRESET_NAME,  // the DSPi's own name for the slot it is currently on
};

// A string the DSPi reports about itself.
//
// Unlike a select's options, which ESPHome fixes at startup, a text sensor's
// state is an ordinary runtime string. That is what makes this the right home
// for a preset name: it follows the device however the preset was loaded --
// from Home Assistant, from DSPi Console, or from a knob on the DSPi itself.
class DSPiTextSensor : public text_sensor::TextSensor, public Component, public DSPiStateListener {
 public:
  void set_parent(DSPiHub *parent) { parent_ = parent; }
  void set_target(TextTarget target) { target_ = target; }
  void dump_config() override;

  void on_dspi_state(const DSPiState &state) override {
    if (target_ != TextTarget::PRESET_NAME)
      return;
    if (!state.active_preset_valid || !state.active_preset_name_valid)
      return;

    // A slot that has never been named comes back as an empty string -- on a
    // fresh device that is true of every slot but 0. Publishing "" would leave
    // a blank entity that looks broken rather than unnamed, so fall back to
    // the slot number, which is at least true and identifies the slot.
    std::string value = state.active_preset_name;
    if (value.empty())
      value = "Slot " + std::to_string(static_cast<unsigned>(state.active_preset));

    if (!has_published_ || value != last_published_) {
      last_published_ = value;
      has_published_ = true;
      this->publish_state(value);
    }
  }

 protected:
  DSPiHub *parent_{nullptr};
  TextTarget target_{TextTarget::PRESET_NAME};
  std::string last_published_;
  bool has_published_{false};
};

}  // namespace dspi
}  // namespace esphome
