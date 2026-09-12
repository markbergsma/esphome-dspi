#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

#include "../dspi.h"

namespace esphome {
namespace dspi {

// User mute.
//
// As with the volume number, state is published on confirmation rather than
// optimistically, so a mute toggled from elsewhere shows up here.
class DSPiMuteSwitch : public switch_::Switch, public Component, public DSPiStateListener {
 public:
  void set_parent(DSPiHub *parent) { parent_ = parent; }
  void dump_config() override;

  void on_dspi_state(const DSPiState &state) override {
    if (state.user_mute_valid && (!has_published_ || state.user_mute != last_published_)) {
      last_published_ = state.user_mute;
      has_published_ = true;
      this->publish_state(state.user_mute);
    }
  }

 protected:
  void write_state(bool value) override { parent_->set_user_mute(value); }

  DSPiHub *parent_{nullptr};
  bool last_published_{false};
  bool has_published_{false};
};

}  // namespace dspi
}  // namespace esphome
