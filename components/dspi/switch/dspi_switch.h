#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

#include "../dspi.h"

namespace esphome {
namespace dspi {

// One of the DSPi's boolean parameters: user mute, or a DSP feature toggle.
//
// Which one is a ToggleTarget rather than a subclass, because they differ only
// in the opcode pair the hub looks up. As with the volume number, state is
// published on confirmation rather than optimistically, so a value changed from
// elsewhere -- DSPi Console, a control surface, a preset load -- shows up here.
class DSPiSwitch : public switch_::Switch, public Component, public DSPiStateListener {
 public:
  void set_parent(DSPiHub *parent) { parent_ = parent; }
  void set_target(ToggleTarget target) { target_ = target; }
  void dump_config() override;

  void on_dspi_state(const DSPiState &state) override {
    if (!state.toggle_valid(target_))
      return;
    const bool value = state.toggle(target_);
    if (!has_published_ || value != last_published_) {
      last_published_ = value;
      has_published_ = true;
      this->publish_state(value);
    }
  }

 protected:
  void write_state(bool value) override { parent_->set_toggle(target_, value); }

  DSPiHub *parent_{nullptr};
  ToggleTarget target_{ToggleTarget::USER_MUTE};
  bool last_published_{false};
  bool has_published_{false};
};

}  // namespace dspi
}  // namespace esphome
