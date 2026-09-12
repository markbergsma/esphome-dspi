#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"

#include "../dspi.h"

namespace esphome {
namespace dspi {

// Which of the DSPi's two independent gain stages this entity drives. They are
// not interchangeable -- see the commentary in protocol.h.
enum class VolumeTarget : uint8_t {
  USER,    // the listening control; also drives loudness compensation
  MASTER,  // the output ceiling; a configuration setting
};

// A DSPi volume, in dB.
//
// The value is never published optimistically from control(): it is published
// when the device confirms it. That is what lets the entity track changes made
// from anywhere else, including a USB host or a knob wired to the DSPi itself.
class DSPiVolumeNumber : public number::Number, public Component, public DSPiStateListener {
 public:
  void set_parent(DSPiHub *parent) { parent_ = parent; }
  void set_target(VolumeTarget target) { target_ = target; }
  void dump_config() override;

  void on_dspi_state(const DSPiState &state) override {
    const bool valid = target_ == VolumeTarget::USER ? state.user_volume_valid : state.master_volume_valid;
    if (!valid)
      return;
    const float db = target_ == VolumeTarget::USER ? state.user_volume_db : state.master_volume_db;
    if (!has_published_ || db != last_published_) {
      last_published_ = db;
      has_published_ = true;
      this->publish_state(db);
    }
  }

 protected:
  void control(float value) override {
    if (target_ == VolumeTarget::USER) {
      parent_->set_user_volume_db(value);
    } else {
      parent_->set_master_volume_db(value);
    }
  }

  DSPiHub *parent_{nullptr};
  VolumeTarget target_{VolumeTarget::USER};
  float last_published_{0.0f};
  bool has_published_{false};
};

}  // namespace dspi
}  // namespace esphome
