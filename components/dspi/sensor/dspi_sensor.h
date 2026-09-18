#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#include "../dspi.h"

namespace esphome {
namespace dspi {

// Which numeric value the DSPi reports about itself this entity publishes.
enum class SensorTarget : uint8_t {
  PIPELINE_RATE,  // the rate the DSP and every output are running at
};

// A number the DSPi reports about itself.
//
// The pipeline rate is worth surfacing because it is the one audio parameter
// nothing in a controller's own config determines. The DSPi has no sample rate
// conversion, so one rate runs the active input, the DSP and every output at
// once, and it is whatever the active source dictates -- an I2S master's clock
// while I2S is selected, the USB host's choice the moment USB is. A config
// that clocks I2S at 44.1 kHz is therefore not evidence the device is running
// at 44.1 kHz; this entity is.
class DSPiSensor : public sensor::Sensor, public Component, public DSPiStateListener {
 public:
  void set_parent(DSPiHub *parent) { parent_ = parent; }
  void set_target(SensorTarget target) { target_ = target; }
  void dump_config() override;

  void on_dspi_state(const DSPiState &state) override {
    if (target_ != SensorTarget::PIPELINE_RATE)
      return;
    if (!state.pipeline_rate_valid)
      return;

    // Republishing an unchanged rate on every refresh would add a Home
    // Assistant state write per poll for a value that moves a few times a day.
    const uint32_t value = state.pipeline_rate_hz;
    if (has_published_ && value == last_published_)
      return;
    last_published_ = value;
    has_published_ = true;
    this->publish_state(static_cast<float>(value));
  }

 protected:
  DSPiHub *parent_{nullptr};
  SensorTarget target_{SensorTarget::PIPELINE_RATE};
  uint32_t last_published_{0};
  bool has_published_{false};
};

}  // namespace dspi
}  // namespace esphome
