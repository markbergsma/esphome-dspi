#pragma once

#include <vector>

#include "esphome/components/select/select.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "../dspi.h"

namespace esphome {
namespace dspi {

// Input source.
//
// The DSPi runs exactly one input source at a time -- the receivers share a
// memory arena and, for the four S/PDIF inputs, a PIO state machine -- so this
// is a source selector rather than a mixer control. The matrix mixer sits
// downstream and routes whichever channels the active source provides.
//
// Which sources a given device can select is narrower than the enum: ADAT and
// S/PDIF 2-4 ship disabled. The configured option list is therefore paired with
// an explicit list of wire values rather than relying on the option index, since
// the set is sparse and its order is the user's to choose.
class DSPiInputSourceSelect : public select::Select, public Component, public DSPiStateListener {
 public:
  void set_parent(DSPiHub *parent) { parent_ = parent; }
  // Wire values in the same order as the configured options.
  void set_source_values(std::vector<uint8_t> values) { source_values_ = std::move(values); }
  void dump_config() override;

  void on_dspi_state(const DSPiState &state) override {
    if (!state.input_source_valid)
      return;

    const int idx = index_for_source_(state.input_source);
    if (idx < 0) {
      // The device is on a source this select does not expose. Publishing
      // anything would misreport it, so leave the entity alone.
      ESP_LOGD("dspi.select", "DSPi is on input source %u, which this select does not list", state.input_source);
      return;
    }
    if (!has_published_ || state.input_source != last_published_) {
      last_published_ = state.input_source;
      has_published_ = true;
      this->publish_state(static_cast<size_t>(idx));
    }
  }

 protected:
  void control(size_t index) override {
    if (index < source_values_.size()) {
      parent_->set_input_source(source_values_[index]);
    }
  }

  // Sources the device can select that this entity does not list. Reported from
  // dump_config rather than once at probe time: the probe completes within a
  // second of boot, long before any log client can attach, so a one-shot
  // warning there is effectively invisible.
  std::vector<uint8_t> unlisted_selectable_() const {
    std::vector<uint8_t> out;
    const DSPiState &state = parent_->state();
    if (!state.selectable_valid)
      return out;
    for (uint8_t src = 0; src < INPUT_SOURCE_COUNT; src++) {
      if ((state.selectable_mask & (1u << src)) && index_for_source_(src) < 0)
        out.push_back(src);
    }
    return out;
  }

  int index_for_source_(uint8_t source) const {
    for (size_t i = 0; i < source_values_.size(); i++) {
      if (source_values_[i] == source)
        return static_cast<int>(i);
    }
    return -1;
  }

  DSPiHub *parent_{nullptr};
  std::vector<uint8_t> source_values_;
  uint8_t last_published_{0};
  bool has_published_{false};
};

// Preset slot.
//
// A DSPi preset is the complete DSP state -- EQ, crossovers, delays, the
// matrix, output configuration -- so this one entity reconfigures the whole
// processor.
//
// The option labels come from YAML rather than from the device, even though
// the device stores a name per slot. ESPHome sends a select's options to Home
// Assistant once, in the entity-list message at connect, and SelectTraits is
// documented as "set once at startup"; a name read asynchronously over UART
// arrives after that and would go stale again on every rename. The device's
// own name for the active slot is published by the preset_name text_sensor
// instead, where it can be updated honestly.
class DSPiPresetSelect : public select::Select, public Component, public DSPiStateListener {
 public:
  void set_parent(DSPiHub *parent) { parent_ = parent; }
  // Slot numbers in the same order as the configured options. As with the
  // input source, the option index is deliberately not the wire value: the
  // configured set is sparse and its order is the user's.
  void set_slot_values(std::vector<uint8_t> values) { slot_values_ = std::move(values); }
  void dump_config() override;

  void on_dspi_state(const DSPiState &state) override {
    if (!state.active_preset_valid)
      return;

    const int idx = index_for_slot_(state.active_preset);
    if (idx < 0) {
      // The device is on a slot this select does not list, which is normal:
      // presets can also be loaded from DSPi Console or a control surface.
      // Publishing anything would misreport it, so leave the entity alone.
      ESP_LOGD("dspi.select", "DSPi is on preset slot %u, which this select does not list", state.active_preset);
      return;
    }
    if (!has_published_ || state.active_preset != last_published_) {
      last_published_ = state.active_preset;
      has_published_ = true;
      this->publish_state(static_cast<size_t>(idx));
    }
  }

 protected:
  void control(size_t index) override {
    if (index < slot_values_.size()) {
      parent_->set_preset_slot(slot_values_[index]);
    }
  }

  int index_for_slot_(uint8_t slot) const {
    for (size_t i = 0; i < slot_values_.size(); i++) {
      if (slot_values_[i] == slot)
        return static_cast<int>(i);
    }
    return -1;
  }

  DSPiHub *parent_{nullptr};
  std::vector<uint8_t> slot_values_;
  uint8_t last_published_{0};
  bool has_published_{false};
};

}  // namespace dspi
}  // namespace esphome
