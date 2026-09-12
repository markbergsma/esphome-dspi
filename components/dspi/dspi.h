#pragma once

#include <array>
#include <string>
#include <vector>

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

#include "framing.h"
#include "protocol.h"

namespace esphome {
namespace dspi {

// The DSPi state this component tracks and republishes.
//
// Every field is authoritative only once its `*_valid` flag is set: before the
// first successful read we genuinely do not know the device's state, and
// publishing a default would be a lie that entities would then echo back.
struct DSPiState {
  // The output ceiling: a configuration setting, not a listening control.
  float master_volume_db{0.0f};
  bool master_volume_valid{false};
  // The listening control, and the same field the USB host's slider drives.
  float user_volume_db{0.0f};
  bool user_volume_valid{false};
  bool user_mute{false};
  bool user_mute_valid{false};
  uint8_t input_source{INPUT_SOURCE_USB};
  bool input_source_valid{false};
  // Bit N set means source N can actually be selected on this device. ADAT and
  // S/PDIF 2-4 ship disabled, so a device's real source list is narrower than
  // the enum and is only knowable by asking. Valid once the probe has run.
  uint8_t selectable_mask{0};
  bool selectable_valid{false};
};

// Implemented by the child entity platforms.  An abstract listener rather than
// direct pointers to number::Number / switch_::Switch / select::Select keeps
// this header free of those components' headers, so the hub still compiles on
// a device that configures none of them.
class DSPiStateListener {
 public:
  virtual void on_dspi_state(const DSPiState &state) = 0;
};

class DSPiHub;

// One request awaiting transmission or a response.
struct Transaction {
  uint8_t req{0};
  // FRAME_SET_REQ or FRAME_GET_REQ.  A property of the opcode, not of the
  // verb: DSPi dispatches some mutating commands on its GET path.  Always set
  // this from the opcode's own definition.
  uint8_t frame_type{FRAME_GET_REQ};
  uint16_t wvalue{0};
  uint16_t wlen{0};
  uint8_t payload[MAX_TX_PAYLOAD]{};
  uint8_t payload_len{0};
  uint8_t attempts{0};
  // 0 = user-initiated, 1 = background refresh.  Lower runs first, so a
  // volume change never queues behind a refresh burst.
  uint8_t priority{1};
  uint32_t timeout_ms{0};
  void (DSPiHub::*on_ok)(const uint8_t *data, uint16_t len){nullptr};
  bool in_use{false};
};

class DSPiHub : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  // Configuration, set from codegen.
  void set_request_timeout(uint32_t ms) { request_timeout_ms_ = ms; }
  void set_flash_timeout(uint32_t ms) { flash_timeout_ms_ = ms; }
  void set_max_retries(uint8_t n) { max_retries_ = n; }
  void set_backoff_base(uint32_t ms) { backoff_base_ms_ = ms; }
  void set_poll_interval(uint32_t ms) { poll_interval_ms_ = ms; }
  void set_refresh_debounce(uint32_t ms) { refresh_debounce_ms_ = ms; }
  void set_max_bytes_per_loop(uint16_t n) { max_bytes_per_loop_ = n; }
  void set_expect_notifications(bool b) { expect_notifications_ = b; }
  void set_boot_input_source(uint8_t src) {
    boot_input_source_ = src;
    has_boot_input_source_ = true;
  }

  void register_listener(DSPiStateListener *l) { listeners_.push_back(l); }

  // Control surface, usable from a lambda with no entity configured.
  void set_master_volume_db(float db);
  void set_user_volume_db(float db);
  void set_user_mute(bool mute);
  void set_input_source(uint8_t source);
  // Re-read everything we publish.  Debounced, so a burst of notifications
  // costs one round of reads rather than one per event.
  void request_refresh();

  const DSPiState &state() const { return state_; }
  // False only when the probe has run and the device said this source is not
  // available. Unknown sources read as selectable: the device arbitrates, and
  // refusing locally on incomplete information would be worse than asking.
  bool is_source_selectable(uint8_t source) const {
    return !state_.selectable_valid || (state_.selectable_mask & (1u << source)) != 0;
  }
  bool is_online() const { return hub_state_ == HubState::IDLE || hub_state_ == HubState::AWAIT_RESP; }

 protected:
  enum class HubState : uint8_t {
    INIT,        // before setup()
    PROBING,     // identifying the device and checking its interface config
    IDLE,        // link up, nothing in flight
    AWAIT_RESP,  // exactly one request outstanding
    BACKOFF,     // waiting out a retry delay
    OFFLINE,     // repeated failures; reprobing slowly
  };

  // --- queue ---------------------------------------------------------------
  static constexpr uint8_t QUEUE_DEPTH = 8;
  std::array<Transaction, QUEUE_DEPTH> queue_{};

  bool enqueue_(const Transaction &txn);
  int find_next_() const;
  void enqueue_get_(uint8_t req, uint16_t wlen, void (DSPiHub::*cb)(const uint8_t *, uint16_t), uint8_t priority = 1);

  // --- transport -----------------------------------------------------------
  void pump_rx_();
  void handle_frame_();
  void handle_response_(uint8_t type, uint8_t status, const uint8_t *payload, uint16_t len);
  void handle_notification_(const uint8_t *packet, uint16_t len);
  void send_next_();
  void fail_current_(const char *reason);
  void retry_or_drop_(const char *reason);
  void go_offline_();

  // --- probe chain ---------------------------------------------------------
  void start_probe_();
  void on_platform_(const uint8_t *data, uint16_t len);
  void on_uart_config_(const uint8_t *data, uint16_t len);
  void on_iface_status_(const uint8_t *data, uint16_t len);
  void on_spdif_input_config_(const uint8_t *data, uint16_t len);
  void on_adat_enable_(const uint8_t *data, uint16_t len);
  void on_adat_pin_(const uint8_t *data, uint16_t len);
  void log_selectable_sources_();
  std::string selectable_sources_string_() const;

  // --- readback handlers ---------------------------------------------------
  void on_master_volume_(const uint8_t *data, uint16_t len);
  void on_user_volume_(const uint8_t *data, uint16_t len);
  void on_user_mute_(const uint8_t *data, uint16_t len);
  void on_input_source_(const uint8_t *data, uint16_t len);
  void publish_state_();

  FrameParser parser_;
  HubState hub_state_{HubState::INIT};
  Transaction current_{};
  bool has_current_{false};
  uint32_t sent_at_{0};
  uint32_t backoff_until_{0};
  uint32_t last_probe_at_{0};
  uint32_t last_poll_at_{0};
  uint32_t refresh_due_at_{0};
  bool refresh_pending_{false};
  uint8_t consecutive_failures_{0};

  // Notification bookkeeping.  A gap in seq means the device dropped events
  // for us, which is the only signal available that we have missed something.
  uint8_t last_seq_{0};
  bool have_seq_{false};
  bool notifications_active_{false};
  bool warned_no_notify_{false};

  DSPiState state_{};
  std::vector<DSPiStateListener *> listeners_;

  // Configuration.
  uint32_t request_timeout_ms_{400};
  uint32_t flash_timeout_ms_{1500};
  uint32_t backoff_base_ms_{60};
  uint32_t poll_interval_ms_{0};
  uint32_t refresh_debounce_ms_{250};
  uint16_t max_bytes_per_loop_{64};
  uint8_t max_retries_{4};
  bool expect_notifications_{true};
  uint8_t boot_input_source_{INPUT_SOURCE_I2S};
  bool has_boot_input_source_{false};
  bool boot_source_checked_{false};

  // Diagnostics, surfaced by dump_config().
  uint32_t rx_bytes_{0};
  uint32_t frames_bad_crc_{0};
  uint32_t frames_malformed_{0};
  uint32_t txn_dropped_{0};
  uint32_t txn_timeouts_{0};
  uint32_t notify_gaps_{0};
  uint32_t notify_count_{0};

  // Device identity, logged once the probe succeeds.
  uint8_t fw_major_{0}, fw_minor_{0}, fw_patch_{0}, platform_id_{0};
  bool identified_{false};

  // ADAT needs both an enable and a pin before it is selectable, and they
  // arrive in separate responses, so both are tracked until the pair is known.
  bool adat_enabled_{false};
  bool adat_pin_set_{false};
  uint8_t logged_selectable_mask_{0};
  bool logged_selectable_{false};
};

}  // namespace dspi
}  // namespace esphome
