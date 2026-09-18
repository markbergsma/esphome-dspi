#pragma once

#include <array>
#include <functional>
#include <string>
#include <vector>

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "framing.h"
#include "protocol.h"

namespace esphome {
namespace dspi {

// The DSPi state this component tracks and republishes.
//
// Every field is authoritative only once its `*_valid` flag is set: before the
// first successful read we genuinely do not know the device's state, and
// publishing a default would be a lie that entities would then echo back.
//
// The boolean DSP parameters are the same rule expressed for a family rather
// than a field: `toggle_valid_mask` is a bitmask of the *_valid flags and
// `toggle_values` holds the values themselves. Reach both only through
// toggle() / toggle_valid(), never by masking directly.
struct DSPiState {
  // The output ceiling: a configuration setting, not a listening control.
  float master_volume_db{0.0f};
  bool master_volume_valid{false};
  // The listening control, and the same field the USB host's slider drives.
  float user_volume_db{0.0f};
  bool user_volume_valid{false};
  // The boolean DSP parameters, indexed by ToggleTarget. Mute lives here too:
  // it is the same 1-byte SET/GET shape as the rest, so giving it its own
  // named pair would only mean carrying two styles in one struct.
  uint16_t toggle_values{0};
  uint16_t toggle_valid_mask{0};
  uint8_t input_source{INPUT_SOURCE_USB};
  bool input_source_valid{false};
  // Bit N set means source N can actually be selected on this device. ADAT and
  // S/PDIF 2-4 ship disabled, so a device's real source list is narrower than
  // the enum and is only knowable by asking. Valid once the probe has run.
  uint8_t selectable_mask{0};
  bool selectable_valid{false};
  // The slot the device is actually on, which is not necessarily one this
  // config lists: presets can also be loaded from DSPi Console or a control
  // surface.
  uint8_t active_preset{0};
  bool active_preset_valid{false};
  // The device's own name for active_preset, read from its directory. Empty
  // when that slot has never been named. Distinct from the labels configured
  // in YAML, which are the user's and are fixed at compile time.
  std::string active_preset_name;
  bool active_preset_name_valid{false};
  // Bit N set = slot N holds user data. Read once at probe; only used for
  // reporting, since a slot being empty does not stop it being loaded.
  uint16_t slot_occupied{0};
  bool preset_dir_valid{false};
  // The rate the DSP pipeline and every output run at, read from
  // REQ_GET_INPUT_RATE. The DSPi does no sample rate conversion, so this is
  // the active input's rate too, and it follows the source rather than
  // whatever an I2S master happens to be clocking: selecting USB makes it the
  // host's rate. Zero until the first readback.
  uint32_t pipeline_rate_hz{0};
  bool pipeline_rate_valid{false};

  bool toggle(ToggleTarget t) const { return (toggle_values & toggle_bit(t)) != 0; }
  bool toggle_valid(ToggleTarget t) const { return (toggle_valid_mask & toggle_bit(t)) != 0; }
};

// Implemented by the child entity platforms.  An abstract listener rather than
// direct pointers to number::Number / switch_::Switch / select::Select keeps
// this header free of those components' headers, so the hub still compiles on
// a device that configures none of them.
class DSPiStateListener {
 public:
  virtual void on_dspi_state(const DSPiState &state) = 0;
};

// What a consumer is handed when a new band frame arrives.
//
// It carries the wire frame plus the little the device reports separately but
// a renderer cannot do without: the level zero point from RtaCaps, so nothing
// downstream hard-codes 243, and whether the data is actually live.
struct RtaBandUpdate {
  RtaBandFrame frame{};
  uint8_t level_zero{RTA_LEVEL_ZERO_DBFS};
  // False when the analyser is idle, the channel has never published, or the
  // newest frame has aged past the staleness window.  The frame still holds
  // the last good data, so a consumer can dim it rather than blanking it.
  bool live{false};

  float db(uint8_t v) const { return rta_level_to_dbfs(v, level_zero); }
  bool is_floor(uint8_t v) const { return rta_level_is_floor(v); }
  float fraction(uint8_t v, float floor_db, float top_db = 0.0f) const {
    return rta_level_to_fraction(v, level_zero, floor_db, top_db);
  }
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
  // 0 = user-initiated, 1 = background refresh, 2 = spectrum streaming.  Lower
  // runs first, so a volume change never queues behind a refresh burst, and a
  // spectrum poll is the first thing evicted when the queue fills.
  uint8_t priority{1};
  // Retries allowed for this transaction, or RETRIES_DEFAULT to use the hub's
  // max_retries_.  Band polls set 0: a frame retried after a backoff is stale
  // by the time it lands, and the next poll re-asks for a fresher one anyway.
  static constexpr uint8_t RETRIES_DEFAULT = 0xFF;
  uint8_t max_retries{RETRIES_DEFAULT};
  uint32_t timeout_ms{0};
  void (DSPiHub::*on_ok)(const uint8_t *data, uint16_t len){nullptr};
  // Called when this transaction ends without an OK: a permanent rejection, or
  // retries exhausted.  Without it a state machine that tracks "one request
  // outstanding" deadlocks the first time the device answers ERROR.
  void (DSPiHub::*on_fail)(uint8_t status){nullptr};
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

  // --- spectrum analyser (RTA) ---------------------------------------------
  //
  // Configured from codegen; no `rta:` block means set_rta_config() is never
  // called, rta_configured_ stays false, and the component never sends a single
  // RTA byte.  A wall knob pays nothing for a feature it does not use.
  void set_rta_config(const RtaConfig &cfg) {
    rta_cfg_desired_ = cfg;
    rta_configured_ = true;
  }
  void set_rta_interval(uint32_t ms) { rta_interval_ms_ = ms; }
  void set_rta_status_interval(uint32_t ms) { rta_status_interval_ms_ = ms; }
  void set_rta_stale_timeout(uint32_t ms) { rta_stale_ms_ = ms; }
  void set_rta_auto_enable(bool b) { rta_auto_enable_ = b; }

  // Start or stop polling.  This is what a display calls when its spectrum
  // page is shown or hidden.
  //
  // Disabling sends one best-effort RTA_CTL_STOP and stops polling; it does not
  // set RTA_FLAG_MANUAL.  With MANUAL the device's own idle timeout is
  // disabled, so if this component crashes, reboots, or loses the UART with the
  // page open, the analyser would run forever -- burning main-loop FFT time and
  // the per-channel bass bank, which is synchronous audio work on both of the
  // DSPi's cores.  Leaving auto-off in place makes our stop advisory: another
  // client's next read restarts the engine, and there is no run state to
  // reconcile after a device reboot.
  void set_rta_enabled(bool enabled);
  void rta_reset_avg();
  void set_rta_tap(uint8_t tap);
  void set_rta_channel_mask(uint16_t mask);

  void add_on_rta_band_frame_callback(std::function<void(const RtaBandUpdate &)> &&cb) {
    rta_band_frame_callback_.add(std::move(cb));
  }

  // True once caps have been read, the device reported protocol V3, and our
  // config was accepted.  False on a firmware with no RTA, or one speaking a
  // different protocol version.
  bool rta_available() const { return rta_caps_valid_ && rta_phase_ != RtaPhase::UNSUPPORTED; }
  bool rta_enabled() const { return rta_enabled_; }
  const RtaCaps &rta_caps() const { return rta_caps_; }
  const RtaConfig &rta_applied_config() const { return rta_cfg_applied_; }
  const RtaStatus &rta_status() const { return rta_status_; }
  // True when the analyser is running someone else's configuration. The DSPi
  // has one engine shared by every client, so this is normal rather than an
  // error: what we draw is then that client's tap, not ours.
  bool rta_foreign_config() const { return rta_foreign_config_; }
  const RtaBandUpdate &rta_last_band_frame() const { return rta_last_update_; }
  // Nominal third-octave centre in Hz, or 0 if the centre table could not be
  // read (which is non-fatal: axis labels degrade, the spectrum still works).
  uint16_t rta_band_centre_hz(uint8_t band) const {
    return band < RTA_MAX_BANDS ? rta_band_centre_hz_[band] : 0;
  }
  float rta_level_db(uint8_t v) const { return rta_level_to_dbfs(v, rta_caps_.level_zero); }

  // Control surface, usable from a lambda with no entity configured.
  void set_master_volume_db(float db);
  void set_user_volume_db(float db);
  void set_input_source(uint8_t source);
  // Set one of the boolean DSP parameters. Also starts reading it back, so this
  // works on a device that configures no switch for it at all -- otherwise the
  // value would be written and then never observed, leaving toggle_valid()
  // false forever.
  void set_toggle(ToggleTarget target, bool on);
  // Named wrappers, so a lambda reads as prose rather than as an enum lookup.
  void set_user_mute(bool mute) { set_toggle(ToggleTarget::USER_MUTE, mute); }
  void set_loudness(bool on) { set_toggle(ToggleTarget::LOUDNESS, on); }
  void set_eq_bypass(bool on) { set_toggle(ToggleTarget::EQ_BYPASS, on); }
  void set_crossfeed(bool on) { set_toggle(ToggleTarget::CROSSFEED, on); }
  void set_leveller(bool on) { set_toggle(ToggleTarget::LEVELLER, on); }
  // Include `target` in the refresh burst. Called from codegen for every
  // configured switch, so a device pays no UART traffic for a parameter it
  // does not expose.
  void enable_toggle_read(ToggleTarget target) { toggle_read_mask_ |= toggle_bit(target); }
  // Load a preset slot (0..9).  The device defers the work, so this returns
  // long before the preset is live and nothing is published until a readback
  // confirms the device actually moved.
  void set_preset_slot(uint8_t slot);
  // Re-read everything we publish.  Debounced, so a burst of notifications
  // costs one round of reads rather than one per event.
  void request_refresh();
  // Re-read just the pipeline rate.  Separate from request_refresh() because
  // the firmware raises no notification when only the rate changes -- a USB
  // host moving between 44.1 and 48 kHz says nothing -- so an entity showing
  // it has to poll, and polling the whole parameter set to get one value would
  // be wasteful. Identical queued reads coalesce, so calling this on a short
  // interval cannot build a backlog.
  void request_input_rate();

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

  // Where the RTA state machine is in bringing the analyser up.  It owns at
  // most one queued transaction at a time, which bounds its share of the queue
  // to one slot and makes the pacing fall out of completion rather than needing
  // a separate rate limiter.
  enum class RtaPhase : uint8_t {
    DISABLED,        // not enabled, or no rta: block at all
    UNSUPPORTED,     // device has no RTA, or speaks a different protocol version
    NEED_CAPS,       // read RtaCaps
    NEED_CENTRES,    // read the band-centre table, chunk by chunk
    READ_CONFIG,     // read the applied config before deciding to write
    SET_CONFIG,      // write our config
    VERIFY_CONFIG,   // read it back; the device clamps rather than rejecting
    STREAMING,       // polling band frames
    STOPPING,        // sending the courtesy RTA_CTL_STOP
  };

  // --- queue ---------------------------------------------------------------
  //
  // Sized from the largest burst plus what can legitimately be in flight
  // alongside it: a refresh is up to 4 fixed reads + TOGGLE_COUNT toggle reads
  // = 9, an RTA band poll owns at most one slot, a user-initiated SET one more,
  // and one spare. Recount this if the refresh burst grows -- and note that
  // loop() deliberately waits for room for the whole burst rather than letting
  // enqueue_() evict a sibling read.
  static constexpr uint8_t QUEUE_DEPTH = 12;
  std::array<Transaction, QUEUE_DEPTH> queue_{};

  bool enqueue_(const Transaction &txn);
  int find_next_() const;
  uint8_t queue_free_() const;
  void enqueue_get_(uint8_t req, uint16_t wlen, void (DSPiHub::*cb)(const uint8_t *, uint16_t), uint8_t priority = 1,
                    uint16_t wvalue = 0, void (DSPiHub::*on_fail)(uint8_t) = nullptr);

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
  void on_input_source_(const uint8_t *data, uint16_t len);
  void on_input_rate_(const uint8_t *data, uint16_t len);
  void publish_state_();

  // --- boolean DSP parameters ----------------------------------------------
  //
  // Transaction::on_ok carries no request identity, so each target needs its
  // own trampoline into the shared store. Recovering the target from
  // current_.req instead would work but would lean on an undocumented lifetime
  // coupling to save five one-line functions.
  void store_toggle_(ToggleTarget target, const uint8_t *data, uint16_t len);
  void mark_toggle_unsupported_(ToggleTarget target, uint8_t status);
  void on_user_mute_(const uint8_t *data, uint16_t len);
  void on_loudness_(const uint8_t *data, uint16_t len);
  void on_eq_bypass_(const uint8_t *data, uint16_t len);
  void on_crossfeed_(const uint8_t *data, uint16_t len);
  void on_leveller_(const uint8_t *data, uint16_t len);
  void on_user_mute_failed_(uint8_t status);
  void on_loudness_failed_(uint8_t status);
  void on_eq_bypass_failed_(uint8_t status);
  void on_crossfeed_failed_(uint8_t status);
  void on_leveller_failed_(uint8_t status);
  void enqueue_toggle_read_(ToggleTarget target);
  // Toggles worth reading: what the YAML configured, less anything the device
  // has permanently rejected.
  uint16_t toggles_to_read_() const { return toggle_read_mask_ & ~toggle_unsupported_; }

  // --- presets -------------------------------------------------------------
  void on_preset_dir_(const uint8_t *data, uint16_t len);
  void on_preset_active_(const uint8_t *data, uint16_t len);
  void on_preset_name_(const uint8_t *data, uint16_t len);
  void on_preset_load_(const uint8_t *data, uint16_t len);
  void on_preset_load_failed_(uint8_t status);
  // Re-reads the active slot until it matches what we asked for.  Called from
  // loop(); does nothing unless a load is awaiting confirmation.
  void service_preset_confirm_(uint32_t now);
  void request_preset_name_(uint8_t slot);

  // --- RTA -----------------------------------------------------------------
  void service_rta_(uint32_t now);
  void reset_rta_();
  void rta_advance_(RtaPhase next, uint32_t gap_ms = 0);
  void enqueue_rta_get_(uint8_t req, uint16_t wvalue, uint16_t wlen,
                        void (DSPiHub::*cb)(const uint8_t *, uint16_t), void (DSPiHub::*fail_cb)(uint8_t),
                        uint8_t priority = 1, uint8_t max_retries = Transaction::RETRIES_DEFAULT);
  void enqueue_rta_set_config_();
  void on_rta_caps_(const uint8_t *data, uint16_t len);
  void on_rta_centres_(const uint8_t *data, uint16_t len);
  void on_rta_config_(const uint8_t *data, uint16_t len);
  void on_rta_bands_(const uint8_t *data, uint16_t len);
  void on_rta_status_(const uint8_t *data, uint16_t len);
  void on_rta_control_(const uint8_t *data, uint16_t len);
  void on_rta_setup_failed_(uint8_t status);
  void on_rta_bands_failed_(uint8_t status);
  void on_rta_optional_failed_(uint8_t status);
  uint8_t rta_next_poll_channel_();
  void rta_mark_not_live_();

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

  // Mute is read unconditionally, because set_user_mute() is documented as
  // usable from a lambda on a device that configures no switch at all.
  uint16_t toggle_read_mask_{toggle_bit(ToggleTarget::USER_MUTE)};
  // Targets whose GET drew a permanent rejection -- older firmware, or a build
  // without that feature. Latched so the read stops being issued instead of
  // logging an error on every refresh forever. Cleared on re-identify.
  uint16_t toggle_unsupported_{0};

  // Notification bookkeeping.  A gap in seq means the device dropped events
  // for us, which is the only signal available that we have missed something.
  uint8_t last_seq_{0};
  bool have_seq_{false};
  bool notifications_active_{false};
  bool warned_no_notify_{false};

  DSPiState state_{};
  std::vector<DSPiStateListener *> listeners_;

  // --- preset state --------------------------------------------------------
  //
  // A load is deferred behind a flash write and a pipeline reset, so the
  // accepted-status byte proves nothing.  These track the confirmation poll.
  //
  // The settle delay is not optional padding.  The firmware pushes
  // PRESET_LOADED at the *start* of the load and only writes last_active_slot
  // at the end, so a read issued the instant the command is accepted returns
  // the slot we were on before.
  static constexpr uint32_t PRESET_CONFIRM_SETTLE_MS = 250;
  static constexpr uint8_t PRESET_CONFIRM_ATTEMPTS = 6;
  bool preset_confirm_pending_{false};
  uint8_t preset_confirm_slot_{0};
  uint8_t preset_confirm_attempts_{0};
  uint32_t preset_confirm_due_{0};
  // Which slot's name we last asked for, so the 32-byte read happens on a
  // change of active slot rather than on every refresh.
  uint8_t preset_name_slot_{0};
  bool preset_name_requested_{false};

  // --- RTA state -----------------------------------------------------------
  RtaPhase rta_phase_{RtaPhase::DISABLED};
  bool rta_configured_{false};  // an rta: block exists
  bool rta_enabled_{false};     // a consumer wants frames right now
  bool rta_busy_{false};        // one RTA transaction outstanding
  uint32_t rta_next_due_{0};
  uint32_t rta_status_due_{0};
  RtaConfig rta_cfg_desired_{};
  RtaConfig rta_cfg_applied_{};
  RtaCaps rta_caps_{};
  bool rta_caps_valid_{false};
  RtaStatus rta_status_{};
  uint16_t rta_band_centre_hz_[RTA_MAX_BANDS]{};
  uint8_t rta_centres_have_{0};   // centres collected so far
  uint16_t rta_centre_chunk_{1};  // wValue of the chunk being read
  // Channels still answering.  Starts as the applied channel_mask and loses any
  // channel the device refuses, so one bad index does not stop the others.
  uint16_t rta_poll_mask_{0};
  uint8_t rta_poll_cursor_{0};
  uint8_t rta_last_seq_[16]{};
  uint16_t rta_seq_valid_{0};  // bit N: rta_last_seq_[N] holds a real seq
  RtaBandUpdate rta_last_update_{};
  bool rta_live_{false};
  // Another client owns the analyser's config. Latched so it is reported once
  // rather than every status poll.
  bool rta_foreign_config_{false};
  CallbackManager<void(const RtaBandUpdate &)> rta_band_frame_callback_{};

  // Configuration.
  uint32_t request_timeout_ms_{400};
  uint32_t flash_timeout_ms_{1500};
  uint32_t backoff_base_ms_{60};
  uint32_t poll_interval_ms_{0};
  uint32_t refresh_debounce_ms_{250};
  // One 89-byte band-frame response (sync + type + 3 header + 82 + 2 CRC) plus
  // a notification must drain in a single pass, or a frame sits half-parsed
  // while its own timeout runs down.
  uint16_t max_bytes_per_loop_{128};
  uint32_t rta_interval_ms_{50};
  uint32_t rta_status_interval_ms_{2000};
  uint32_t rta_stale_ms_{500};
  bool rta_auto_enable_{false};
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
  uint32_t rta_frames_{0};
  uint32_t rta_dropped_{0};
  uint32_t rta_stale_events_{0};

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
