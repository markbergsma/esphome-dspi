#include "dspi.h"

#include <cinttypes>
#include <cstring>
#include <string>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dspi {

static const char *const TAG = "dspi";

// How long to wait between reprobes once the link is considered down.
static constexpr uint32_t OFFLINE_PROBE_INTERVAL_MS = 5000;
// Consecutive dropped transactions before declaring the link down.
static constexpr uint8_t FAILURES_BEFORE_OFFLINE = 5;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void DSPiHub::setup() {
  // Nothing blocking here: setup() must return promptly, so the probe is only
  // queued.  The response arrives through loop() like any other.
  start_probe_();
}

void DSPiHub::loop() {
  const uint32_t now = millis();

  pump_rx_();

  switch (hub_state_) {
    case HubState::AWAIT_RESP:
      if (now - sent_at_ >= current_.timeout_ms) {
        txn_timeouts_++;
        retry_or_drop_("timeout");
      }
      break;

    case HubState::BACKOFF:
      if (now - backoff_until_ < 0x80000000UL) {  // wrap-safe "now >= deadline"
        hub_state_ = HubState::IDLE;
      }
      break;

    default:
      break;
  }

  // Keep probing until the device answers. This covers the OFFLINE state and,
  // just as importantly, the case where the very first probe simply ran out of
  // retries: dropping it leaves an empty queue, and without this the component
  // would sit silent forever waiting for work that nothing was going to queue.
  // It is also how a DSPi that is powered up after the ESP32 gets picked up.
  if (!identified_ && (hub_state_ == HubState::IDLE || hub_state_ == HubState::OFFLINE) &&
      now - last_probe_at_ >= OFFLINE_PROBE_INTERVAL_MS) {
    start_probe_();
  }

  // A due refresh becomes real work only once the debounce window has passed,
  // so a burst of notifications collapses into one round of reads.
  if (refresh_pending_ && (now - refresh_due_at_) < 0x80000000UL) {
    refresh_pending_ = false;
    enqueue_get_(REQ_GET_MASTER_VOLUME, 4, &DSPiHub::on_master_volume_);
    enqueue_get_(REQ_GET_USER_VOLUME, 4, &DSPiHub::on_user_volume_);
    enqueue_get_(REQ_GET_USER_MUTE, 1, &DSPiHub::on_user_mute_);
    enqueue_get_(REQ_GET_INPUT_SOURCE, 1, &DSPiHub::on_input_source_);
  }

  // Polling is the fallback for a device whose notifications are switched off.
  // With notifications working this stays disabled and costs nothing.
  if (poll_interval_ms_ && !notifications_active_ && hub_state_ == HubState::IDLE &&
      now - last_poll_at_ >= poll_interval_ms_) {
    last_poll_at_ = now;
    request_refresh();
  }

  if (hub_state_ == HubState::IDLE) {
    send_next_();
  }
}

void DSPiHub::dump_config() {
  ESP_LOGCONFIG(TAG, "DSPi:");
  this->check_uart_settings(this->parent_->get_baud_rate(), 1, uart::UART_CONFIG_PARITY_NONE, 8);
  if (identified_) {
    ESP_LOGCONFIG(TAG, "  Firmware: %u.%u.%u (platform %u)", fw_major_, fw_minor_, fw_patch_, platform_id_);
  } else {
    ESP_LOGCONFIG(TAG, "  Firmware: not yet identified");
  }
  ESP_LOGCONFIG(TAG, "  Notifications: %s", notifications_active_ ? "enabled" : "disabled (polling)");
  if (poll_interval_ms_) {
    ESP_LOGCONFIG(TAG, "  Poll interval: %" PRIu32 " ms", poll_interval_ms_);
  }
  if (has_boot_input_source_) {
    ESP_LOGCONFIG(TAG, "  Boot input source: %u", boot_input_source_);
  }
  // Repeated here as well as logged at discovery: dump_config runs again for
  // every log client that attaches, whereas the probe happens once within a
  // second of boot and is easily missed.
  if (state_.selectable_valid) {
    ESP_LOGCONFIG(TAG, "  Selectable input sources: %s", selectable_sources_string_().c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  Selectable input sources: not yet known");
  }
  ESP_LOGCONFIG(TAG, "  Counters: %" PRIu32 " bytes in, %" PRIu32 " notifications, %" PRIu32 " seq gaps, %" PRIu32
                     " bad CRC, %" PRIu32 " malformed, %" PRIu32 " timeouts, %" PRIu32 " dropped",
                rx_bytes_, notify_count_, notify_gaps_, frames_bad_crc_, frames_malformed_, txn_timeouts_,
                txn_dropped_);
  if (rx_bytes_ == 0) {
    ESP_LOGCONFIG(TAG, "  No bytes have arrived from the DSPi. Check that TX and RX are crossed,");
    ESP_LOGCONFIG(TAG, "  that grounds are common, and that its UART interface is enabled over USB.");
  }
}

// ---------------------------------------------------------------------------
// Queue
// ---------------------------------------------------------------------------

bool DSPiHub::enqueue_(const Transaction &txn) {
  // Coalesce: an identical opcode already waiting is replaced rather than
  // appended.  Dragging a volume slider produces one in-flight request and one
  // queued request holding the newest value, instead of a backlog that keeps
  // draining after the user has stopped.
  for (auto &slot : queue_) {
    if (slot.in_use && slot.req == txn.req && slot.frame_type == txn.frame_type) {
      const uint8_t attempts = slot.attempts;
      slot = txn;
      slot.in_use = true;
      // Keep the higher attempt count of the two. A retry that coalesced with
      // a freshly queued request of the same opcode would otherwise have its
      // budget reset to zero, and a device answering BUSY indefinitely would
      // then be retried forever.
      slot.attempts = attempts > txn.attempts ? attempts : txn.attempts;
      return true;
    }
  }

  for (auto &slot : queue_) {
    if (!slot.in_use) {
      slot = txn;
      slot.in_use = true;
      return true;
    }
  }

  // Full.  Evict the least important waiting entry rather than blocking or
  // growing; a dropped background refresh will be reissued by the next hint.
  Transaction *victim = nullptr;
  for (auto &slot : queue_) {
    if (!victim || slot.priority > victim->priority) {
      victim = &slot;
    }
  }
  if (victim && victim->priority >= txn.priority) {
    *victim = txn;
    victim->in_use = true;
    txn_dropped_++;
    return true;
  }
  txn_dropped_++;
  return false;
}

int DSPiHub::find_next_() const {
  int best = -1;
  for (int i = 0; i < QUEUE_DEPTH; i++) {
    if (!queue_[i].in_use)
      continue;
    if (best < 0 || queue_[i].priority < queue_[best].priority) {
      best = i;
    }
  }
  return best;
}

void DSPiHub::enqueue_get_(uint8_t req, uint16_t wlen, void (DSPiHub::*cb)(const uint8_t *, uint16_t),
                           uint8_t priority) {
  Transaction t;
  t.req = req;
  t.frame_type = FRAME_GET_REQ;
  t.wlen = wlen;  // caps the response size
  t.priority = priority;
  t.timeout_ms = request_timeout_ms_;
  t.on_ok = cb;
  enqueue_(t);
}

// ---------------------------------------------------------------------------
// Transport
// ---------------------------------------------------------------------------

void DSPiHub::pump_rx_() {
  // Bounded per call: this component shares a cooperatively scheduled loop
  // with everything else on the device, including the audio path.
  uint16_t budget = max_bytes_per_loop_;
  while (budget-- && this->available()) {
    uint8_t byte;
    if (!this->read_byte(&byte))
      break;
    // Counted before parsing, so a silent link can be told apart from one
    // carrying bytes we cannot make sense of. Zero here means nothing is
    // arriving at all, which is a wiring question rather than a protocol one.
    rx_bytes_++;

    switch (parser_.feed(byte)) {
      case ParseResult::FRAME:
        handle_frame_();
        break;
      case ParseResult::CRC_FAILED:
        frames_bad_crc_++;
        // Deliberately not failing the outstanding transaction here: we may
        // have mis-parsed a frame boundary, and sending a retry into the tail
        // of a frame still arriving would compound the desync.  Letting the
        // response timeout fire retries from a known-quiet link instead.
        ESP_LOGW(TAG, "Frame failed CRC; waiting for timeout");
        break;
      case ParseResult::OVERSIZE:
        frames_malformed_++;
        ESP_LOGW(TAG, "Oversize frame length; resynchronising");
        break;
      case ParseResult::NEED_MORE:
        break;
    }
  }
}

void DSPiHub::handle_frame_() {
  // Dispatch on the frame type rather than on hub state, so a notification
  // arriving mid-transaction is handled without disturbing that transaction.
  if (parser_.type() == FRAME_NOTIFY) {
    handle_notification_(parser_.payload(), parser_.len());
    return;
  }
  handle_response_(parser_.type(), parser_.status(), parser_.payload(), parser_.len());
}

void DSPiHub::handle_response_(uint8_t type, uint8_t status, const uint8_t *payload, uint16_t len) {
  if (hub_state_ != HubState::AWAIT_RESP || !has_current_) {
    ESP_LOGV(TAG, "Unsolicited response (type 0x%02X); ignoring", type);
    return;
  }

  // The response kind must match what we asked for.  A mismatch means we are
  // reading someone else's frame, so treat it as a desync and let the timeout
  // resynchronise rather than acting on data we cannot attribute.
  const uint8_t expected = (current_.frame_type == FRAME_SET_REQ) ? FRAME_SET_RESP : FRAME_GET_RESP;
  if (type != expected) {
    ESP_LOGW(TAG, "Response type 0x%02X does not match request 0x%02X; ignoring", type, current_.req);
    return;
  }

  if (status == CTRL_STATUS_OK) {
    consecutive_failures_ = 0;
    auto cb = current_.on_ok;
    has_current_ = false;
    hub_state_ = HubState::IDLE;
    if (cb) {
      (this->*cb)(payload, len);
    }
    return;
  }

  if (ctrl_status_is_retryable(status)) {
    retry_or_drop_(ctrl_status_to_string(status));
    return;
  }

  // Permanent failures are dropped without retry.  BLOCKED in particular
  // means the command is refused on this transport by design and will be
  // refused identically every time; retrying it would spin forever.
  ESP_LOGE(TAG, "Command 0x%02X rejected: %s", current_.req, ctrl_status_to_string(status));
  has_current_ = false;
  hub_state_ = HubState::IDLE;
}

void DSPiHub::handle_notification_(const uint8_t *packet, uint16_t len) {
  if (len < NOTIFY_HEADER_LEN || packet[0] != NOTIFY_VERSION_V2) {
    ESP_LOGV(TAG, "Ignoring notification packet (len %u, version %u)", len, len ? packet[0] : 0);
    return;
  }

  notify_count_++;
  if (!notifications_active_) {
    // Seeing one of these proves the device is pushing, whatever we expected.
    notifications_active_ = true;
    ESP_LOGD(TAG, "Notifications are active");
  }

  const uint8_t event_id = packet[1];
  const uint8_t seq = packet[3];

  if (have_seq_) {
    const uint8_t expected = static_cast<uint8_t>(last_seq_ + 1);
    if (seq != expected) {
      // The device dropped events for us. There is no way to learn which, so
      // re-read everything we publish.
      notify_gaps_++;
      ESP_LOGW(TAG, "Notification sequence gap (expected %u, got %u); re-syncing", expected, seq);
      request_refresh();
    }
  }
  last_seq_ = seq;
  have_seq_ = true;

  // Notifications are treated as a hint that something changed, not as a
  // carrier of the new value.  PARAM_CHANGED identifies its field by byte
  // offset into the firmware's internal parameter struct, which is tied to a
  // wire-format version that has already moved several times; decoding it
  // would silently produce wrong values after a DSPi firmware update. Reading
  // back the three values we publish costs a few dozen bytes and cannot rot.
  if (notify_event_affects_params(event_id)) {
    request_refresh();
  } else {
    ESP_LOGV(TAG, "Notification event 0x%02X (no action)", event_id);
  }
}

void DSPiHub::send_next_() {
  const int idx = find_next_();
  if (idx < 0)
    return;

  current_ = queue_[idx];
  queue_[idx].in_use = false;
  has_current_ = true;

  uint8_t frame[MAX_TX_FRAME];
  const size_t n = build_request_frame(frame, current_.frame_type, current_.req, current_.wvalue, /*windex=*/0,
                                       current_.wlen, current_.payload, current_.payload_len);

  // Small enough to hand to the driver in one go; the TX FIFO absorbs it
  // without blocking, so there is no reason to pump this byte by byte.
  this->write_array(frame, n);

  sent_at_ = millis();
  hub_state_ = HubState::AWAIT_RESP;
  ESP_LOGV(TAG, "Sent 0x%02X (attempt %u)", current_.req, current_.attempts + 1);
}

void DSPiHub::retry_or_drop_(const char *reason) {
  if (!has_current_) {
    hub_state_ = HubState::IDLE;
    return;
  }

  current_.attempts++;
  if (current_.attempts > max_retries_) {
    ESP_LOGW(TAG, "Command 0x%02X failed after %u attempts (%s)", current_.req, current_.attempts, reason);
    fail_current_(reason);
    return;
  }

  // Exponential backoff. The device disables interrupts for roughly 45 ms
  // while it writes flash, so the first retry already clears a single-sector
  // write and the ladder covers a multi-sector one.
  const uint32_t shift = current_.attempts > 3 ? 3 : current_.attempts - 1;
  backoff_until_ = millis() + (backoff_base_ms_ << shift);
  hub_state_ = HubState::BACKOFF;

  // Put it back, preserving both its attempt count and its original priority:
  // promoting a failing background refresh would let it push ahead of the
  // user's own commands.
  Transaction again = current_;
  has_current_ = false;
  enqueue_(again);
  ESP_LOGD(TAG, "Retrying 0x%02X after %s", again.req, reason);
}

void DSPiHub::fail_current_(const char *reason) {
  has_current_ = false;
  txn_dropped_++;
  hub_state_ = HubState::IDLE;
  if (++consecutive_failures_ >= FAILURES_BEFORE_OFFLINE) {
    ESP_LOGE(TAG, "DSPi unreachable (%s)", reason);
    go_offline_();
  }
}

void DSPiHub::go_offline_() {
  hub_state_ = HubState::OFFLINE;
  last_probe_at_ = millis();
  identified_ = false;
  // A device that went away may come back with different state, and its
  // notification counter restarts from scratch, so treat the next sequence
  // number as a fresh baseline rather than a gap.
  have_seq_ = false;
  for (auto &slot : queue_) {
    slot.in_use = false;
  }
  publish_state_();
}

// ---------------------------------------------------------------------------
// Probe chain
// ---------------------------------------------------------------------------

void DSPiHub::start_probe_() {
  hub_state_ = HubState::PROBING;
  last_probe_at_ = millis();
  parser_.reset();
  enqueue_get_(REQ_GET_PLATFORM, 7, &DSPiHub::on_platform_, /*priority=*/0);
  // PROBING only gates the reprobe timer; the queue drains normally.
  hub_state_ = HubState::IDLE;
}

void DSPiHub::on_platform_(const uint8_t *data, uint16_t len) {
  if (len < 2) {
    ESP_LOGW(TAG, "Short platform response (%u bytes)", len);
    return;
  }
  platform_id_ = data[0];
  fw_major_ = data[1];
  // Byte 2 packs minor and patch for older hosts; bytes 4 and 5 carry them at
  // full width. Prefer those when the device is new enough to send them.
  if (len >= 6) {
    fw_minor_ = data[4];
    fw_patch_ = data[5];
  } else if (len >= 3) {
    fw_minor_ = data[2] >> 4;
    fw_patch_ = data[2] & 0x0F;
  }
  identified_ = true;
  consecutive_failures_ = 0;
  ESP_LOGI(TAG, "Connected to DSPi firmware %u.%u.%u (platform %u)", fw_major_, fw_minor_, fw_patch_, platform_id_);

  enqueue_get_(REQ_GET_UART_CONFIG, 8, &DSPiHub::on_uart_config_, /*priority=*/0);
  enqueue_get_(REQ_GET_CTRL_IFACE_STATUS, 8, &DSPiHub::on_iface_status_, /*priority=*/0);
  // Which sources this device can actually select. ADAT and S/PDIF 2-4 ship
  // disabled, so the real list is narrower than the enum; the firmware exposes
  // 0xEF specifically so a host can build its source list from the device
  // rather than hardcoding one.
  enqueue_get_(REQ_GET_SPDIF_INPUT_CONFIG, 2 + SPDIF_RX_NUM_INPUTS, &DSPiHub::on_spdif_input_config_,
               /*priority=*/0);
  enqueue_get_(REQ_GET_ADAT_INPUT_ENABLE, 1, &DSPiHub::on_adat_enable_, /*priority=*/0);
  enqueue_get_(REQ_GET_ADAT_INPUT_PIN, 1, &DSPiHub::on_adat_pin_, /*priority=*/0);
  request_refresh();
}

void DSPiHub::on_uart_config_(const uint8_t *data, uint16_t len) {
  if (len < 8) {
    ESP_LOGW(TAG, "Short UART config response (%u bytes)", len);
    return;
  }
  const bool notify_enabled = data[3] != 0;
  notifications_active_ = notify_enabled;

  if (!notify_enabled && !warned_no_notify_) {
    warned_no_notify_ = true;
    // Worth a warning rather than silence: without this the component still
    // works, but only as fast as it polls, and the cause is a one-line change
    // the user can only make over USB.
    ESP_LOGW(TAG,
             "DSPi notifications are disabled, so state updates will be polled. "
             "Set notify_enable=1 in UartCtrlConfig over USB to enable push updates.");
    if (poll_interval_ms_ == 0) {
      ESP_LOGW(TAG, "poll_interval is 0 and notifications are off: state will only update when changed from here.");
    }
  }
}

void DSPiHub::on_iface_status_(const uint8_t *data, uint16_t len) {
  if (len < 5) {
    ESP_LOGW(TAG, "Short interface status response (%u bytes)", len);
    return;
  }
  ESP_LOGD(TAG, "Control interface: uart_live=%u, protocol version %u", data[1], data[4]);
  if (data[1] == 0) {
    ESP_LOGW(TAG, "DSPi reports its UART interface is not live, despite answering us.");
  }
}

void DSPiHub::on_spdif_input_config_(const uint8_t *data, uint16_t len) {
  if (len < 2) {
    ESP_LOGW(TAG, "Short S/PDIF input config response (%u bytes)", len);
    return;
  }
  // data[0] = number of inputs, data[1] = enable mask with bit 0 (input 1)
  // always set, then one GPIO per input. USB and I2S need no enabling.
  const uint8_t count = data[0] < SPDIF_RX_NUM_INPUTS ? data[0] : SPDIF_RX_NUM_INPUTS;
  const uint8_t enable_mask = data[1];

  uint8_t mask = (1u << INPUT_SOURCE_USB) | (1u << INPUT_SOURCE_I2S);
  for (uint8_t i = 0; i < count; i++) {
    if (enable_mask & (1u << i)) {
      // Index 0 is INPUT_SOURCE_SPDIF; 1..3 are SPDIF2..SPDIF4, contiguous.
      mask |= 1u << (i == 0 ? INPUT_SOURCE_SPDIF : (INPUT_SOURCE_SPDIF2 + i - 1));
    }
  }
  // Preserve whatever the ADAT responses have already established.
  mask |= state_.selectable_mask & (1u << INPUT_SOURCE_ADAT);

  state_.selectable_mask = mask;
  state_.selectable_valid = true;
  log_selectable_sources_();
  publish_state_();
}

void DSPiHub::on_adat_enable_(const uint8_t *data, uint16_t len) {
  if (len < 1)
    return;
  adat_enabled_ = data[0] != 0;
  on_adat_pin_(nullptr, 0);  // re-evaluate with whatever the pin is known to be
}

void DSPiHub::on_adat_pin_(const uint8_t *data, uint16_t len) {
  if (len >= 1) {
    adat_pin_set_ = data[0] != 0xFF;
  }
  // ADAT needs both, and is RP2350-only; a device without it answers disabled.
  if (adat_enabled_ && adat_pin_set_) {
    state_.selectable_mask |= 1u << INPUT_SOURCE_ADAT;
  } else {
    state_.selectable_mask &= ~(uint8_t) (1u << INPUT_SOURCE_ADAT);
  }
  if (state_.selectable_valid) {
    log_selectable_sources_();
    publish_state_();
  }
}

void DSPiHub::log_selectable_sources_() {
  // Only on change: the ADAT enable and pin arrive in separate responses, so
  // the mask settles over two or three callbacks and would otherwise repeat.
  if (logged_selectable_ && logged_selectable_mask_ == state_.selectable_mask)
    return;
  logged_selectable_ = true;
  logged_selectable_mask_ = state_.selectable_mask;

  ESP_LOGI(TAG, "DSPi selectable input sources: %s", selectable_sources_string_().c_str());
}

std::string DSPiHub::selectable_sources_string_() const {
  static const char *const NAMES[INPUT_SOURCE_COUNT] = {"USB",     "S/PDIF",  "I2S",    "ADAT",
                                                        "S/PDIF2", "S/PDIF3", "S/PDIF4"};
  std::string list;
  for (uint8_t i = 0; i < INPUT_SOURCE_COUNT; i++) {
    if (state_.selectable_mask & (1u << i)) {
      if (!list.empty())
        list += ", ";
      list += NAMES[i];
    }
  }
  return list;
}

// ---------------------------------------------------------------------------
// Readback
// ---------------------------------------------------------------------------

void DSPiHub::on_master_volume_(const uint8_t *data, uint16_t len) {
  if (len < 4)
    return;
  float db;
  std::memcpy(&db, data, 4);
  state_.master_volume_db = db;
  state_.master_volume_valid = true;
  publish_state_();
}

void DSPiHub::on_user_volume_(const uint8_t *data, uint16_t len) {
  if (len < 4)
    return;
  float db;
  std::memcpy(&db, data, 4);
  state_.user_volume_db = db;
  state_.user_volume_valid = true;
  publish_state_();
}

void DSPiHub::on_user_mute_(const uint8_t *data, uint16_t len) {
  if (len < 1)
    return;
  state_.user_mute = data[0] != 0;
  state_.user_mute_valid = true;
  publish_state_();
}

void DSPiHub::on_input_source_(const uint8_t *data, uint16_t len) {
  if (len < 1)
    return;
  state_.input_source = data[0];
  state_.input_source_valid = true;
  publish_state_();

  // Claim the input once, on the first reading after boot, if asked to.  This
  // is deliberately not repeated: switching the source resets the DSPi audio
  // pipeline, so doing it automatically in response to anything else would
  // interrupt playback.
  if (has_boot_input_source_ && !boot_source_checked_) {
    boot_source_checked_ = true;
    if (state_.input_source != boot_input_source_) {
      ESP_LOGI(TAG, "Switching input source %u -> %u (boot_input_source)", state_.input_source, boot_input_source_);
      set_input_source(boot_input_source_);
    }
  }
}

void DSPiHub::publish_state_() {
  for (auto *l : listeners_) {
    l->on_dspi_state(state_);
  }
}

// ---------------------------------------------------------------------------
// Public control surface
// ---------------------------------------------------------------------------

void DSPiHub::set_master_volume_db(float db) {
  // -128 is a mute sentinel rather than a level, so it is allowed through
  // unclamped; everything else is held inside the range the device accepts.
  if (db != MASTER_VOLUME_MUTE_DB) {
    db = clamp(db, MASTER_VOLUME_MIN_DB, MASTER_VOLUME_MAX_DB);
  }

  Transaction t;
  t.req = REQ_SET_MASTER_VOLUME;
  t.frame_type = FRAME_SET_REQ;
  t.wlen = 4;
  t.payload_len = 4;
  std::memcpy(t.payload, &db, 4);
  t.priority = 0;
  t.timeout_ms = request_timeout_ms_;
  enqueue_(t);

  // The device answers OK for any well-formed frame, including one whose
  // value its own handler then rejects, so the reply is not proof the value
  // took. The readback is what actually confirms it, and is also what updates
  // the published state.
  request_refresh();
}

void DSPiHub::set_user_volume_db(float db) {
  // No mute sentinel on this one: the firmware clamps to [-60, 0] and mute is a
  // separate command. Sending -128 here would simply be clamped to -60, which
  // would be loud rather than silent -- so mute must go through set_user_mute().
  db = clamp(db, USER_VOLUME_MIN_DB, USER_VOLUME_MAX_DB);

  Transaction t;
  t.req = REQ_SET_USER_VOLUME;
  t.frame_type = FRAME_SET_REQ;
  t.wlen = 4;
  t.payload_len = 4;
  std::memcpy(t.payload, &db, 4);
  t.priority = 0;
  t.timeout_ms = request_timeout_ms_;
  enqueue_(t);
  request_refresh();
}

void DSPiHub::set_user_mute(bool mute) {
  Transaction t;
  t.req = REQ_SET_USER_MUTE;
  t.frame_type = FRAME_SET_REQ;
  t.wlen = 1;
  t.payload_len = 1;
  t.payload[0] = mute ? 1 : 0;
  t.priority = 0;
  t.timeout_ms = request_timeout_ms_;
  enqueue_(t);
  request_refresh();
}

void DSPiHub::set_input_source(uint8_t source) {
  if (!is_source_selectable(source)) {
    // Sent anyway: the device is the authority and silently ignores a source it
    // cannot select, which the readback will then reveal. Warning here turns
    // "the entity sprang back" into something with an explanation attached.
    ESP_LOGW(TAG, "Input source %u is not enabled on this DSPi; the device will likely ignore this", source);
  }

  Transaction t;
  t.req = REQ_SET_INPUT_SOURCE;
  t.frame_type = FRAME_SET_REQ;
  t.wlen = 1;
  t.payload_len = 1;
  t.payload[0] = source;
  t.priority = 0;
  // Changing the source tears down and rebuilds the audio pipeline and may
  // write flash, so allow it noticeably longer than an ordinary command.
  t.timeout_ms = flash_timeout_ms_;
  enqueue_(t);
  request_refresh();
}

void DSPiHub::request_refresh() {
  refresh_due_at_ = millis() + refresh_debounce_ms_;
  refresh_pending_ = true;
}

}  // namespace dspi
}  // namespace esphome
