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
  //
  // The burst goes out whole or not at all.  Every read in it carries the same
  // priority, so enqueue_() would answer a full queue by evicting a sibling
  // read -- silently, and with nothing that would ever reissue it: the probe's
  // refresh is one-shot, polling is off whenever notifications work, and the
  // next notification only arrives if something else changes.  Waiting for room
  // costs a few loop passes instead and cannot stall, because every queued
  // transaction ends in a response, a permanent rejection or a timeout.
  if (refresh_pending_ && (now - refresh_due_at_) < 0x80000000UL) {
    const uint16_t toggles = toggles_to_read_();
    if (queue_free_() >= 5 + toggle_mask_count(toggles)) {
      refresh_pending_ = false;
      enqueue_get_(REQ_GET_MASTER_VOLUME, 4, &DSPiHub::on_master_volume_);
      enqueue_get_(REQ_GET_USER_VOLUME, 4, &DSPiHub::on_user_volume_);
      enqueue_get_(REQ_GET_INPUT_SOURCE, 1, &DSPiHub::on_input_source_);
      // The pipeline rate follows the active source, and INPUT_FORMAT brings us
      // back through here whenever that source or its format changes.
      enqueue_get_(REQ_GET_INPUT_RATE, INPUT_RATE_LEN, &DSPiHub::on_input_rate_);
      // One byte, and the only way a preset loaded elsewhere -- DSPi Console, a
      // control surface -- becomes visible here.  The 32-byte name read it may
      // trigger is conditional on the slot having actually changed.
      enqueue_get_(REQ_PRESET_GET_ACTIVE, 1, &DSPiHub::on_preset_active_);
      // Mute, plus whichever DSP toggles this config exposes.  A preset load
      // rewrites all of them, and the trailing BULK_INVALIDATED brings us back
      // through here, so no separate handling is needed for that.
      for (uint8_t i = 0; i < TOGGLE_COUNT; i++) {
        const ToggleTarget target = static_cast<ToggleTarget>(i);
        if ((toggles & toggle_bit(target)) != 0)
          enqueue_toggle_read_(target);
      }
    }
  }

  // Polling is the fallback for a device whose notifications are switched off.
  // With notifications working this stays disabled and costs nothing.
  if (poll_interval_ms_ && !notifications_active_ && hub_state_ == HubState::IDLE &&
      now - last_poll_at_ >= poll_interval_ms_) {
    last_poll_at_ = now;
    request_refresh();
  }

  // Confirming a deferred preset load.  Ahead of the spectrum so a pending
  // confirmation is not starved by band polls, and cheap: it enqueues nothing
  // at all unless a load is actually awaiting confirmation.
  service_preset_confirm_(now);

  // Spectrum polling, last of the producers so the debounced refresh above and
  // any user command already queued keep precedence within this same pass.  It
  // enqueues at most one transaction and never goes through request_refresh().
  service_rta_(now);

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
  if (state_.active_preset_valid) {
    ESP_LOGCONFIG(TAG, "  Active preset: slot %u \"%s\"", state_.active_preset,
                  state_.active_preset_name_valid ? state_.active_preset_name.c_str() : "?");
  }
  if (state_.preset_dir_valid) {
    // Which slots hold saved settings. Loading an unlisted one is allowed and
    // simply applies factory defaults, so this is information rather than a
    // constraint. `tools/dspi_setup.py` prints the names over USB.
    std::string occupied;
    for (uint8_t slot = 0; slot < PRESET_SLOTS; slot++) {
      if ((state_.slot_occupied >> slot) & 1u) {
        if (!occupied.empty())
          occupied += ", ";
        occupied += std::to_string(static_cast<unsigned>(slot));
      }
    }
    ESP_LOGCONFIG(TAG, "  Saved preset slots: %s", occupied.empty() ? "none" : occupied.c_str());
  }
  // Which booleans are being polled, so a missing switch entity is visible as a
  // missing read rather than only as an entity that never moves.
  std::string toggles;
  std::string rejected;
  for (uint8_t i = 0; i < TOGGLE_COUNT; i++) {
    const ToggleTarget target = static_cast<ToggleTarget>(i);
    if ((toggle_read_mask_ & toggle_bit(target)) == 0)
      continue;
    std::string &dest = (toggle_unsupported_ & toggle_bit(target)) != 0 ? rejected : toggles;
    if (!dest.empty())
      dest += ", ";
    dest += toggle_name(target);
  }
  ESP_LOGCONFIG(TAG, "  DSP toggles read: %s", toggles.empty() ? "none" : toggles.c_str());
  if (!rejected.empty()) {
    ESP_LOGCONFIG(TAG, "  DSP toggles unsupported by this firmware: %s", rejected.c_str());
  }
  if (rta_configured_) {
    ESP_LOGCONFIG(TAG, "  Spectrum analyser:");
    ESP_LOGCONFIG(TAG, "    Requested: tap %u, channel mask 0x%04X, FFT order %u, avg %u ms, peak decay %u dB/s",
                  rta_cfg_desired_.tap, rta_cfg_desired_.channel_mask, rta_cfg_desired_.fft_order,
                  rta_cfg_desired_.avg_ms, rta_cfg_desired_.peak_decay_db_s);
    ESP_LOGCONFIG(TAG, "    Poll interval: %" PRIu32 " ms (x %u channels)", rta_interval_ms_,
                  rta_cfg_desired_.channel_mask ? __builtin_popcount(rta_cfg_desired_.channel_mask) : 0);
    if (rta_phase_ == RtaPhase::UNSUPPORTED) {
      ESP_LOGCONFIG(TAG, "    State: not supported by this firmware");
    } else if (!rta_caps_valid_) {
      ESP_LOGCONFIG(TAG, "    State: %s, capabilities not yet read", rta_enabled_ ? "enabled" : "idle");
    } else {
      ESP_LOGCONFIG(TAG, "    State: %s, %u bands (%u continuous bass), level zero %u",
                    rta_enabled_ ? "enabled" : "idle", rta_caps_.max_bands, rta_caps_.bass_bands,
                    rta_caps_.level_zero);
      ESP_LOGCONFIG(TAG, "    Applied: tap %u, channel mask 0x%04X, FFT order %u", rta_cfg_applied_.tap,
                    rta_cfg_applied_.channel_mask, rta_cfg_applied_.fft_order);
    }
    ESP_LOGCONFIG(TAG, "    Counters: %" PRIu32 " frames, %" PRIu32 " dropped, %" PRIu32 " stale", rta_frames_,
                  rta_dropped_, rta_stale_events_);
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
  // wvalue is part of the identity, not incidental: REQ_RTA_GET_BANDS carries
  // the channel there and REQ_RTA_GET_CAPS the chunk index, so matching on the
  // opcode alone would silently fold two different reads into one.
  for (auto &slot : queue_) {
    if (slot.in_use && slot.req == txn.req && slot.frame_type == txn.frame_type && slot.wvalue == txn.wvalue) {
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

uint8_t DSPiHub::queue_free_() const {
  uint8_t n = 0;
  for (const auto &slot : queue_) {
    if (!slot.in_use)
      n++;
  }
  return n;
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
                           uint8_t priority, uint16_t wvalue, void (DSPiHub::*on_fail)(uint8_t)) {
  Transaction t;
  t.req = req;
  t.frame_type = FRAME_GET_REQ;
  // Not merely a cap: the device truncates its response to wlen, so asking for
  // less than a command returns yields a short payload with a valid CRC.
  t.wlen = wlen;
  t.wvalue = wvalue;
  t.priority = priority;
  t.timeout_ms = request_timeout_ms_;
  t.on_ok = cb;
  // Optional, and only worth passing for a read whose failure we must remember:
  // most reads are stateless and a dropped one is reissued by the next refresh.
  t.on_fail = on_fail;
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
  auto fail_cb = current_.on_fail;
  has_current_ = false;
  hub_state_ = HubState::IDLE;
  if (fail_cb) {
    (this->*fail_cb)(status);
  }
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
  // A transaction may cap its own retries below the hub's budget.  A spectrum
  // band frame does: by the time a backoff has elapsed the frame it would
  // fetch is stale, and the next poll asks for a fresher one anyway.
  const uint8_t limit = current_.max_retries == Transaction::RETRIES_DEFAULT ? max_retries_ : current_.max_retries;
  if (current_.attempts > limit) {
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
  auto fail_cb = current_.on_fail;
  has_current_ = false;
  txn_dropped_++;
  hub_state_ = HubState::IDLE;
  if (++consecutive_failures_ >= FAILURES_BEFORE_OFFLINE) {
    ESP_LOGE(TAG, "DSPi unreachable (%s)", reason);
    go_offline_();
  }
  // After go_offline_(), so a state machine that reset itself there is not
  // dragged back out by its own failure handler.
  if (fail_cb) {
    (this->*fail_cb)(CTRL_STATUS_LINK_FAILED);
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
  // Clearing the queue silently would strand any state machine waiting on a
  // transaction that is now never going to complete.
  reset_rta_();
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
  // Preset occupancy, for reporting only.  The ten slot names are deliberately
  // not read here: dump_config runs long before this answers, so it could not
  // print them anyway, and nothing else needs a name for a slot the device is
  // not on.  `tools/dspi_setup.py` lists them over USB instead.
  enqueue_get_(REQ_PRESET_GET_DIR, PRESET_DIR_LEN, &DSPiHub::on_preset_dir_, /*priority=*/0);

  // Any preset load we were waiting on belongs to the device we were talking
  // to before, which may not be this one.  Drop it rather than confirming a
  // request the new device never received.
  preset_confirm_pending_ = false;
  preset_name_requested_ = false;
  state_.active_preset_name_valid = false;

  // Same reasoning for the boolean DSP parameters: a device we have not read
  // since identifying it may be a different one, so both the values and any
  // verdict of "this firmware does not have that parameter" are stale.  The
  // refresh at the end of this function is what fills them in again.
  state_.toggle_valid_mask = 0;
  toggle_unsupported_ = 0;

  // A device we have just identified may be a different one, or the same one
  // running new firmware, so anything we concluded about its RTA support is
  // now stale -- including a previous verdict of "unsupported".  Nothing is
  // asked of it here: RTA capabilities are read lazily on the first enable, so
  // a build with no spectrum page never sends an RTA byte.
  rta_caps_valid_ = false;
  rta_centres_have_ = 0;
  reset_rta_();
  if (rta_configured_ && rta_auto_enable_ && !rta_enabled_) {
    set_rta_enabled(true);
  }

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

void DSPiHub::store_toggle_(ToggleTarget target, const uint8_t *data, uint16_t len) {
  if (len < 1)
    return;
  const uint16_t bit = toggle_bit(target);
  if (data[0] != 0) {
    state_.toggle_values |= bit;
  } else {
    state_.toggle_values &= static_cast<uint16_t>(~bit);
  }
  state_.toggle_valid_mask |= bit;
  publish_state_();
}

void DSPiHub::mark_toggle_unsupported_(ToggleTarget target, uint8_t status) {
  // A link failure is not an answer about the parameter -- the device never
  // replied at all, so this arrived via fail_current_() rather than from a
  // rejection.  Latching on it would let one flaky moment stop a toggle being
  // read for good, and only a reprobe would undo it.
  if (status == CTRL_STATUS_LINK_FAILED)
    return;

  const uint16_t bit = toggle_bit(target);
  if ((toggle_unsupported_ & bit) != 0)
    return;
  toggle_unsupported_ |= bit;
  // Warn once and then go quiet.  A permanent rejection here means the firmware
  // has no such parameter, so retrying it on every refresh would fill the log
  // for the lifetime of the device without ever succeeding.
  ESP_LOGW(TAG, "Device rejected %s (0x%02X): %s; not reading it again", toggle_name(target),
           toggle_get_opcode(target), ctrl_status_to_string(status));
}

void DSPiHub::on_user_mute_(const uint8_t *data, uint16_t len) { store_toggle_(ToggleTarget::USER_MUTE, data, len); }
void DSPiHub::on_loudness_(const uint8_t *data, uint16_t len) { store_toggle_(ToggleTarget::LOUDNESS, data, len); }
void DSPiHub::on_eq_bypass_(const uint8_t *data, uint16_t len) { store_toggle_(ToggleTarget::EQ_BYPASS, data, len); }
void DSPiHub::on_crossfeed_(const uint8_t *data, uint16_t len) { store_toggle_(ToggleTarget::CROSSFEED, data, len); }
void DSPiHub::on_leveller_(const uint8_t *data, uint16_t len) {
  store_toggle_(ToggleTarget::LEVELLER, data, len);
}

void DSPiHub::on_user_mute_failed_(uint8_t status) { mark_toggle_unsupported_(ToggleTarget::USER_MUTE, status); }
void DSPiHub::on_loudness_failed_(uint8_t status) { mark_toggle_unsupported_(ToggleTarget::LOUDNESS, status); }
void DSPiHub::on_eq_bypass_failed_(uint8_t status) { mark_toggle_unsupported_(ToggleTarget::EQ_BYPASS, status); }
void DSPiHub::on_crossfeed_failed_(uint8_t status) { mark_toggle_unsupported_(ToggleTarget::CROSSFEED, status); }
void DSPiHub::on_leveller_failed_(uint8_t status) {
  mark_toggle_unsupported_(ToggleTarget::LEVELLER, status);
}

void DSPiHub::enqueue_toggle_read_(ToggleTarget target) {
  void (DSPiHub::*ok)(const uint8_t *, uint16_t) = nullptr;
  void (DSPiHub::*fail)(uint8_t) = nullptr;
  switch (target) {
    case ToggleTarget::USER_MUTE:
      ok = &DSPiHub::on_user_mute_;
      fail = &DSPiHub::on_user_mute_failed_;
      break;
    case ToggleTarget::LOUDNESS:
      ok = &DSPiHub::on_loudness_;
      fail = &DSPiHub::on_loudness_failed_;
      break;
    case ToggleTarget::EQ_BYPASS:
      ok = &DSPiHub::on_eq_bypass_;
      fail = &DSPiHub::on_eq_bypass_failed_;
      break;
    case ToggleTarget::CROSSFEED:
      ok = &DSPiHub::on_crossfeed_;
      fail = &DSPiHub::on_crossfeed_failed_;
      break;
    case ToggleTarget::LEVELLER:
      ok = &DSPiHub::on_leveller_;
      fail = &DSPiHub::on_leveller_failed_;
      break;
  }
  enqueue_get_(toggle_get_opcode(target), 1, ok, /*priority=*/1, /*wvalue=*/0, fail);
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

void DSPiHub::on_input_rate_(const uint8_t *data, uint16_t len) {
  InputRate rate;
  if (!parse_input_rate(data, len, &rate))
    return;
  // Only the live pipeline rate is published. The companion field is the
  // *selected* I2S input rate, which says nothing in clock-slave mode where
  // the device detects the rate for itself.
  state_.pipeline_rate_hz = rate.freq;
  state_.pipeline_rate_valid = true;
  publish_state_();
}

void DSPiHub::publish_state_() {
  for (auto *l : listeners_) {
    l->on_dspi_state(state_);
  }
}

// ---------------------------------------------------------------------------
// Presets
// ---------------------------------------------------------------------------
//
// A preset is the whole DSP state, so loading one can move master volume, the
// input source and the output configuration as well.  Nothing here has to
// handle that: the device emits BULK_INVALIDATED at the end of a load, which
// already drives a full refresh of everything this component publishes.

void DSPiHub::on_preset_dir_(const uint8_t *data, uint16_t len) {
  PresetDirectory dir;
  if (!parse_preset_directory(data, len, &dir)) {
    ESP_LOGW(TAG, "Short preset directory response (%u bytes)", len);
    return;
  }
  state_.slot_occupied = dir.slot_occupied;
  state_.preset_dir_valid = true;
  // last_active_slot is the same field REQ_PRESET_GET_ACTIVE returns, so the
  // directory read doubles as a first active-slot reading.
  on_preset_active_(&dir.last_active_slot, 1);
}

void DSPiHub::on_preset_active_(const uint8_t *data, uint16_t len) {
  if (len < 1)
    return;
  const uint8_t slot = data[0];
  if (slot >= PRESET_SLOTS) {
    ESP_LOGW(TAG, "DSPi reported active preset slot %u, which is out of range", slot);
    return;
  }

  const bool changed = !state_.active_preset_valid || state_.active_preset != slot;
  state_.active_preset = slot;
  state_.active_preset_valid = true;

  if (preset_confirm_pending_ && slot == preset_confirm_slot_) {
    ESP_LOGD(TAG, "Preset slot %u confirmed active", slot);
    preset_confirm_pending_ = false;
  }

  // The name is a 32-byte read, so it is worth doing only when the slot has
  // actually moved.  Re-request on a change even if one is already in flight:
  // enqueue_() coalesces on opcode and wValue, so a stale request for the
  // previous slot is replaced rather than both being sent.
  if (changed || !state_.active_preset_name_valid) {
    request_preset_name_(slot);
  }
  publish_state_();
}

void DSPiHub::request_preset_name_(uint8_t slot) {
  preset_name_slot_ = slot;
  preset_name_requested_ = true;
  enqueue_get_(REQ_PRESET_GET_NAME, PRESET_NAME_LEN, &DSPiHub::on_preset_name_, /*priority=*/1,
               /*wvalue=*/slot);
}

void DSPiHub::on_preset_name_(const uint8_t *data, uint16_t len) {
  if (len < 1) {
    ESP_LOGW(TAG, "Short preset name response (%u bytes)", len);
    return;
  }
  // The reply carries no slot number, so it is attributed to the slot the
  // outstanding request named.  If the active slot moved again while this was
  // in flight, drop it and let the newer request answer instead.
  if (!preset_name_requested_ || preset_name_slot_ != state_.active_preset)
    return;

  preset_name_requested_ = false;
  state_.active_preset_name.assign(reinterpret_cast<const char *>(data), preset_name_length(data, len));
  state_.active_preset_name_valid = true;
  publish_state_();
}

void DSPiHub::set_preset_slot(uint8_t slot) {
  if (slot >= PRESET_SLOTS) {
    ESP_LOGE(TAG, "Preset slot %u is out of range (0..%u)", slot, PRESET_SLOTS - 1);
    return;
  }
  if (state_.preset_dir_valid && !((state_.slot_occupied >> slot) & 1u)) {
    // Sent anyway: an empty slot is a legitimate thing to load, it just means
    // factory defaults rather than stored settings. Worth saying out loud,
    // because the audible result is not what "load a preset" suggests.
    ESP_LOGW(TAG, "Preset slot %u has never been saved; loading it applies factory defaults", slot);
  }

  Transaction t;
  t.req = REQ_PRESET_LOAD;
  // A GET frame even though this rewrites every DSP parameter: the firmware
  // dispatches it under vendor_handle_get() with the slot in wValue. Sending
  // it as a SET stalls.
  t.frame_type = FRAME_GET_REQ;
  t.wvalue = slot;
  t.wlen = 1;
  t.priority = 0;
  // The load writes flash and resets the pipeline, so it gets the same longer
  // budget as an input-source switch.
  t.timeout_ms = flash_timeout_ms_;
  t.on_ok = &DSPiHub::on_preset_load_;
  t.on_fail = &DSPiHub::on_preset_load_failed_;
  if (!enqueue_(t))
    return;

  preset_confirm_pending_ = true;
  preset_confirm_slot_ = slot;
  preset_confirm_attempts_ = 0;
  preset_confirm_due_ = millis() + PRESET_CONFIRM_SETTLE_MS;
}

void DSPiHub::on_preset_load_(const uint8_t *data, uint16_t len) {
  // A PRESET_* code, not a CtrlStatus: the transport already said OK to get
  // here, and this byte is the preset layer's own verdict on the request.
  const uint8_t status = len >= 1 ? data[0] : static_cast<uint8_t>(PRESET_OK);
  if (status != PRESET_OK) {
    ESP_LOGE(TAG, "DSPi rejected preset load of slot %u: %s", preset_confirm_slot_,
             preset_status_to_string(status));
    preset_confirm_pending_ = false;
    // Republish so the entity springs back to whatever the device is really
    // on, rather than sitting on a slot that was refused.
    publish_state_();
    return;
  }
  // Accepted, not applied: the firmware only set a pending flag. The real work
  // happens in its main loop, and service_preset_confirm_() watches for it.
  ESP_LOGD(TAG, "Preset load of slot %u accepted; awaiting confirmation", preset_confirm_slot_);
}

void DSPiHub::on_preset_load_failed_(uint8_t status) {
  ESP_LOGW(TAG, "Preset load of slot %u failed: %s", preset_confirm_slot_, ctrl_status_to_string(status));
  // Clearing this matters: without it a permanent failure would leave the
  // confirmation poll running against a load the device never started.
  preset_confirm_pending_ = false;
  publish_state_();
}

void DSPiHub::service_preset_confirm_(uint32_t now) {
  if (!preset_confirm_pending_)
    return;
  if ((now - preset_confirm_due_) >= 0x80000000UL)  // wrap-safe "now < due"
    return;

  if (++preset_confirm_attempts_ > PRESET_CONFIRM_ATTEMPTS) {
    ESP_LOGW(TAG, "Preset slot %u never became active; leaving the entity on what the device reports",
             preset_confirm_slot_);
    preset_confirm_pending_ = false;
    publish_state_();
    return;
  }

  preset_confirm_due_ = now + PRESET_CONFIRM_SETTLE_MS;
  enqueue_get_(REQ_PRESET_GET_ACTIVE, 1, &DSPiHub::on_preset_active_, /*priority=*/0);
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

void DSPiHub::set_toggle(ToggleTarget target, bool on) {
  Transaction t;
  // An ordinary SET, unlike the preset load: these opcodes sit on the firmware's
  // SET path.  wvalue stays 0, as every other setter here leaves it -- each
  // toggle has its own opcode, so enqueue_()'s (req, frame_type, wvalue)
  // identity already keeps two different toggles apart, and folds a toggle
  // flipped twice in quick succession down to the newest value, which is what
  // we want.  Packing the target into wvalue would work (the firmware ignores
  // it) but would put a meaningless value on the wire.
  t.req = toggle_set_opcode(target);
  t.frame_type = FRAME_SET_REQ;
  t.wlen = 1;
  t.payload_len = 1;
  t.payload[0] = on ? 1 : 0;
  t.priority = 0;
  // None of these writes flash or resets the pipeline, so an ordinary timeout.
  t.timeout_ms = request_timeout_ms_;
  enqueue_(t);

  // Start reading it back even if no entity asked for it.  Without this a
  // lambda-only caller would write the value and never observe it, leaving
  // toggle_valid() false for the lifetime of the device.
  toggle_read_mask_ |= toggle_bit(target);
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

void DSPiHub::request_input_rate() {
  // Skipped while the device has not answered the probe: the queue is for
  // work that can actually be sent, and a poller firing every few seconds
  // would otherwise churn it for the whole time a DSPi is absent.
  if (!is_online())
    return;
  enqueue_get_(REQ_GET_INPUT_RATE, INPUT_RATE_LEN, &DSPiHub::on_input_rate_);
}

// ---------------------------------------------------------------------------
// Spectrum analyser (RTA)
// ---------------------------------------------------------------------------
//
// The engine is transient: it starts on the first band read and stops itself
// five seconds after the last one.  That shapes everything here.  There is no
// explicit start, disabling is mostly a matter of ceasing to ask, and a device
// nobody is watching costs nothing.
//
// Bring-up reads capabilities, then the band-centre table, then the applied
// config, and only writes a config if the device's differs from ours.  The
// read-back is not ceremony: the device boots with the output tap selected
// across every output channel, so a silently failed write leaves an
// eight-channel rotation that still draws a spectrum, just eight times too
// slowly.  Comparing what came back is the only way to catch that.

void DSPiHub::reset_rta_() {
  rta_busy_ = false;
  rta_centre_chunk_ = 1;
  rta_poll_cursor_ = 0;
  rta_seq_valid_ = 0;
  rta_next_due_ = 0;
  rta_status_due_ = 0;
  rta_foreign_config_ = false;
  rta_mark_not_live_();
  if (!rta_configured_) {
    rta_phase_ = RtaPhase::DISABLED;
    return;
  }
  rta_phase_ = rta_enabled_ ? (rta_caps_valid_ ? RtaPhase::READ_CONFIG : RtaPhase::NEED_CAPS) : RtaPhase::DISABLED;
}

void DSPiHub::rta_advance_(RtaPhase next, uint32_t gap_ms) {
  rta_phase_ = next;
  rta_busy_ = false;
  rta_next_due_ = millis() + gap_ms;
}

void DSPiHub::set_rta_enabled(bool enabled) {
  if (!rta_configured_) {
    ESP_LOGW(TAG, "RTA requested but no rta: block is configured");
    return;
  }
  if (enabled == rta_enabled_)
    return;
  rta_enabled_ = enabled;

  if (enabled) {
    if (rta_phase_ == RtaPhase::UNSUPPORTED) {
      ESP_LOGD(TAG, "RTA enable ignored: device does not support it");
      return;
    }
    rta_busy_ = false;
    rta_next_due_ = millis();
    rta_phase_ = rta_caps_valid_ ? RtaPhase::READ_CONFIG : RtaPhase::NEED_CAPS;
    ESP_LOGD(TAG, "RTA enabled");
    return;
  }

  ESP_LOGD(TAG, "RTA disabled");
  rta_mark_not_live_();
  // Courtesy stop, so the DSPi drops the FFT and bass-bank work now rather
  // than when its idle timer expires.  If it fails, nothing is lost: the
  // engine switches itself off a few seconds later anyway.
  if (rta_phase_ == RtaPhase::STREAMING && !rta_busy_ && is_online()) {
    rta_phase_ = RtaPhase::STOPPING;
    rta_next_due_ = millis();
  } else {
    rta_phase_ = RtaPhase::DISABLED;
    rta_busy_ = false;
  }
}

void DSPiHub::rta_reset_avg() {
  if (rta_phase_ != RtaPhase::STREAMING)
    return;
  Transaction t;
  t.req = REQ_RTA_CONTROL;
  // Write-as-read: the action rides in wValue on the GET path.
  t.frame_type = FRAME_GET_REQ;
  t.wvalue = RTA_CTL_RESET_AVG;
  t.wlen = 1;
  t.priority = 1;
  t.timeout_ms = request_timeout_ms_;
  enqueue_(t);
}

void DSPiHub::set_rta_tap(uint8_t tap) {
  if (rta_cfg_desired_.tap == tap)
    return;
  rta_cfg_desired_.tap = tap;
  if (rta_enabled_ && rta_phase_ != RtaPhase::UNSUPPORTED) {
    rta_advance_(RtaPhase::READ_CONFIG);
  }
}

void DSPiHub::set_rta_channel_mask(uint16_t mask) {
  if (!mask || rta_cfg_desired_.channel_mask == mask)
    return;
  rta_cfg_desired_.channel_mask = mask;
  if (rta_enabled_ && rta_phase_ != RtaPhase::UNSUPPORTED) {
    rta_advance_(RtaPhase::READ_CONFIG);
  }
}

void DSPiHub::enqueue_rta_get_(uint8_t req, uint16_t wvalue, uint16_t wlen,
                               void (DSPiHub::*cb)(const uint8_t *, uint16_t), void (DSPiHub::*fail_cb)(uint8_t),
                               uint8_t priority, uint8_t max_retries) {
  Transaction t;
  t.req = req;
  t.frame_type = FRAME_GET_REQ;
  t.wvalue = wvalue;
  // Always the full structure length: the device truncates to wlen.
  t.wlen = wlen;
  t.priority = priority;
  t.max_retries = max_retries;
  t.timeout_ms = request_timeout_ms_;
  t.on_ok = cb;
  t.on_fail = fail_cb;
  if (enqueue_(t)) {
    rta_busy_ = true;
  } else {
    // The queue is full of work that outranks a spectrum poll. Wait a beat
    // rather than re-offering it on every loop iteration.
    rta_next_due_ = millis() + rta_interval_ms_;
  }
}

void DSPiHub::enqueue_rta_set_config_() {
  Transaction t;
  t.req = REQ_RTA_SET_CONFIG;
  t.frame_type = FRAME_SET_REQ;
  t.wlen = RTA_CONFIG_LEN;
  t.payload_len = RTA_CONFIG_LEN;
  rta_write_config(t.payload, rta_cfg_desired_);
  t.priority = 1;
  t.timeout_ms = request_timeout_ms_;
  t.on_ok = &DSPiHub::on_rta_config_;
  t.on_fail = &DSPiHub::on_rta_setup_failed_;
  if (enqueue_(t)) {
    rta_busy_ = true;
  } else {
    rta_next_due_ = millis() + rta_interval_ms_;
  }
}

uint8_t DSPiHub::rta_next_poll_channel_() {
  if (!rta_poll_mask_)
    return 0xFF;
  for (uint8_t i = 0; i < 16; i++) {
    const uint8_t ch = static_cast<uint8_t>((rta_poll_cursor_ + i) % 16);
    if (rta_poll_mask_ & (1u << ch)) {
      rta_poll_cursor_ = static_cast<uint8_t>((ch + 1) % 16);
      return ch;
    }
  }
  return 0xFF;
}

void DSPiHub::rta_mark_not_live_() {
  if (!rta_live_)
    return;
  rta_live_ = false;
  rta_stale_events_++;
  // Edge-triggered: one final update carrying the last good frame marked dead,
  // so a display can dim what it has.  Repeating this every poll interval
  // would cost a redraw a frame for a spectrum that is not moving.
  rta_last_update_.live = false;
  rta_band_frame_callback_.call(rta_last_update_);
}

void DSPiHub::service_rta_(uint32_t now) {
  if (!rta_configured_ || rta_busy_ || !identified_)
    return;
  if (hub_state_ == HubState::OFFLINE)
    return;
  // Wrap-safe "now >= rta_next_due_".
  if ((now - rta_next_due_) >= 0x80000000UL)
    return;

  switch (rta_phase_) {
    case RtaPhase::DISABLED:
    case RtaPhase::UNSUPPORTED:
      return;

    case RtaPhase::NEED_CAPS:
      enqueue_rta_get_(REQ_RTA_GET_CAPS, 0, RTA_CAPS_LEN, &DSPiHub::on_rta_caps_, &DSPiHub::on_rta_setup_failed_);
      return;

    case RtaPhase::NEED_CENTRES:
      enqueue_rta_get_(REQ_RTA_GET_CAPS, rta_centre_chunk_, RTA_CENTRES_CHUNK_LEN, &DSPiHub::on_rta_centres_,
                       &DSPiHub::on_rta_optional_failed_);
      return;

    case RtaPhase::READ_CONFIG:
    case RtaPhase::VERIFY_CONFIG:
      enqueue_rta_get_(REQ_RTA_GET_CONFIG, 0, RTA_CONFIG_LEN, &DSPiHub::on_rta_config_,
                       &DSPiHub::on_rta_setup_failed_);
      return;

    case RtaPhase::SET_CONFIG:
      enqueue_rta_set_config_();
      return;

    case RtaPhase::STOPPING:
      enqueue_rta_get_(REQ_RTA_CONTROL, RTA_CTL_STOP, 1, &DSPiHub::on_rta_control_,
                       &DSPiHub::on_rta_optional_failed_, /*priority=*/1, /*max_retries=*/0);
      return;

    case RtaPhase::STREAMING: {
      // Status is read occasionally to notice a device that rebooted and lost
      // our config.  It deliberately does not count as a read for the idle
      // timer, so it can never keep the engine alive on its own and mask a
      // band-read path that has stopped working.
      if ((now - rta_status_due_) < 0x80000000UL) {
        rta_status_due_ = now + rta_status_interval_ms_;
        enqueue_rta_get_(REQ_RTA_GET_STATUS, 0, RTA_STATUS_LEN, &DSPiHub::on_rta_status_,
                         &DSPiHub::on_rta_optional_failed_);
        return;
      }
      const uint8_t ch = rta_next_poll_channel_();
      if (ch == 0xFF) {
        ESP_LOGW(TAG, "RTA has no channels left to poll");
        rta_phase_ = RtaPhase::UNSUPPORTED;
        return;
      }
      // Priority 2 keeps a spectrum poll behind both user commands and the
      // background refresh, and makes it the first thing evicted if the queue
      // fills.  Frames are worthless once stale, so they are never retried.
      enqueue_rta_get_(REQ_RTA_GET_BANDS, ch, RTA_BAND_FRAME_LEN, &DSPiHub::on_rta_bands_,
                       &DSPiHub::on_rta_bands_failed_, /*priority=*/2, /*max_retries=*/0);
      return;
    }
  }
}

void DSPiHub::on_rta_caps_(const uint8_t *data, uint16_t len) {
  RtaCaps caps{};
  if (!rta_parse_caps(data, len, &caps)) {
    ESP_LOGW(TAG, "Short RTA caps response (%u bytes)", len);
    rta_advance_(RtaPhase::NEED_CAPS, rta_interval_ms_);
    return;
  }

  if (caps.version != RTA_CFG_VERSION) {
    // No best-effort parse.  V3 renumbered the band-frame indices relative to
    // V2, so guessing would draw a plausible but wrong spectrum -- worse than
    // drawing none.
    ESP_LOGE(TAG, "RTA protocol version %u is not supported (need %u)", caps.version, RTA_CFG_VERSION);
    rta_phase_ = RtaPhase::UNSUPPORTED;
    rta_busy_ = false;
    return;
  }

  rta_caps_ = caps;
  rta_caps_valid_ = true;
  if (rta_caps_.max_bands > RTA_MAX_BANDS) {
    ESP_LOGW(TAG, "Device reports %u bands, clamping to %u", rta_caps_.max_bands, RTA_MAX_BANDS);
    rta_caps_.max_bands = RTA_MAX_BANDS;
  }
  ESP_LOGD(TAG, "RTA caps: %u bands (%u continuous bass), level zero %u, order %u-%u, %u in / %u out",
           rta_caps_.max_bands, rta_caps_.bass_bands, rta_caps_.level_zero, rta_caps_.fft_order_min,
           rta_caps_.fft_order_max, rta_caps_.input_channels, rta_caps_.output_channels);

  // The device masks channel bits it does not have and only refuses outright
  // when nothing is left, so an out-of-range bit would otherwise vanish in
  // silence.
  const uint8_t width = rta_cfg_desired_.tap == RTA_TAP_OUTPUT ? rta_caps_.output_channels : rta_caps_.input_channels;
  const uint16_t valid = width >= 16 ? 0xFFFF : static_cast<uint16_t>((1u << width) - 1u);
  if (rta_cfg_desired_.channel_mask & ~valid) {
    ESP_LOGW(TAG, "RTA channel_mask 0x%04X exceeds the %u channels at this tap; the device will ignore the extra bits",
             rta_cfg_desired_.channel_mask, width);
  }
  if (rta_cfg_desired_.fft_order < rta_caps_.fft_order_min || rta_cfg_desired_.fft_order > rta_caps_.fft_order_max) {
    ESP_LOGW(TAG, "RTA fft_order %u is outside the device's %u-%u; using %u", rta_cfg_desired_.fft_order,
             rta_caps_.fft_order_min, rta_caps_.fft_order_max, rta_caps_.fft_order_default);
    rta_cfg_desired_.fft_order = rta_caps_.fft_order_default;
  }

  rta_centres_have_ = 0;
  rta_centre_chunk_ = 1;
  rta_advance_(rta_caps_.max_bands ? RtaPhase::NEED_CENTRES : RtaPhase::READ_CONFIG);
}

void DSPiHub::on_rta_centres_(const uint8_t *data, uint16_t len) {
  const uint8_t want = rta_caps_.max_bands;
  const uint16_t got = static_cast<uint16_t>(len / 2);
  for (uint16_t i = 0; i < got && rta_centres_have_ < want && rta_centres_have_ < RTA_MAX_BANDS; i++) {
    rta_band_centre_hz_[rta_centres_have_++] = rta_rd_u16(data, static_cast<uint16_t>(i * 2));
  }

  // Stop on having collected what caps promised, rather than on the first
  // chunk that errors: a chunk past the end is refused, and treating that as
  // the end of the table would hide a genuine fault behind a normal condition.
  if (rta_centres_have_ >= want || got == 0) {
    ESP_LOGD(TAG, "RTA band centres: %u of %u (%u Hz .. %u Hz)", rta_centres_have_, want,
             rta_centres_have_ ? rta_band_centre_hz_[0] : 0,
             rta_centres_have_ ? rta_band_centre_hz_[rta_centres_have_ - 1] : 0);
    rta_advance_(RtaPhase::READ_CONFIG);
    return;
  }
  rta_centre_chunk_++;
  rta_advance_(RtaPhase::NEED_CENTRES);
}

void DSPiHub::on_rta_config_(const uint8_t *data, uint16_t len) {
  // Reached from three phases: the initial read, the response to our write,
  // and the verify read.  A SET response carries no payload, so a short
  // response here simply means "go and read it back".
  if (rta_phase_ == RtaPhase::SET_CONFIG) {
    rta_advance_(RtaPhase::VERIFY_CONFIG);
    return;
  }

  RtaConfig applied{};
  if (!rta_parse_config(data, len, &applied)) {
    ESP_LOGW(TAG, "Short RTA config response (%u bytes)", len);
    rta_advance_(RtaPhase::READ_CONFIG, rta_interval_ms_);
    return;
  }
  rta_cfg_applied_ = applied;

  const bool matches = applied.tap == rta_cfg_desired_.tap && applied.channel_mask == rta_cfg_desired_.channel_mask &&
                       applied.fft_order == rta_cfg_desired_.fft_order && applied.flags == rta_cfg_desired_.flags;

  if (!matches) {
    if (rta_phase_ == RtaPhase::VERIFY_CONFIG) {
      // The device clamps avg_ms and peak_decay_db_s rather than rejecting
      // them, so a difference in those is legitimate and already excluded
      // above.  A difference in the rest is not, and matters: its boot default
      // is every output channel, which still draws, just far too slowly.
      ESP_LOGE(TAG,
               "RTA config was not applied: asked for tap %u mask 0x%04X order %u, device reports tap %u mask 0x%04X "
               "order %u",
               rta_cfg_desired_.tap, rta_cfg_desired_.channel_mask, rta_cfg_desired_.fft_order, applied.tap,
               applied.channel_mask, applied.fft_order);
    } else {
      rta_advance_(RtaPhase::SET_CONFIG);
      return;
    }
  }

  // Poll only the channels the device actually accepted.
  rta_poll_mask_ = applied.channel_mask;
  rta_poll_cursor_ = 0;
  rta_seq_valid_ = 0;
  rta_status_due_ = millis() + rta_status_interval_ms_;
  ESP_LOGD(TAG, "RTA streaming: tap %u, mask 0x%04X, order %u, avg %u ms, peak decay %u dB/s", applied.tap,
           applied.channel_mask, applied.fft_order, applied.avg_ms, applied.peak_decay_db_s);
  rta_advance_(RtaPhase::STREAMING);
}

void DSPiHub::on_rta_bands_(const uint8_t *data, uint16_t len) {
  rta_busy_ = false;
  rta_next_due_ = millis() + rta_interval_ms_;

  RtaBandFrame frame{};
  if (!rta_parse_band_frame(data, len, &frame)) {
    ESP_LOGW(TAG, "Bad RTA band frame (%u bytes, version %u)", len, len ? data[0] : 0);
    rta_dropped_++;
    return;
  }

  // The engine reports zero bands while it is idle: its band table is only
  // built once it starts, and the read that started it returns the frame from
  // before that happened.  Publishing it would blank every bar for a frame.
  if (frame.n_bands == 0) {
    return;
  }

  const uint8_t ch = frame.channel;
  const bool stale = frame.age_ms == RTA_AGE_NEVER || frame.age_ms > rta_stale_ms_;
  if (stale) {
    rta_mark_not_live_();
    return;
  }

  const bool seq_known = ch < 16 && (rta_seq_valid_ & (1u << ch));
  if (seq_known && rta_last_seq_[ch] == frame.seq && rta_live_) {
    // Same frame we already published: the poll interval simply beat the
    // device's frame rate.  Not an error, and not a redraw.
    return;
  }
  if (ch < 16) {
    rta_last_seq_[ch] = frame.seq;
    rta_seq_valid_ |= static_cast<uint16_t>(1u << ch);
  }

  rta_frames_++;
  rta_live_ = true;
  rta_last_update_.frame = frame;
  rta_last_update_.level_zero = rta_caps_.level_zero;
  rta_last_update_.live = true;
  rta_band_frame_callback_.call(rta_last_update_);
}

void DSPiHub::on_rta_status_(const uint8_t *data, uint16_t len) {
  rta_busy_ = false;
  rta_next_due_ = millis();

  if (!rta_parse_status(data, len, &rta_status_)) {
    ESP_LOGW(TAG, "Short RTA status response (%u bytes)", len);
    return;
  }

  // The analyser is one engine with one global config, shared by every client
  // on every transport. If a USB host reconfigures it, we see its settings
  // rather than ours.
  //
  // Do not "correct" that. An earlier version reapplied our config whenever
  // the tap differed, which against a running DSPi-Console turned into a
  // config war: each side rewrote the other's tap every couple of seconds,
  // restarting the frame and clearing the averaging each time. There is no way
  // to distinguish another client's deliberate change from a device that
  // rebooted and lost our settings, and of the two, fighting is much the worse
  // failure. A reboot is caught anyway by the link dropping and the probe
  // re-running, which resets this state machine from the top.
  //
  // So adapt instead: report it once, and draw whatever the device is
  // actually analysing.
  if (rta_status_.tap != rta_cfg_desired_.tap) {
    if (!rta_foreign_config_) {
      rta_foreign_config_ = true;
      ESP_LOGI(TAG, "RTA is configured by another client (tap %u, we asked for %u); following it", rta_status_.tap,
               rta_cfg_desired_.tap);
    }
  } else if (rta_foreign_config_) {
    rta_foreign_config_ = false;
    ESP_LOGI(TAG, "RTA is back on our own configuration (tap %u)", rta_status_.tap);
  }
}

void DSPiHub::on_rta_control_(const uint8_t *data, uint16_t len) {
  (void) data;
  (void) len;
  rta_busy_ = false;
  if (rta_phase_ == RtaPhase::STOPPING) {
    rta_phase_ = RtaPhase::DISABLED;
  }
}

void DSPiHub::on_rta_setup_failed_(uint8_t status) {
  rta_busy_ = false;
  if (status == CTRL_STATUS_LINK_FAILED) {
    // Never heard back, rather than refused.  The link is the problem, not the
    // feature, so keep the verdict open and try again after a pause.
    ESP_LOGW(TAG, "RTA setup did not complete; retrying");
    rta_next_due_ = millis() + 1000;
    return;
  }
  // A permanent rejection here means the firmware does not implement these
  // opcodes, or refused our config outright.  Retrying either would spin.
  ESP_LOGW(TAG, "RTA unavailable on firmware %u.%u.%u (%s)", fw_major_, fw_minor_, fw_patch_,
           ctrl_status_to_string(status));
  rta_phase_ = RtaPhase::UNSUPPORTED;
}

void DSPiHub::on_rta_bands_failed_(uint8_t status) {
  rta_busy_ = false;
  rta_dropped_++;
  rta_next_due_ = millis() + rta_interval_ms_;

  // A refused channel is one the device does not have at this tap.  Drop it
  // and keep the others rather than stopping the whole display.
  if (status != CTRL_STATUS_LINK_FAILED && !ctrl_status_is_retryable(status)) {
    const uint8_t ch = current_.wvalue < 16 ? static_cast<uint8_t>(current_.wvalue) : 0;
    if (rta_poll_mask_ & (1u << ch)) {
      ESP_LOGW(TAG, "RTA channel %u refused (%s); dropping it", ch, ctrl_status_to_string(status));
      rta_poll_mask_ &= static_cast<uint16_t>(~(1u << ch));
    }
  }
}

void DSPiHub::on_rta_optional_failed_(uint8_t status) {
  // Centres, status and the courtesy stop are all non-fatal: without centres
  // the axis labels degrade, without status a device reboot is noticed late,
  // and a failed stop just means waiting for the device's own idle timeout.
  ESP_LOGD(TAG, "Optional RTA request failed (%s)", ctrl_status_to_string(status));
  rta_busy_ = false;
  switch (rta_phase_) {
    case RtaPhase::NEED_CENTRES:
      ESP_LOGW(TAG, "RTA band centres unavailable; frequency labels will be missing");
      rta_advance_(RtaPhase::READ_CONFIG);
      break;
    case RtaPhase::STOPPING:
      rta_phase_ = RtaPhase::DISABLED;
      break;
    default:
      rta_next_due_ = millis() + rta_interval_ms_;
      break;
  }
}

}  // namespace dspi
}  // namespace esphome
