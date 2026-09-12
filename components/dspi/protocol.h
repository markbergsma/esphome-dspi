#pragma once

#include <cstdint>

// Wire-protocol constants for the DSPi external control interface.
//
// Source of truth: Documentation/Features/control_interfaces_spec.md in the
// DSPi firmware repository (wire-protocol version 1), with opcodes verified
// against firmware/DSPi/config.h.
//
// This header has no ESPHome dependencies so the unit tests can compile it
// with a host compiler.

namespace esphome {
namespace dspi {

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------

static constexpr uint8_t SYNC_BYTE = 0xA5;

// Frame type byte, immediately after the sync byte.
enum FrameType : uint8_t {
  FRAME_SET_REQ = 0x01,   // host -> device, payload follows the header
  FRAME_GET_REQ = 0x02,   // host -> device, no payload; wLen caps the response
  FRAME_NOTIFY = 0x40,    // device -> host, unsolicited (opt-in)
  FRAME_SET_RESP = 0x81,  // device -> host, len always 0
  FRAME_GET_RESP = 0x82,  // device -> host, payload present when status == OK
};

// Bytes between the type byte and the payload, in both directions:
// requests carry bReq + wValue + wIndex + wLength; responses carry
// status + len.  A notification reuses the response shape with a fixed 0x00
// in the status slot.
static constexpr uint8_t REQ_HEADER_LEN = 7;
static constexpr uint8_t RESP_HEADER_LEN = 3;

// Longest frame we will build: sync + type + 7 header + payload + 2 CRC.
static constexpr uint8_t MAX_TX_PAYLOAD = 8;
static constexpr uint8_t MAX_TX_FRAME = 1 + 1 + REQ_HEADER_LEN + MAX_TX_PAYLOAD + 2;

// Longest response payload we will accept.  The largest response any command
// in our opcode set returns is 16 bytes (I2sSlaveStatusPacket); notification
// packets top out at 12 + 52.  Anything longer is a desync, not a frame.
static constexpr uint16_t MAX_RX_PAYLOAD = 80;

// ---------------------------------------------------------------------------
// Status codes (byte 0 of every SET/GET response)
// ---------------------------------------------------------------------------

enum CtrlStatus : uint8_t {
  CTRL_STATUS_OK = 0x00,
  CTRL_STATUS_BUSY = 0x01,         // retry: a USB control SET was in flight
  CTRL_STATUS_ERROR = 0x02,        // permanent: unknown command or bad parameter
  CTRL_STATUS_BLOCKED = 0x03,      // permanent: USB-only command over this transport
  CTRL_STATUS_BULK_LOCKED = 0x04,  // retry: another transport holds the bulk buffer
  CTRL_STATUS_CRC_ERROR = 0x05,    // retry: the device rejected our CRC
  CTRL_STATUS_OVERSIZE = 0x06,     // permanent: non-bulk payload exceeded 64 bytes
  CTRL_STATUS_FRAME_ERROR = 0x07,  // permanent: malformed or truncated frame
};

// True for statuses where resending the identical request may succeed.
//
// The distinction matters: retrying a permanent failure is an infinite
// busy-loop.  BLOCKED in particular means a programming error (a USB-only
// command was sent over UART) and will never succeed however often it is sent.
inline bool ctrl_status_is_retryable(uint8_t status) {
  return status == CTRL_STATUS_BUSY || status == CTRL_STATUS_BULK_LOCKED || status == CTRL_STATUS_CRC_ERROR;
}

inline const char *ctrl_status_to_string(uint8_t status) {
  switch (status) {
    case CTRL_STATUS_OK:
      return "OK";
    case CTRL_STATUS_BUSY:
      return "BUSY";
    case CTRL_STATUS_ERROR:
      return "ERROR";
    case CTRL_STATUS_BLOCKED:
      return "BLOCKED";
    case CTRL_STATUS_BULK_LOCKED:
      return "BULK_LOCKED";
    case CTRL_STATUS_CRC_ERROR:
      return "CRC_ERROR";
    case CTRL_STATUS_OVERSIZE:
      return "OVERSIZE";
    case CTRL_STATUS_FRAME_ERROR:
      return "FRAME_ERROR";
    default:
      return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------------
// Opcodes
// ---------------------------------------------------------------------------
//
// A handful of mutating DSPi opcodes are dispatched on the device's GET path
// and must be sent as GET-type frames despite being writes ("write-as-read",
// see commands.md section 1.1).  None of the opcodes below are affected, but
// the frame type is therefore a property of the opcode rather than of the
// verb: never infer it from "am I setting or getting".  Anything added here
// later must be checked against that list -- REQ_SAVE_MASTER_VOLUME (0xD6) is
// the nearest trap.

enum Opcode : uint8_t {
  REQ_GET_ADAT_INPUT_ENABLE = 0x69,   // GET  -> uint8 0/1
  REQ_GET_ADAT_INPUT_PIN = 0x6B,      // GET  -> uint8 GPIO (0xFF = unset)
  REQ_GET_PLATFORM = 0x7F,            // GET  -> up to 7 bytes, truncatable
  REQ_SET_MASTER_VOLUME = 0xD2,       // SET  <- float32 dB
  REQ_GET_MASTER_VOLUME = 0xD3,       // GET  -> float32 dB
  REQ_SET_USER_VOLUME = 0xDA,         // SET  <- float32 dB
  REQ_GET_USER_VOLUME = 0xDB,         // GET  -> float32 dB
  REQ_SET_USER_MUTE = 0xDC,           // SET  <- uint8 0/1
  REQ_GET_USER_MUTE = 0xDD,           // GET  -> uint8 0/1
  REQ_SET_INPUT_SOURCE = 0xE0,        // SET  <- uint8 InputSource
  REQ_GET_INPUT_SOURCE = 0xE1,        // GET  -> uint8 InputSource (active, not pending)
  REQ_GET_SPDIF_INPUT_CONFIG = 0xEF,  // GET  -> count, enable mask, one GPIO per input
  REQ_GET_UART_CONFIG = 0xF6,         // GET  -> 8-byte UartCtrlConfig
  REQ_GET_CTRL_IFACE_STATUS = 0xF9,   // GET  -> 8-byte CtrlIfaceStatus
};

// The DSPi has two independent gain stages that multiply together, and they are
// not interchangeable:
//
//   final = sample * output_gain * (user_volume * master_volume)
//
// MASTER volume is a device-side output ceiling applied at the very end of the
// signal chain. It is a configuration setting -- "how loud is this system ever
// allowed to get" -- not something to adjust while listening.
//
// USER volume is the same field the USB host's volume slider drives (the
// firmware's comment on 0xDA reads "Vendor-channel user-perceived volume. Same
// field as the UAC1 host slider"), and is what a listener actually turns. It
// also feeds loudness compensation, so the equal-loudness contour tracks it;
// master volume deliberately does not affect that. This is the one to bind to a
// knob or to a media player's volume.
//
// The names here follow the firmware's own opcode names so this file can be read
// against config.h without translation, and that name is carried through to the
// YAML `type:` key. master_volume_spec.md calls 0xDA/0xDB "USB host volume" in
// prose when describing the USB path; that is the same field.
//
// See Documentation/Features/master_volume_spec.md in the firmware repository.

// Master volume: [-127, 0] dB, with -128 reserved as a true -inf mute sentinel
// (MASTER_VOL_MUTE_DB in the firmware).
static constexpr float MASTER_VOLUME_MIN_DB = -127.0f;
static constexpr float MASTER_VOLUME_MAX_DB = 0.0f;
static constexpr float MASTER_VOLUME_MUTE_DB = -128.0f;

// User volume: the firmware clamps to [-CENTER_VOLUME_INDEX, 0], and
// CENTER_VOLUME_INDEX is 60. There is no mute sentinel here; mute is a separate
// command (0xDC).
static constexpr float USER_VOLUME_MIN_DB = -60.0f;
static constexpr float USER_VOLUME_MAX_DB = 0.0f;

// Input source enum, as carried by 0xE0 / 0xE1.
//
// Exactly one source runs at a time, and that is a hardware constraint rather
// than a policy: the S/PDIF RX FIFO, the I2S RX rings and the ADAT ring overlay
// one memory arena as a union, and the four S/PDIF inputs additionally share a
// single PIO state machine, so only the active one claims its GPIO. Selecting a
// source is therefore closer to a source button on an AV receiver than to a
// mixer input -- the matrix mixer sits downstream and routes whichever channels
// the active source provides.
//
// Channel counts differ by source: USB and I2S carry 2/4/6/8, each S/PDIF input
// carries 2, and ADAT carries 8.
enum InputSource : uint8_t {
  INPUT_SOURCE_USB = 0,
  INPUT_SOURCE_SPDIF = 1,  // S/PDIF input 1, always present
  INPUT_SOURCE_I2S = 2,
  INPUT_SOURCE_ADAT = 3,     // RP2350 only, and off until enabled
  INPUT_SOURCE_SPDIF2 = 4,   // optional, off until enabled
  INPUT_SOURCE_SPDIF3 = 5,   // optional, off until enabled
  INPUT_SOURCE_SPDIF4 = 6,   // optional, off until enabled
  INPUT_SOURCE_COUNT = 7,
};

// Number of S/PDIF inputs the firmware defines. Index 0 is INPUT_SOURCE_SPDIF
// and is always enabled; indices 1..3 map to SPDIF2..SPDIF4 and are optional.
static constexpr uint8_t SPDIF_RX_NUM_INPUTS = 4;

// ---------------------------------------------------------------------------
// Notification packets (v2), carried verbatim inside a 0x40 frame
// ---------------------------------------------------------------------------
//
// Every v2 packet opens with { version, event_id, flags, seq }.  We decode
// only that header: PARAM_CHANGED addresses its field by byte offset into the
// firmware's WireBulkParams struct, which is version-coupled and has already
// drifted substantially between firmware releases.  Rather than track those
// offsets, any parameter-affecting event is treated as a hint that something
// changed, and triggers a small targeted re-read of the values we publish.
// That costs a few dozen bytes per event and cannot go stale.

static constexpr uint8_t NOTIFY_VERSION_V2 = 0x02;
static constexpr uint8_t NOTIFY_HEADER_LEN = 4;

enum NotifyEvent : uint8_t {
  NOTIFY_EVT_IDLE = 0x00,              // keep-alive, discard
  NOTIFY_EVT_MASTER_VOLUME = 0x01,     // v1 legacy, never sent over UART
  NOTIFY_EVT_PARAM_CHANGED = 0x02,     // a parameter changed
  NOTIFY_EVT_BULK_INVALIDATED = 0x03,  // many changed; host should re-sync
  NOTIFY_EVT_PRESET_LOADED = 0x04,     // followed by BULK_INVALIDATED
  NOTIFY_EVT_INPUT_FORMAT = 0x05,      // input channel count changed
};

// True if this event means state we publish may have changed.  Unknown event
// IDs deliberately return false: the enum is actively growing in the firmware
// (0x07..0x0C are already assigned), and treating an unrecognised event as an
// error, or as a reason to re-read, would be wrong in both directions.
inline bool notify_event_affects_params(uint8_t event_id) {
  return event_id == NOTIFY_EVT_PARAM_CHANGED || event_id == NOTIFY_EVT_BULK_INVALIDATED ||
         event_id == NOTIFY_EVT_PRESET_LOADED;
}

}  // namespace dspi
}  // namespace esphome
