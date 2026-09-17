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
//
// The largest payload we build is the 12-byte RtaConfig of REQ_RTA_SET_CONFIG;
// 16 leaves a little room and keeps Transaction naturally aligned.  The device
// accepts up to PAYLOAD_MAX = 64 bytes on a non-bulk SET (uart_control.c), so
// this cap is ours, not the wire's.
static constexpr uint8_t MAX_TX_PAYLOAD = 16;
static constexpr uint8_t MAX_TX_FRAME = 1 + 1 + REQ_HEADER_LEN + MAX_TX_PAYLOAD + 2;

// Longest response payload we will accept.  This matches the device's own
// non-bulk GET copy buffer, TX_COPY_MAX == sizeof(CsMacro) == 132
// (uart_control.c), which is the same size as the dispatcher's response copy
// and the I2C one.  Nothing on this transport can legitimately be longer, so
// anything that is, is a desync rather than a frame.
//
// The largest response we actually read is the 82-byte RtaBandFrame; the
// previous value of 80 was sized for a 16-byte I2sSlaveStatusPacket and
// rejected every band frame as OVERSIZE.  Raising it loosens the resync
// heuristic by 52 bytes -- a desync now has to burn that much more before the
// parser gives up on a frame -- which the request timeout covers.
static constexpr uint16_t MAX_RX_PAYLOAD = 132;

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

  // Not a wire status.  Synthesised locally when a transaction ends without
  // any answer at all -- retries exhausted, or the link dropped -- so a
  // failure handler can tell "the device refused this" from "we never heard
  // back".  The first says the command is unsupported; the second says to try
  // again later.
  CTRL_STATUS_LINK_FAILED = 0xFF,
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
    case CTRL_STATUS_LINK_FAILED:
      return "LINK_FAILED";
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
// see commands.md section 1.1).  REQ_RTA_CONTROL below is one of them: it
// starts, stops and resets the analyser, but is handled under
// vendor_handle_get() with the action packed into wValue.  The frame type is
// therefore a property of the opcode rather than of the verb: never infer it
// from "am I setting or getting".  Anything added here later must be checked
// against that list -- REQ_SAVE_MASTER_VOLUME (0xD6) is the nearest trap.
//
// Note also that wLength is a hard cap on a GET, not a hint:
// vendor_dispatch_get() ends with `if (wLength && *resp_len > wLength)
// *resp_len = wLength;`, so asking for fewer bytes than a command returns
// yields a silently truncated payload with a perfectly valid CRC.  Every GET
// below must be issued with the full length of the structure it expects.

enum Opcode : uint8_t {
  // Spectrum analyser (RTA).  Protocol V3; see rta.h and
  // Documentation/Features/spectrum_analyser_spec.md.
  //
  // 0x0C (REQ_RTA_GET_BINS) and 0x0F (REQ_RTA_GET_BANDS_ALL) are deliberately
  // absent.  The bin frame is up to 529 bytes against this transport's
  // 132-byte cap, so it needs offset-chunked reads with seq head/tail
  // revalidation for a product we do not draw; and GET_BANDS_ALL is answered
  // only for a USB host, returning ERROR on UART.
  REQ_RTA_SET_CONFIG = 0x08,  // SET  <- 12-byte RtaConfig; ERROR on invalid
  REQ_RTA_GET_CONFIG = 0x09,  // GET  -> 12-byte RtaConfig, as applied
  REQ_RTA_GET_CAPS = 0x0A,    // GET  wValue 0 -> 16-byte RtaCaps
                              //      wValue 1.. -> band centres, 32x uint16 Hz
  REQ_RTA_GET_BANDS = 0x0B,   // GET  wValue = channel -> 82-byte RtaBandFrame
  REQ_RTA_GET_STATUS = 0x0D,  // GET  -> 24-byte RtaStatus
  REQ_RTA_CONTROL = 0x0E,     // GET  wValue = RTA_CTL_* -> 1 status byte
                              //      (write-as-read, see above)

  // DSP feature toggles.  Each is a plain 1-byte SET with a 1-byte GET and is
  // dispatched on the ordinary SET path, unlike the write-as-read commands
  // above.  See ToggleTarget below, which pairs them up.
  REQ_SET_BYPASS = 0x46,              // SET  <- uint8 0/1 (master EQ bypass)
  REQ_GET_BYPASS = 0x47,              // GET  -> uint8 0/1
  REQ_SET_LOUDNESS = 0x58,            // SET  <- uint8 0/1
  REQ_GET_LOUDNESS = 0x59,            // GET  -> uint8 0/1
  REQ_SET_CROSSFEED = 0x5E,           // SET  <- uint8 0/1
  REQ_GET_CROSSFEED = 0x5F,           // GET  -> uint8 0/1

  REQ_GET_ADAT_INPUT_ENABLE = 0x69,   // GET  -> uint8 0/1
  REQ_GET_ADAT_INPUT_PIN = 0x6B,      // GET  -> uint8 GPIO (0xFF = unset)
  REQ_GET_PLATFORM = 0x7F,            // GET  -> up to 7 bytes, truncatable

  // Presets.  REQ_PRESET_LOAD is one of the write-as-read commands described
  // above: it is dispatched under the firmware's vendor_handle_get() with the
  // slot packed into wValue, so it must be sent as a GET frame even though it
  // mutates the entire DSP state.  Sending it as a SET stalls.
  //
  // The load is also deferred: the firmware sets a pending flag and performs
  // the work later in its main loop behind pipeline_reset_ready(), so the
  // returned byte means "accepted", not "applied".  Confirm with
  // REQ_PRESET_GET_ACTIVE rather than trusting it.
  REQ_PRESET_LOAD = 0x91,        // GET  wValue = slot -> 1 status byte (PRESET_*)
                                 //      (write-as-read, see above)
  REQ_PRESET_GET_NAME = 0x93,    // GET  wValue = slot -> 32 bytes, NUL-padded
  REQ_PRESET_GET_DIR = 0x95,     // GET  -> 7-byte PresetDirectory
  REQ_PRESET_GET_ACTIVE = 0x9A,  // GET  -> uint8 active slot 0..9
  REQ_SET_LEVELLER_ENABLE = 0xB4,     // SET  <- uint8 0/1 (a DSP feature toggle)
  REQ_GET_LEVELLER_ENABLE = 0xB5,     // GET  -> uint8 0/1
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
// Boolean DSP parameters
// ---------------------------------------------------------------------------
//
// The DSPi exposes a family of plain on/off parameters, each with its own SET
// and GET opcode carrying a single 0/1 byte. This enum names the ones this
// component drives and pairs each with its opcodes, so a caller never has to
// hold an opcode and a target at the same time.
//
// The REQ_* constants above carry the firmware's own spellings, and this enum
// is where that stops: EQ_BYPASS and LEVELLER read better than the opcodes'
// BYPASS and LEVELLER_ENABLE, and the mapping below is the one place the two
// naming schemes have to meet. So this enum, toggle_name() and the YAML `type:`
// key all agree with each other, and only the opcodes agree with config.h.
//
// Every one of these is an *ordinary* SET -- their handlers sit above
// vendor_handle_get() in the firmware's vendor_commands.c, so the frame type is
// FRAME_SET_REQ to write and FRAME_GET_REQ to read. That is worth stating in
// this header precisely because it is not true of the whole protocol (see the
// write-as-read warning above REQ_RTA_CONTROL). Anything added here must be
// checked against the firmware rather than assumed to match its neighbours;
// there is deliberately no frame-type column below, because a write-as-read
// opcode does not belong in this family at all.
//
// None of them writes flash and none resets the audio pipeline, so they are
// cheap to poll and cheap to set. They are, however, RAM-only and *per-preset*:
// the firmware stores them in a preset slot and restores them on load, but a
// SET alone does not persist, so a toggle lasts until the device reboots or
// loads a preset.
enum class ToggleTarget : uint8_t {
  USER_MUTE,
  LOUDNESS,
  EQ_BYPASS,
  CROSSFEED,
  LEVELLER,
};

// Number of ToggleTarget values, for iterating the family.
static constexpr uint8_t TOGGLE_COUNT = 5;

// A switch rather than a lookup table: a table at namespace scope in a header
// gets one copy per translation unit and can carry a silent hole, whereas a
// missing enumerator here is a -Wswitch warning, and tests/run.sh compiles with
// -Werror.
inline constexpr uint8_t toggle_set_opcode(ToggleTarget t) {
  switch (t) {
    case ToggleTarget::USER_MUTE:
      return REQ_SET_USER_MUTE;
    case ToggleTarget::LOUDNESS:
      return REQ_SET_LOUDNESS;
    case ToggleTarget::EQ_BYPASS:
      return REQ_SET_BYPASS;
    case ToggleTarget::CROSSFEED:
      return REQ_SET_CROSSFEED;
    case ToggleTarget::LEVELLER:
      return REQ_SET_LEVELLER_ENABLE;
  }
  return 0;
}

inline constexpr uint8_t toggle_get_opcode(ToggleTarget t) {
  switch (t) {
    case ToggleTarget::USER_MUTE:
      return REQ_GET_USER_MUTE;
    case ToggleTarget::LOUDNESS:
      return REQ_GET_LOUDNESS;
    case ToggleTarget::EQ_BYPASS:
      return REQ_GET_BYPASS;
    case ToggleTarget::CROSSFEED:
      return REQ_GET_CROSSFEED;
    case ToggleTarget::LEVELLER:
      return REQ_GET_LEVELLER_ENABLE;
  }
  return 0;
}

// The name each target goes by in logs, dump_config and the YAML `type:` key,
// so a warning about one names the thing a user would edit.
inline const char *toggle_name(ToggleTarget t) {
  switch (t) {
    case ToggleTarget::USER_MUTE:
      return "user_mute";
    case ToggleTarget::LOUDNESS:
      return "loudness";
    case ToggleTarget::EQ_BYPASS:
      return "eq_bypass";
    case ToggleTarget::CROSSFEED:
      return "crossfeed";
    case ToggleTarget::LEVELLER:
      return "leveller";
  }
  return "unknown";
}

// Bit for `target` in a toggle mask. The masks are uint16_t, so TOGGLE_COUNT
// must stay under 16; the static_assert below is the reminder.
inline constexpr uint16_t toggle_bit(ToggleTarget t) { return static_cast<uint16_t>(1u << static_cast<uint8_t>(t)); }

static_assert(TOGGLE_COUNT <= 16, "toggle masks are uint16_t");

// How many targets a toggle mask holds. std::popcount is C++20 and this file
// compiles as C++17 under both the host tests and the ESP toolchain.
inline uint8_t toggle_mask_count(uint16_t mask) {
  uint8_t n = 0;
  while (mask != 0) {
    mask = static_cast<uint16_t>(mask & (mask - 1));
    n++;
  }
  return n;
}

// ---------------------------------------------------------------------------
// Presets
// ---------------------------------------------------------------------------
//
// A preset slot holds a complete DSP state: EQ, crossovers, delays, the matrix,
// output configuration, channel names, and -- when the device's master-volume
// mode is 1 -- master volume too.  A preset is always active; there is no "no
// preset" state, and loading an unoccupied slot applies factory defaults rather
// than failing.
//
// Names live in the directory sector and exist for every slot regardless of
// occupancy.  On a fresh device slot 0 is named "Default" and slots 1..9 are
// empty strings.

// PRESET_SLOTS and PRESET_NAME_LEN in the firmware's config.h.
static constexpr uint8_t PRESET_SLOTS = 10;
static constexpr uint8_t PRESET_NAME_LEN = 32;

// Payload status codes returned by the preset commands.
//
// These are NOT CtrlStatus values.  They occupy their own numbering, which
// agrees with CtrlStatus only on 0x00 meaning OK, and every other value
// collides with an unrelated transport meaning.
//
// Two of those collisions are actively dangerous, which is why this is a
// separate enum rather than a set of loose constants:
//
//   PRESET_ERR_INVALID_SLOT  0x01  ==  CTRL_STATUS_BUSY        (retryable)
//   PRESET_ERR_FLASH_WRITE   0x04  ==  CTRL_STATUS_BULK_LOCKED (retryable)
//
// Passing one of these to ctrl_status_is_retryable() therefore does not merely
// mislabel it, it turns a permanent rejection into an infinite retry loop.
// None of these codes is ever retryable: the transport already succeeded by
// the time one is read, and the answer will not change on a second attempt.
// test_framing.cpp pins the collisions so this cannot rot silently.
enum PresetStatus : uint8_t {
  PRESET_OK = 0x00,
  PRESET_ERR_INVALID_SLOT = 0x01,
  PRESET_ERR_SLOT_EMPTY = 0x02,
  PRESET_ERR_CRC = 0x03,
  PRESET_ERR_FLASH_WRITE = 0x04,
};

inline const char *preset_status_to_string(uint8_t status) {
  switch (status) {
    case PRESET_OK:
      return "OK";
    case PRESET_ERR_INVALID_SLOT:
      return "INVALID_SLOT";
    case PRESET_ERR_SLOT_EMPTY:
      return "SLOT_EMPTY";
    case PRESET_ERR_CRC:
      return "CRC";
    case PRESET_ERR_FLASH_WRITE:
      return "FLASH_WRITE";
    default:
      return "UNKNOWN";
  }
}

// REQ_PRESET_GET_DIR (0x95) response, 7 bytes.
struct PresetDirectory {
  uint16_t slot_occupied{0};  // bit N set = slot N holds user data
  uint8_t startup_mode{0};    // 0 = load default_slot, 1 = load last_active
  uint8_t default_slot{0};
  uint8_t last_active_slot{0};
  uint8_t output_config_mode{0};  // 0 = device-global, 1 = travels with preset
  uint8_t master_volume_mode{0};  // 0 = device-global, 1 = travels with preset

  bool is_occupied(uint8_t slot) const {
    return slot < PRESET_SLOTS && (slot_occupied & (1u << slot)) != 0;
  }
};

static constexpr uint16_t PRESET_DIR_LEN = 7;

inline bool parse_preset_directory(const uint8_t *data, uint16_t len, PresetDirectory *out) {
  if (len < PRESET_DIR_LEN)
    return false;
  out->slot_occupied = static_cast<uint16_t>(data[0] | (data[1] << 8));
  out->startup_mode = data[2];
  out->default_slot = data[3];
  out->last_active_slot = data[4];
  out->output_config_mode = data[5];
  out->master_volume_mode = data[6];
  return true;
}

// Length of a 32-byte NUL-padded device name.
//
// The firmware guarantees termination when a name is set through its own API,
// but a full 32 characters leaves no room for the NUL, so the length is capped
// at the field width rather than trusting a terminator to be there.
inline uint8_t preset_name_length(const uint8_t *data, uint16_t len) {
  const uint16_t limit = len < PRESET_NAME_LEN ? len : PRESET_NAME_LEN;
  uint16_t n = 0;
  while (n < limit && data[n] != 0)
    n++;
  return static_cast<uint8_t>(n);
}

// ---------------------------------------------------------------------------
// Spectrum analyser (RTA)
// ---------------------------------------------------------------------------
//
// The DSPi runs one FFT engine that rotates over a selected set of channels at
// a selected tap, and publishes a third-octave band table per channel.  It is
// transient by design: off at boot, never persisted, not part of presets, and
// it stops itself RTA_IDLE_TIMEOUT_MS after the last band read.  Any band read
// starts it again, so a client that simply polls needs no explicit start.
//
// Names mirror firmware/DSPi/rta.h and rta_fft.h exactly.

static constexpr uint8_t RTA_CFG_VERSION = 3;
static constexpr uint16_t RTA_IDLE_TIMEOUT_MS = 5000;

static constexpr uint8_t RTA_TAP_INPUT = 0;   // after per-input PEQ, before the matrix
static constexpr uint8_t RTA_TAP_OUTPUT = 1;  // after gain + delay, before encode

// CONTROL owns run state: no auto-start, no auto-off.  We never set this -- see
// the note on set_rta_enabled() in dspi.h for why.
static constexpr uint8_t RTA_FLAG_MANUAL = 0x01;

static constexpr uint8_t RTA_STATE_IDLE = 0;
static constexpr uint8_t RTA_STATE_CAPTURING = 1;
static constexpr uint8_t RTA_STATE_TRANSFORMING = 2;

static constexpr uint16_t RTA_CTL_STOP = 0;
static constexpr uint16_t RTA_CTL_START = 1;
static constexpr uint16_t RTA_CTL_RESET_AVG = 2;

static constexpr uint8_t RTA_ORDER_MIN = 8;   // 256 points
static constexpr uint8_t RTA_ORDER_MAX = 10;  // 1024 points
static constexpr uint8_t RTA_MAX_BANDS = 37;

// Level bytes are 0.5 dB per step with RTA_LEVEL_ZERO_DBFS meaning 0 dBFS, so
// 255 is +6 dBFS.  The zero point is also reported in RtaCaps and clients are
// expected to take it from there rather than hard-coding this; it exists as a
// fallback for the window before caps have been read.
static constexpr uint8_t RTA_LEVEL_ZERO_DBFS = 243;

// A level byte of 0 is "floor": no data, or below the bottom of the scale.  It
// is not -121.5 dB of signal and must not be rendered as one.
static constexpr uint8_t RTA_LEVEL_FLOOR = 0;

// RtaBandFrame.age_ms when the channel has never published a frame.
static constexpr uint16_t RTA_AGE_NEVER = 0xFFFF;

// Wire lengths.  Every GET must ask for exactly these (see the wLength note on
// the opcode list).
static constexpr uint16_t RTA_CONFIG_LEN = 12;
static constexpr uint16_t RTA_CAPS_LEN = 16;
static constexpr uint16_t RTA_BAND_FRAME_LEN = 8 + 2 * RTA_MAX_BANDS;  // 82
static constexpr uint16_t RTA_STATUS_LEN = 24;

// Band-centre chunks: 32 uint16 Hz values each, requested with wValue 1..
static constexpr uint16_t RTA_CENTRES_PER_CHUNK = 32;
static constexpr uint16_t RTA_CENTRES_CHUNK_LEN = RTA_CENTRES_PER_CHUNK * 2;

// Byte offsets into RtaBandFrame.  Protocol V3 inserted a two-byte reserved
// field after age_ms, shifting both level arrays by three relative to V2, so
// these are pinned by test rather than trusted by eye.
static constexpr uint16_t RTA_BF_OFF_VERSION = 0;
static constexpr uint16_t RTA_BF_OFF_CHANNEL = 1;
static constexpr uint16_t RTA_BF_OFF_SEQ = 2;
static constexpr uint16_t RTA_BF_OFF_N_BANDS = 3;
static constexpr uint16_t RTA_BF_OFF_AGE_MS = 4;
static constexpr uint16_t RTA_BF_OFF_AVG = 8;
static constexpr uint16_t RTA_BF_OFF_PEAK = RTA_BF_OFF_AVG + RTA_MAX_BANDS;  // 45

static_assert(MAX_TX_PAYLOAD >= RTA_CONFIG_LEN, "TX payload buffer cannot hold an RtaConfig");
static_assert(MAX_RX_PAYLOAD >= RTA_BAND_FRAME_LEN, "RX payload buffer cannot hold an RtaBandFrame");
static_assert(MAX_RX_PAYLOAD >= RTA_CENTRES_CHUNK_LEN, "RX payload buffer cannot hold a band-centre chunk");

// Host-side mirrors of the device's packed wire structures.  These are plain
// structs, not overlays: every field is assembled from explicit byte offsets by
// the parse helpers below, so nothing here depends on the compiler's layout.

struct RtaConfig {
  uint8_t version{RTA_CFG_VERSION};
  uint8_t tap{RTA_TAP_INPUT};
  uint16_t channel_mask{0};
  uint8_t fft_order{RTA_ORDER_MAX};
  uint16_t avg_ms{0};
  uint8_t peak_decay_db_s{0};
  uint8_t flags{0};
};

struct RtaCaps {
  uint8_t version{0};
  uint8_t input_channels{0};
  uint8_t output_channels{0};
  uint8_t fft_order_min{0};
  uint8_t fft_order_max{0};
  uint8_t fft_order_default{0};
  uint8_t bass_bands{0};  // continuous bands, starting at index 0
  uint8_t max_bands{0};
  uint8_t level_zero{RTA_LEVEL_ZERO_DBFS};
  uint8_t dynamic_range_db{0};
  uint16_t idle_timeout_ms{0};
  uint16_t max_bin_frame{0};
  uint16_t bass_dynamic_range_db{0};
};

struct RtaBandFrame {
  uint8_t version{0};
  uint8_t channel{0};
  uint8_t seq{0};
  uint8_t n_bands{0};
  uint16_t age_ms{RTA_AGE_NEVER};
  uint8_t avg[RTA_MAX_BANDS]{};
  uint8_t peak[RTA_MAX_BANDS]{};
};

struct RtaStatus {
  uint8_t version{0};
  uint8_t state{RTA_STATE_IDLE};
  uint8_t tap{RTA_TAP_INPUT};
  uint8_t channel{0};
  uint8_t live_count{0};
  uint16_t live_mask{0};
  uint16_t frames_per_s{0};
  uint16_t busy_us_per_s{0};
  uint16_t last_frame_us{0};
  uint16_t idle_ms{0};
  uint32_t sample_rate_hz{0};
  uint8_t first_band{0};
  uint16_t bass_busy_us_per_s{0};
};

inline uint16_t rta_rd_u16(const uint8_t *d, uint16_t off) {
  return static_cast<uint16_t>(d[off] | (static_cast<uint16_t>(d[off + 1]) << 8));
}

inline uint32_t rta_rd_u32(const uint8_t *d, uint16_t off) {
  return static_cast<uint32_t>(d[off]) | (static_cast<uint32_t>(d[off + 1]) << 8) |
         (static_cast<uint32_t>(d[off + 2]) << 16) | (static_cast<uint32_t>(d[off + 3]) << 24);
}

inline void rta_wr_u16(uint8_t *d, uint16_t off, uint16_t v) {
  d[off] = static_cast<uint8_t>(v & 0xFF);
  d[off + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

// Serialises into exactly RTA_CONFIG_LEN bytes.  Reserved bytes are zeroed.
inline void rta_write_config(uint8_t *out, const RtaConfig &cfg) {
  for (uint16_t i = 0; i < RTA_CONFIG_LEN; i++)
    out[i] = 0;
  out[0] = cfg.version;
  out[1] = cfg.tap;
  rta_wr_u16(out, 2, cfg.channel_mask);
  out[4] = cfg.fft_order;
  // out[5] is reserved0
  rta_wr_u16(out, 6, cfg.avg_ms);
  out[8] = cfg.peak_decay_db_s;
  out[9] = cfg.flags;
  // out[10..11] are reserved
}

inline bool rta_parse_config(const uint8_t *d, uint16_t len, RtaConfig *out) {
  if (len < RTA_CONFIG_LEN)
    return false;
  out->version = d[0];
  out->tap = d[1];
  out->channel_mask = rta_rd_u16(d, 2);
  out->fft_order = d[4];
  out->avg_ms = rta_rd_u16(d, 6);
  out->peak_decay_db_s = d[8];
  out->flags = d[9];
  return true;
}

inline bool rta_parse_caps(const uint8_t *d, uint16_t len, RtaCaps *out) {
  if (len < RTA_CAPS_LEN)
    return false;
  out->version = d[0];
  out->input_channels = d[1];
  out->output_channels = d[2];
  out->fft_order_min = d[3];
  out->fft_order_max = d[4];
  out->fft_order_default = d[5];
  out->bass_bands = d[6];
  out->max_bands = d[7];
  out->level_zero = d[8];
  out->dynamic_range_db = d[9];
  out->idle_timeout_ms = rta_rd_u16(d, 10);
  out->max_bin_frame = rta_rd_u16(d, 12);
  out->bass_dynamic_range_db = rta_rd_u16(d, 14);
  return true;
}

// Rejects anything that is not a full V3 frame.  V3 is deliberately
// incompatible with V2 -- the frame grew from 80 to 82 bytes and both level
// arrays shifted by three -- so a V2 frame parsed here would yield a plausible
// but wrong spectrum.  Refusing it is the only safe answer.
inline bool rta_parse_band_frame(const uint8_t *d, uint16_t len, RtaBandFrame *out) {
  if (len < RTA_BAND_FRAME_LEN)
    return false;
  if (d[RTA_BF_OFF_VERSION] != RTA_CFG_VERSION)
    return false;
  out->version = d[RTA_BF_OFF_VERSION];
  out->channel = d[RTA_BF_OFF_CHANNEL];
  out->seq = d[RTA_BF_OFF_SEQ];
  out->n_bands = d[RTA_BF_OFF_N_BANDS];
  if (out->n_bands > RTA_MAX_BANDS)
    out->n_bands = RTA_MAX_BANDS;
  out->age_ms = rta_rd_u16(d, RTA_BF_OFF_AGE_MS);
  for (uint16_t i = 0; i < RTA_MAX_BANDS; i++) {
    out->avg[i] = d[RTA_BF_OFF_AVG + i];
    out->peak[i] = d[RTA_BF_OFF_PEAK + i];
  }
  return true;
}

inline bool rta_parse_status(const uint8_t *d, uint16_t len, RtaStatus *out) {
  if (len < RTA_STATUS_LEN)
    return false;
  out->version = d[0];
  out->state = d[1];
  out->tap = d[2];
  out->channel = d[3];
  // d[4] is reserved0
  out->live_count = d[5];
  out->live_mask = rta_rd_u16(d, 6);
  out->frames_per_s = rta_rd_u16(d, 8);
  out->busy_us_per_s = rta_rd_u16(d, 10);
  out->last_frame_us = rta_rd_u16(d, 12);
  out->idle_ms = rta_rd_u16(d, 14);
  out->sample_rate_hz = rta_rd_u32(d, 16);
  out->first_band = d[20];
  // d[21] is reserved1
  out->bass_busy_us_per_s = rta_rd_u16(d, 22);
  return true;
}

inline bool rta_level_is_floor(uint8_t v) { return v == RTA_LEVEL_FLOOR; }

// Only meaningful for a non-floor byte; a floor byte means "no signal here",
// which has no dB value.  Callers that render should use rta_level_to_fraction.
inline float rta_level_to_dbfs(uint8_t v, uint8_t level_zero) {
  return (static_cast<float>(v) - static_cast<float>(level_zero)) * 0.5f;
}

// Maps a level byte onto 0.0 .. 1.0 across [floor_db, top_db], clamped at both
// ends, with a floor byte pinned to exactly 0.0 however low floor_db is.
//
// This is the one piece of arithmetic every consumer needs and the one place a
// sign error would be invisible on screen, so it lives here and is tested on
// the host rather than being rewritten in each caller's lambda.
inline float rta_level_to_fraction(uint8_t v, uint8_t level_zero, float floor_db, float top_db = 0.0f) {
  if (rta_level_is_floor(v))
    return 0.0f;
  const float span = top_db - floor_db;
  if (span <= 0.0f)
    return 0.0f;
  const float f = (rta_level_to_dbfs(v, level_zero) - floor_db) / span;
  if (f <= 0.0f)
    return 0.0f;
  if (f >= 1.0f)
    return 1.0f;
  return f;
}

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
