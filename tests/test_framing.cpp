// Host-compiled tests for the DSPi wire framing.
//
// These cover the two things that are expensive to debug on hardware: the
// CRC/frame layout, and the parser's behaviour on noisy or truncated input.
// Run with tests/run.sh, or:
//
//   c++ -std=c++17 -I components/dspi -o /tmp/t tests/test_framing.cpp && /tmp/t

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "crc16.h"
#include "framing.h"
#include "protocol.h"

using namespace esphome::dspi;

static int failures = 0;

static void check(bool ok, const std::string &what) {
  if (!ok) {
    failures++;
    std::printf("  FAIL  %s\n", what.c_str());
  } else {
    std::printf("  ok    %s\n", what.c_str());
  }
}

static std::vector<uint8_t> unhex(const std::string &s) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i < s.size(); i++) {
    if (s[i] == ' ')
      continue;
    out.push_back(static_cast<uint8_t>(std::stoul(s.substr(i, 2), nullptr, 16)));
    i++;
  }
  return out;
}

static std::string hex(const uint8_t *d, size_t n) {
  std::string s;
  char buf[4];
  for (size_t i = 0; i < n; i++) {
    std::snprintf(buf, sizeof(buf), "%02X", d[i]);
    if (i)
      s += ' ';
    s += buf;
  }
  return s;
}

// ---------------------------------------------------------------------------

static void test_crc_reference() {
  std::printf("CRC16-CCITT-FALSE\n");
  // The check value the spec gives for validating a client before wiring it
  // to hardware.
  const char *s = "123456789";
  check(crc16(reinterpret_cast<const uint8_t *>(s), 9) == 0x29B1, "crc16(\"123456789\") == 0x29B1");

  // Incremental folding must agree with the one-shot form, since the parser
  // and the frame builder both rely on the incremental path.
  uint16_t inc = CRC16_INIT;
  for (int i = 0; i < 9; i++)
    inc = crc16_update(inc, static_cast<uint8_t>(s[i]));
  check(inc == 0x29B1, "incremental folding matches one-shot");
}

// The worked examples from control_interfaces_spec.md section 6.1 and 6.2:
// setting master volume to -20.0 dB and reading it back.  Byte-for-byte.
static void test_spec_vectors() {
  std::printf("Spec worked examples (master volume -20.0 dB)\n");

  const float minus20 = -20.0f;
  uint8_t payload[4];
  std::memcpy(payload, &minus20, 4);
  check(hex(payload, 4) == "00 00 A0 C1", "-20.0f little-endian == 00 00 A0 C1");

  uint8_t frame[MAX_TX_FRAME];

  size_t n = build_request_frame(frame, FRAME_SET_REQ, REQ_SET_MASTER_VOLUME, 0, 0, 4, payload, 4);
  check(hex(frame, n) == "A5 01 D2 00 00 00 00 04 00 00 00 A0 C1 CF 6D", "6.1 SET request bytes");

  n = build_request_frame(frame, FRAME_GET_REQ, REQ_GET_MASTER_VOLUME, 0, 0, 4, nullptr, 0);
  check(hex(frame, n) == "A5 02 D3 00 00 00 00 04 00 B0 EB", "6.2 GET request bytes");
}

// Feed a byte string to a parser and return the results, so tests can assert
// on what a device would have produced.
struct Parsed {
  std::vector<ParseResult> results;
  uint8_t type{0}, status{0};
  uint16_t len{0};
  std::vector<uint8_t> payload;
  int frames{0};
};

static Parsed run_parser(const std::vector<uint8_t> &bytes) {
  FrameParser p;
  Parsed out;
  for (uint8_t b : bytes) {
    ParseResult r = p.feed(b);
    if (r != ParseResult::NEED_MORE)
      out.results.push_back(r);
    if (r == ParseResult::FRAME) {
      out.frames++;
      out.type = p.type();
      out.status = p.status();
      out.len = p.len();
      out.payload.assign(p.payload(), p.payload() + p.len());
    }
  }
  return out;
}

static void test_parse_responses() {
  std::printf("Response parsing\n");

  // 6.1 SET response, OK, no payload.
  Parsed a = run_parser(unhex("A5 81 00 00 00 4C 2F"));
  check(a.frames == 1 && a.type == FRAME_SET_RESP && a.status == CTRL_STATUS_OK && a.len == 0,
        "SET response OK parses");

  // 6.2 GET response carrying the -20.0 dB float back.
  Parsed b = run_parser(unhex("A5 82 00 04 00 00 00 A0 C1 AB 91"));
  bool ok = b.frames == 1 && b.type == FRAME_GET_RESP && b.status == CTRL_STATUS_OK && b.len == 4;
  float got = 0;
  if (ok)
    std::memcpy(&got, b.payload.data(), 4);
  check(ok && got == -20.0f, "GET response payload decodes to -20.0 dB");

  // A corrupted payload byte must be rejected, not delivered.
  Parsed c = run_parser(unhex("A5 82 00 04 00 00 00 A0 C2 AB 91"));
  check(c.frames == 0 && c.results.size() == 1 && c.results[0] == ParseResult::CRC_FAILED,
        "corrupt payload fails CRC and is dropped");
}

static void test_resync() {
  std::printf("Resynchronisation\n");

  // Leading noise before a good frame.
  Parsed a = run_parser(unhex("13 37 00 FF A5 81 00 00 00 4C 2F"));
  check(a.frames == 1, "leading noise is skipped");

  // The case worth being careful about: a truncated frame whose discarded
  // bytes are followed immediately by a real sync byte.  A parser that
  // returned to SYNC on seeing 0xA5 in the type position would swallow it and
  // lose the frame that follows.
  Parsed b = run_parser(unhex("A5 A5 81 00 00 00 4C 2F"));
  check(b.frames == 1, "0xA5 in the type position does not eat the next sync");

  // A frame truncated *mid-header* is a harder case, and one the protocol
  // cannot fully solve: there is no byte stuffing, so a 0xA5 arriving where a
  // length byte is expected is indistinguishable from data and gets consumed
  // as such.  The parser notices the resulting nonsense length and resyncs,
  // but the sync byte it ate belonged to the next frame, which is lost with
  // it.  Recovery therefore lands on the frame after that.  This is why
  // responses have timeouts and notifications carry a sequence number.
  Parsed c = run_parser(unhex("A5 82 00 04 A5 81 00 00 00 4C 2F"));
  check(c.frames == 0 && !c.results.empty() && c.results[0] == ParseResult::OVERSIZE,
        "header truncated onto a sync byte is detected, consuming that frame");

  Parsed d2 = run_parser(unhex("A5 82 00 04 A5 81 00 00 00 4C 2F A5 81 00 00 00 4C 2F"));
  check(d2.frames == 1 && d2.type == FRAME_SET_RESP, "and the parser recovers on the frame after that");

  // Two back-to-back frames must both be delivered.
  Parsed d = run_parser(unhex("A5 81 00 00 00 4C 2F A5 81 00 00 00 4C 2F"));
  check(d.frames == 2, "back-to-back frames both parse");

  // An absurd length field is a desync, not a 64 KB buffer.
  Parsed e = run_parser(unhex("A5 82 00 FF FF 00 00"));
  check(!e.results.empty() && e.results[0] == ParseResult::OVERSIZE, "oversize length is rejected");
}

static void test_notifications() {
  std::printf("Notification frames\n");

  // A BULK_INVALIDATED notification: 8-byte v2 packet inside a 0x40 frame.
  // { version=2, event=0x03, flags=0, seq=0x2A, source=3, reserved }
  std::vector<uint8_t> packet = {0x02, 0x03, 0x00, 0x2A, 0x03, 0x00, 0x00, 0x00};
  std::vector<uint8_t> body = {FRAME_NOTIFY, 0x00, static_cast<uint8_t>(packet.size()), 0x00};
  body.insert(body.end(), packet.begin(), packet.end());
  uint16_t crc = crc16(body.data(), body.size());

  std::vector<uint8_t> frame = {SYNC_BYTE};
  frame.insert(frame.end(), body.begin(), body.end());
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>(crc >> 8));

  Parsed a = run_parser(frame);
  check(a.frames == 1 && a.type == FRAME_NOTIFY && a.len == 8, "notification frame parses");
  check(a.payload.size() == 8 && a.payload[0] == NOTIFY_VERSION_V2 && a.payload[1] == NOTIFY_EVT_BULK_INVALIDATED &&
            a.payload[3] == 0x2A,
        "v2 header decodes (version, event, seq)");

  check(notify_event_affects_params(NOTIFY_EVT_PARAM_CHANGED), "PARAM_CHANGED triggers a refresh");
  check(notify_event_affects_params(NOTIFY_EVT_BULK_INVALIDATED), "BULK_INVALIDATED triggers a refresh");
  check(!notify_event_affects_params(NOTIFY_EVT_IDLE), "IDLE does not");
  // The firmware's event enum is still growing; an unknown ID must be ignored
  // rather than treated as an error or as a reason to re-read.
  check(!notify_event_affects_params(0x7E), "an unknown event ID is ignored");
}

static void test_status_classification() {
  std::printf("Status classification\n");
  check(ctrl_status_is_retryable(CTRL_STATUS_BUSY), "BUSY is retryable");
  check(ctrl_status_is_retryable(CTRL_STATUS_BULK_LOCKED), "BULK_LOCKED is retryable");
  check(ctrl_status_is_retryable(CTRL_STATUS_CRC_ERROR), "CRC_ERROR is retryable");
  // Retrying these is an infinite loop: the device will refuse identically
  // however many times we ask.
  check(!ctrl_status_is_retryable(CTRL_STATUS_ERROR), "ERROR is permanent");
  check(!ctrl_status_is_retryable(CTRL_STATUS_BLOCKED), "BLOCKED is permanent");
  check(!ctrl_status_is_retryable(CTRL_STATUS_OVERSIZE), "OVERSIZE is permanent");
  check(!ctrl_status_is_retryable(CTRL_STATUS_FRAME_ERROR), "FRAME_ERROR is permanent");
}

// ---------------------------------------------------------------------------
// Spectrum analyser (RTA)
// ---------------------------------------------------------------------------

// Builds a well-formed GET response frame around an arbitrary payload, so the
// parser is exercised through the same path a real device response takes.
static std::vector<uint8_t> make_get_response(const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> f;
  f.push_back(SYNC_BYTE);
  f.push_back(FRAME_GET_RESP);
  f.push_back(CTRL_STATUS_OK);
  f.push_back(static_cast<uint8_t>(payload.size() & 0xFF));
  f.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0xFF));
  f.insert(f.end(), payload.begin(), payload.end());
  const uint16_t crc = crc16(f.data() + 1, f.size() - 1);
  f.push_back(static_cast<uint8_t>(crc & 0xFF));
  f.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return f;
}

static void test_rta_wire_sizes() {
  std::printf("RTA wire sizes\n");
  check(RTA_CFG_VERSION == 3, "protocol version is 3");
  check(RTA_CONFIG_LEN == 12, "RtaConfig is 12 bytes");
  check(RTA_CAPS_LEN == 16, "RtaCaps is 16 bytes");
  check(RTA_BAND_FRAME_LEN == 82, "RtaBandFrame is 82 bytes");
  check(RTA_STATUS_LEN == 24, "RtaStatus is 24 bytes");
  check(RTA_MAX_BANDS == 37, "37 bands maximum");
  check(RTA_LEVEL_ZERO_DBFS == 243, "level 243 is 0 dBFS");
  // V3 moved both level arrays three bytes later than V2. Pinning the offsets
  // is what makes a V2-derived parse fail here rather than on screen.
  check(RTA_BF_OFF_AVG == 8, "avg[] starts at offset 8");
  check(RTA_BF_OFF_PEAK == 45, "peak[] starts at offset 45");
}

static void test_rta_config_serialise() {
  std::printf("RTA config serialisation\n");
  RtaConfig cfg;
  cfg.version = RTA_CFG_VERSION;
  cfg.tap = RTA_TAP_OUTPUT;
  cfg.channel_mask = 0x0102;
  cfg.fft_order = 10;
  cfg.avg_ms = 250;
  cfg.peak_decay_db_s = 20;
  cfg.flags = 0;

  uint8_t buf[RTA_CONFIG_LEN];
  rta_write_config(buf, cfg);
  // Pins every little-endian uint16 into its slot, and the reserved bytes at
  // offsets 5 and 10-11 to zero.
  check(hex(buf, sizeof(buf)) == "03 01 02 01 0A 00 FA 00 14 00 00 00", "config serialises to the expected bytes");

  RtaConfig back;
  check(rta_parse_config(buf, sizeof(buf), &back), "parses back");
  check(back.version == 3 && back.tap == RTA_TAP_OUTPUT && back.channel_mask == 0x0102 && back.fft_order == 10 &&
            back.avg_ms == 250 && back.peak_decay_db_s == 20 && back.flags == 0,
        "round-trips field for field");
  check(!rta_parse_config(buf, RTA_CONFIG_LEN - 1, &back), "a short config is rejected");
}

static void test_rta_band_frame_parse() {
  std::printf("RTA band frame parsing\n");
  std::vector<uint8_t> raw(RTA_BAND_FRAME_LEN, 0);
  raw[0] = RTA_CFG_VERSION;
  raw[1] = 5;     // channel
  raw[2] = 0x7B;  // seq
  raw[3] = 34;    // n_bands
  raw[4] = 0x2C;  // age_ms low
  raw[5] = 0x01;  // age_ms high -> 300
  // Marker bytes either side of the avg/peak boundary. A parse that still
  // used the V2 layout would read these three bytes earlier and swap them.
  raw[RTA_BF_OFF_AVG] = 0x11;
  raw[RTA_BF_OFF_AVG + RTA_MAX_BANDS - 1] = 0xAA;
  raw[RTA_BF_OFF_PEAK] = 0xBB;
  raw[RTA_BF_OFF_PEAK + RTA_MAX_BANDS - 1] = 0x22;

  RtaBandFrame f;
  check(rta_parse_band_frame(raw.data(), raw.size(), &f), "a V3 frame parses");
  check(f.channel == 5 && f.seq == 0x7B && f.n_bands == 34, "header fields land correctly");
  check(f.age_ms == 300, "age_ms is little-endian");
  check(f.avg[0] == 0x11 && f.avg[RTA_MAX_BANDS - 1] == 0xAA, "avg[] spans offsets 8..44");
  check(f.peak[0] == 0xBB && f.peak[RTA_MAX_BANDS - 1] == 0x22, "peak[] spans offsets 45..81");
}

static void test_rta_band_frame_reject() {
  std::printf("RTA band frame rejection\n");
  std::vector<uint8_t> raw(RTA_BAND_FRAME_LEN, 0);
  raw[0] = RTA_CFG_VERSION;
  RtaBandFrame f;

  raw[0] = 2;
  check(!rta_parse_band_frame(raw.data(), raw.size(), &f), "a V2 version byte is refused");
  raw[0] = 4;
  check(!rta_parse_band_frame(raw.data(), raw.size(), &f), "an unknown future version is refused");

  raw[0] = RTA_CFG_VERSION;
  // A real V2 frame is 80 bytes: two short, with every index shifted. Length
  // alone rejects it even before the version byte is consulted.
  check(!rta_parse_band_frame(raw.data(), 80, &f), "an 80-byte V2-sized frame is refused");
  check(!rta_parse_band_frame(raw.data(), 0, &f), "an empty payload is refused");

  // A device claiming more bands than the array holds must not be allowed to
  // drive a caller past the end of avg[]/peak[].
  raw[3] = 255;
  check(rta_parse_band_frame(raw.data(), raw.size(), &f), "an over-large n_bands still parses");
  check(f.n_bands == RTA_MAX_BANDS, "n_bands is clamped to the array size");
}

static void test_rta_level_decode() {
  std::printf("RTA level decoding\n");
  check(rta_level_to_dbfs(243, 243) == 0.0f, "243 is 0 dBFS");
  check(rta_level_to_dbfs(255, 243) == 6.0f, "255 is +6 dBFS");
  check(rta_level_to_dbfs(223, 243) == -10.0f, "half a dB per step");
  check(rta_level_is_floor(0), "0 is the floor");
  check(!rta_level_is_floor(1), "1 is not");

  // The floor means "nothing here", not -121.5 dB of signal, so it must render
  // as an empty bar however low the scale bottom is set.
  check(rta_level_to_fraction(0, 243, -60.0f) == 0.0f, "a floor byte is exactly 0.0");
  check(rta_level_to_fraction(0, 243, -1000.0f) == 0.0f, "even against an absurd floor");
  check(rta_level_to_fraction(243, 243, -60.0f) == 1.0f, "0 dBFS is full scale");
  check(rta_level_to_fraction(255, 243, -60.0f) == 1.0f, "above 0 dBFS clamps to full scale");
  check(rta_level_to_fraction(123, 243, -60.0f) == 0.0f, "at the floor dB, clamps to 0.0");
  const float half = rta_level_to_fraction(183, 243, -60.0f);
  check(half > 0.49f && half < 0.51f, "-30 dBFS is half way up a -60 dB scale");
}

static void test_rta_frame_over_transport() {
  std::printf("RTA band frame over the wire\n");
  // The regression guard for MAX_RX_PAYLOAD: at its previous value of 80 the
  // parser rejected every 82-byte band frame as OVERSIZE and resynced, so the
  // spectrum could never have worked at all.
  check(MAX_RX_PAYLOAD >= RTA_BAND_FRAME_LEN, "the RX buffer can hold a band frame");

  std::vector<uint8_t> payload(RTA_BAND_FRAME_LEN);
  for (size_t i = 0; i < payload.size(); i++)
    payload[i] = static_cast<uint8_t>(i + 1);
  payload[0] = RTA_CFG_VERSION;

  const auto frame = make_get_response(payload);
  FrameParser p;
  ParseResult r = ParseResult::NEED_MORE;
  for (uint8_t b : frame)
    r = p.feed(b);

  check(r == ParseResult::FRAME, "an 82-byte payload parses as a frame, not OVERSIZE");
  check(p.type() == FRAME_GET_RESP && p.status() == CTRL_STATUS_OK, "type and status survive");
  check(p.len() == RTA_BAND_FRAME_LEN, "the full payload length is reported");
  check(std::memcmp(p.payload(), payload.data(), payload.size()) == 0, "the payload round-trips byte for byte");
}

static void test_rta_request_bytes() {
  std::printf("RTA request framing\n");
  uint8_t buf[MAX_TX_FRAME];

  // wLength is a hard truncation on the device, not a hint: asking for fewer
  // bytes than the frame holds would return a short payload with a valid CRC.
  size_t n = build_request_frame(buf, FRAME_GET_REQ, REQ_RTA_GET_BANDS, /*wvalue=*/1, /*windex=*/0,
                                 /*wlen=*/RTA_BAND_FRAME_LEN, nullptr, 0);
  check(hex(buf, n) == "A5 02 0B 01 00 00 00 52 00 CA D8",
        "GET_BANDS carries the channel in wValue and 82 in wLength");

  // REQ_RTA_SET_CONFIG is the one command whose payload outgrew the old
  // 8-byte TX buffer.
  check(MAX_TX_PAYLOAD >= RTA_CONFIG_LEN, "the TX buffer can hold an RtaConfig");
  RtaConfig cfg;
  cfg.tap = RTA_TAP_INPUT;
  cfg.channel_mask = 0x0003;
  cfg.fft_order = 10;
  cfg.avg_ms = 100;
  cfg.peak_decay_db_s = 30;
  uint8_t payload[RTA_CONFIG_LEN];
  rta_write_config(payload, cfg);
  n = build_request_frame(buf, FRAME_SET_REQ, REQ_RTA_SET_CONFIG, 0, 0, RTA_CONFIG_LEN, payload, RTA_CONFIG_LEN);
  check(n == 1 + 1 + REQ_HEADER_LEN + RTA_CONFIG_LEN + 2, "SET_CONFIG builds a full 12-byte payload frame");
  check(buf[2] == REQ_RTA_SET_CONFIG && buf[7] == RTA_CONFIG_LEN, "opcode and wLength are in place");

  // The control opcode mutates state but is dispatched on the device's GET
  // path, with the action in wValue.
  n = build_request_frame(buf, FRAME_GET_REQ, REQ_RTA_CONTROL, RTA_CTL_STOP, 0, 1, nullptr, 0);
  check(buf[1] == FRAME_GET_REQ && buf[2] == REQ_RTA_CONTROL, "CONTROL is sent as a GET frame");
  check(n == 1 + 1 + REQ_HEADER_LEN + 2, "with no payload");
}

// ---------------------------------------------------------------------------
// Presets
// ---------------------------------------------------------------------------

static void test_preset_directory_parse() {
  std::printf("Preset directory\n");
  check(PRESET_SLOTS == 10, "10 preset slots");
  check(PRESET_NAME_LEN == 32, "names are 32 bytes");
  check(PRESET_DIR_LEN == 7, "the directory is 7 bytes");

  // Occupancy is a little-endian u16, so a slot above 7 lives in the second
  // byte -- reading it as one byte would silently lose slots 8 and 9.
  const auto raw = unhex("05 03 01 04 02 00 01");
  PresetDirectory dir;
  check(parse_preset_directory(raw.data(), raw.size(), &dir), "a 7-byte payload parses");
  check(dir.slot_occupied == 0x0305, "occupancy is little-endian");
  check(dir.is_occupied(0) && dir.is_occupied(2) && dir.is_occupied(8) && dir.is_occupied(9),
        "slots 0, 2, 8 and 9 are occupied");
  check(!dir.is_occupied(1) && !dir.is_occupied(7), "slots 1 and 7 are not");
  check(!dir.is_occupied(PRESET_SLOTS), "an out-of-range slot is never occupied");
  check(dir.startup_mode == 1 && dir.default_slot == 4, "startup mode and default slot");
  check(dir.last_active_slot == 2, "last active slot");
  check(dir.output_config_mode == 0 && dir.master_volume_mode == 1, "both persistence modes");

  check(!parse_preset_directory(raw.data(), 6, &dir), "a short payload is rejected");
}

static void test_preset_name_parse() {
  std::printf("Preset name\n");
  std::vector<uint8_t> name(PRESET_NAME_LEN, 0);
  std::memcpy(name.data(), "Movie", 5);
  check(preset_name_length(name.data(), name.size()) == 5, "a NUL-padded name stops at the terminator");

  // Every slot but 0 is an empty string on a fresh device, which is a real
  // state to report rather than a parse failure.
  std::vector<uint8_t> empty(PRESET_NAME_LEN, 0);
  check(preset_name_length(empty.data(), empty.size()) == 0, "an unnamed slot reads as empty");

  // 32 visible characters leave no room for a terminator, so the field width
  // has to be the limit rather than the NUL.
  std::vector<uint8_t> full(PRESET_NAME_LEN, 'x');
  check(preset_name_length(full.data(), full.size()) == PRESET_NAME_LEN, "a full 32-byte name is not overrun");

  // A truncated response must not be read past its actual length.
  check(preset_name_length(full.data(), 4) == 4, "a short response is bounded by its own length");
}

static void test_preset_status_is_not_ctrl_status() {
  std::printf("Preset status codes\n");
  // PRESET_* codes live in byte 0 of a preset command's response payload, and
  // they are a different namespace from the transport's CtrlStatus. The two
  // agree only on 0x00 meaning OK, and every other value collides with an
  // unrelated meaning.
  //
  // The casts are deliberate: -Wenum-compare refuses to compare the two enums
  // directly, which is the compiler making the same point.
  check(static_cast<uint8_t>(PRESET_OK) == 0x00, "PRESET_OK is 0x00");
  check(static_cast<uint8_t>(PRESET_ERR_SLOT_EMPTY) == static_cast<uint8_t>(CTRL_STATUS_ERROR),
        "SLOT_EMPTY collides with CTRL_STATUS_ERROR by value");

  // This is the trap, asserted rather than described. Two of the four preset
  // errors land on transport codes that are *retryable*, so feeding a preset
  // status to the transport classifier does not merely mislabel it -- it turns
  // a permanent rejection into an infinite retry loop. Both of these read
  // "true", and both are wrong. Hence the separate enum, and hence
  // on_preset_load_() reading the byte as a PresetStatus and never routing it
  // through ctrl_status_is_retryable().
  check(static_cast<uint8_t>(PRESET_ERR_INVALID_SLOT) == static_cast<uint8_t>(CTRL_STATUS_BUSY),
        "INVALID_SLOT collides with the retryable BUSY");
  check(static_cast<uint8_t>(PRESET_ERR_FLASH_WRITE) == static_cast<uint8_t>(CTRL_STATUS_BULK_LOCKED),
        "FLASH_WRITE collides with the retryable BULK_LOCKED");
  check(ctrl_status_is_retryable(PRESET_ERR_INVALID_SLOT),
        "so the transport classifier would wrongly retry INVALID_SLOT");
  check(ctrl_status_is_retryable(PRESET_ERR_FLASH_WRITE),
        "and wrongly retry FLASH_WRITE -- never classify a preset status this way");

  check(std::string(preset_status_to_string(PRESET_ERR_FLASH_WRITE)) == "FLASH_WRITE", "status names its code");
  check(std::string(preset_status_to_string(PRESET_ERR_INVALID_SLOT)) == "INVALID_SLOT", "and the others");
}

static void test_preset_request_bytes() {
  std::printf("Preset request framing\n");
  uint8_t buf[MAX_TX_FRAME];

  // The load rewrites every DSP parameter, yet the firmware dispatches it on
  // its GET path with the slot in wValue. Sent as a SET frame it stalls, so
  // the frame type is pinned here rather than inferred from "this is a write".
  size_t n = build_request_frame(buf, FRAME_GET_REQ, REQ_PRESET_LOAD, /*wvalue=*/3, /*windex=*/0,
                                 /*wlen=*/1, nullptr, 0);
  check(buf[1] == FRAME_GET_REQ, "PRESET_LOAD is sent as a GET frame");
  check(buf[2] == REQ_PRESET_LOAD && buf[3] == 3 && buf[4] == 0, "the slot rides in wValue");
  check(buf[7] == 1 && buf[8] == 0, "one status byte is requested");
  check(n == 1 + 1 + REQ_HEADER_LEN + 2, "with no payload");

  // The name read is the one preset command with a payload worth sizing: 32
  // bytes against a 132-byte transport cap.
  n = build_request_frame(buf, FRAME_GET_REQ, REQ_PRESET_GET_NAME, /*wvalue=*/7, 0, PRESET_NAME_LEN, nullptr, 0);
  check(buf[2] == REQ_PRESET_GET_NAME && buf[3] == 7, "GET_NAME carries the slot in wValue");
  check(buf[7] == PRESET_NAME_LEN, "and asks for the full 32 bytes");
  check(PRESET_NAME_LEN <= MAX_RX_PAYLOAD, "a name fits the transport");
  (void) n;
}

// All five ToggleTarget values, so a new one added to the enum but forgotten
// here shows up as a -Wswitch warning in the helpers rather than as a silently
// untested row.
static const ToggleTarget ALL_TOGGLES[] = {
    ToggleTarget::USER_MUTE, ToggleTarget::LOUDNESS, ToggleTarget::EQ_BYPASS,
    ToggleTarget::CROSSFEED, ToggleTarget::LEVELLER,
};

static void test_toggle_opcodes() {
  std::printf("DSP toggle opcode table\n");

  // Pinned against the firmware's config.h so a transcription slip is a test
  // failure rather than a device that silently ignores a write.
  check(toggle_set_opcode(ToggleTarget::USER_MUTE) == 0xDC, "user_mute SET is 0xDC");
  check(toggle_get_opcode(ToggleTarget::USER_MUTE) == 0xDD, "user_mute GET is 0xDD");
  check(toggle_set_opcode(ToggleTarget::LOUDNESS) == 0x58, "loudness SET is 0x58");
  check(toggle_get_opcode(ToggleTarget::LOUDNESS) == 0x59, "loudness GET is 0x59");
  check(toggle_set_opcode(ToggleTarget::EQ_BYPASS) == 0x46, "eq_bypass SET is 0x46");
  check(toggle_get_opcode(ToggleTarget::EQ_BYPASS) == 0x47, "eq_bypass GET is 0x47");
  check(toggle_set_opcode(ToggleTarget::CROSSFEED) == 0x5E, "crossfeed SET is 0x5E");
  check(toggle_get_opcode(ToggleTarget::CROSSFEED) == 0x5F, "crossfeed GET is 0x5F");
  check(toggle_set_opcode(ToggleTarget::LEVELLER) == 0xB4, "leveller SET is 0xB4");
  check(toggle_get_opcode(ToggleTarget::LEVELLER) == 0xB5, "leveller GET is 0xB5");

  const size_t count = sizeof(ALL_TOGGLES) / sizeof(ALL_TOGGLES[0]);
  check(count == TOGGLE_COUNT, "TOGGLE_COUNT matches the enum");

  // Every opcode in the family must be distinct, in both directions at once:
  // two targets sharing a SET opcode would coalesce in the hub's queue and lose
  // a write, and sharing a GET would route one target's readback into another.
  bool all_distinct = true;
  bool all_named = true;
  for (size_t i = 0; i < count; i++) {
    if (std::string(toggle_name(ALL_TOGGLES[i])) == "unknown")
      all_named = false;
    for (size_t j = i + 1; j < count; j++) {
      if (toggle_set_opcode(ALL_TOGGLES[i]) == toggle_set_opcode(ALL_TOGGLES[j]))
        all_distinct = false;
      if (toggle_get_opcode(ALL_TOGGLES[i]) == toggle_get_opcode(ALL_TOGGLES[j]))
        all_distinct = false;
    }
    // A target whose SET and GET were the same opcode would read back what it
    // had just written from the wrong path entirely.
    if (toggle_set_opcode(ALL_TOGGLES[i]) == toggle_get_opcode(ALL_TOGGLES[i]))
      all_distinct = false;
  }
  check(all_distinct, "every SET and GET opcode in the family is distinct");
  check(all_named, "every target has a name for dump_config");

  // Bits and their popcount, which the refresh burst sizes itself from.
  check(toggle_bit(ToggleTarget::USER_MUTE) == 0x0001, "USER_MUTE is bit 0");
  check(toggle_bit(ToggleTarget::LEVELLER) == 0x0010, "LEVELLER is bit 4");
  check(toggle_mask_count(0) == 0, "an empty mask counts zero");
  check(toggle_mask_count(0x001F) == 5, "a full mask counts every target");
  check(toggle_mask_count(0x0005) == 2, "and a sparse one counts its bits");
}

static void test_toggle_request_bytes() {
  std::printf("DSP toggle framing\n");
  uint8_t buf[MAX_TX_FRAME];
  uint8_t payload[1] = {1};

  // Unlike REQ_PRESET_LOAD, these are ordinary SETs: their handlers sit above
  // vendor_handle_get() in the firmware. Pinned byte for byte, because sending
  // one on the wrong path is the single most likely way to get this wrong.
  size_t n = build_request_frame(buf, FRAME_SET_REQ, toggle_set_opcode(ToggleTarget::LOUDNESS), /*wvalue=*/0,
                                 /*windex=*/0, /*wlen=*/1, payload, 1);
  check(hex(buf, n) == "A5 01 58 00 00 00 00 01 00 01 FE 01", "loudness SET request bytes");
  check(buf[1] == FRAME_SET_REQ, "a toggle write is a SET frame, not write-as-read");
  check(buf[3] == 0 && buf[4] == 0, "with nothing in wValue -- the value is a payload byte");

  n = build_request_frame(buf, FRAME_GET_REQ, toggle_get_opcode(ToggleTarget::LOUDNESS), 0, 0, 1, nullptr, 0);
  check(hex(buf, n) == "A5 02 59 00 00 00 00 01 00 F2 4C", "loudness GET request bytes");
  check(n == 1 + 1 + REQ_HEADER_LEN + 2, "and no payload");

  // Every target answers one byte, so the read length is uniform.
  for (size_t i = 0; i < sizeof(ALL_TOGGLES) / sizeof(ALL_TOGGLES[0]); i++) {
    n = build_request_frame(buf, FRAME_GET_REQ, toggle_get_opcode(ALL_TOGGLES[i]), 0, 0, 1, nullptr, 0);
    if (buf[7] != 1 || buf[8] != 0) {
      check(false, std::string("one byte requested for ") + toggle_name(ALL_TOGGLES[i]));
      break;
    }
  }
  check(true, "one byte requested for every target");
}

int main() {
  test_crc_reference();
  test_spec_vectors();
  test_parse_responses();
  test_resync();
  test_notifications();
  test_status_classification();
  test_rta_wire_sizes();
  test_rta_config_serialise();
  test_rta_band_frame_parse();
  test_rta_band_frame_reject();
  test_rta_level_decode();
  test_rta_frame_over_transport();
  test_rta_request_bytes();
  test_preset_directory_parse();
  test_preset_name_parse();
  test_preset_status_is_not_ctrl_status();
  test_preset_request_bytes();
  test_toggle_opcodes();
  test_toggle_request_bytes();

  if (failures) {
    std::printf("\n%d test(s) FAILED\n", failures);
    return 1;
  }
  std::printf("\nAll tests passed.\n");
  return 0;
}
