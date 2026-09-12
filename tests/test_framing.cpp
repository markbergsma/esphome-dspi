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

int main() {
  test_crc_reference();
  test_spec_vectors();
  test_parse_responses();
  test_resync();
  test_notifications();
  test_status_classification();

  if (failures) {
    std::printf("\n%d test(s) FAILED\n", failures);
    return 1;
  }
  std::printf("\nAll tests passed.\n");
  return 0;
}
