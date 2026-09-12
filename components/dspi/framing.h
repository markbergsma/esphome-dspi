#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "crc16.h"
#include "protocol.h"

// Frame construction and a byte-at-a-time frame parser for the DSPi UART
// control transport.
//
// Kept free of ESPHome dependencies so the unit tests exercise this exact
// code with a host compiler.  Framing and resynchronisation are the two parts
// most expensive to debug on hardware, so they are the two parts worth
// testing off it.

namespace esphome {
namespace dspi {

// Serialise a request into `out`, which must have room for MAX_TX_FRAME bytes.
// Returns the number of bytes written.
//
// `type` is FRAME_SET_REQ or FRAME_GET_REQ and comes from the opcode, not from
// whether the caller considers this a read or a write (see protocol.h).
// `wlen` is the payload length on a SET, or a cap on the response size on a
// GET (0 meaning uncapped).
inline size_t build_request_frame(uint8_t *out, uint8_t type, uint8_t req, uint16_t wvalue, uint16_t windex,
                                  uint16_t wlen, const uint8_t *payload, uint8_t payload_len) {
  size_t n = 0;
  uint16_t crc = CRC16_INIT;

  out[n++] = SYNC_BYTE;  // not covered by the CRC

  const uint8_t header[] = {
      type,
      req,
      static_cast<uint8_t>(wvalue & 0xFF),
      static_cast<uint8_t>(wvalue >> 8),
      static_cast<uint8_t>(windex & 0xFF),
      static_cast<uint8_t>(windex >> 8),
      static_cast<uint8_t>(wlen & 0xFF),
      static_cast<uint8_t>(wlen >> 8),
  };
  for (uint8_t b : header) {
    out[n++] = b;
    crc = crc16_update(crc, b);
  }

  for (uint8_t i = 0; i < payload_len; i++) {
    out[n++] = payload[i];
    crc = crc16_update(crc, payload[i]);
  }

  out[n++] = static_cast<uint8_t>(crc & 0xFF);  // little-endian on the wire
  out[n++] = static_cast<uint8_t>(crc >> 8);
  return n;
}

// Result of feeding one byte to the parser.
enum class ParseResult : uint8_t {
  NEED_MORE,   // nothing to report yet
  FRAME,       // a complete, CRC-valid frame is available
  CRC_FAILED,  // a frame completed but its CRC did not match; it was discarded
  OVERSIZE,    // a length field exceeded MAX_RX_PAYLOAD; resynchronising
};

// Byte-at-a-time parser for device-to-host frames.
//
// Tolerates arbitrary noise: the spec permits unrecognised bytes on the link
// and requires resynchronisation on the next sync byte.  The parser never
// allocates, never blocks, and holds exactly one frame at a time.
class FrameParser {
 public:
  ParseResult feed(uint8_t byte) {
    switch (state_) {
      case State::SYNC:
        if (byte == SYNC_BYTE) {
          state_ = State::TYPE;
        }
        return ParseResult::NEED_MORE;

      case State::TYPE:
        // A sync byte here means the previous one was noise, or a frame was
        // truncated mid-flight.  Stay in TYPE rather than falling back to
        // SYNC: treating this 0xA5 as the discarded byte would consume the
        // real sync byte and desynchronise us for a whole further frame.
        if (byte == SYNC_BYTE) {
          return ParseResult::NEED_MORE;
        }
        if (byte != FRAME_SET_RESP && byte != FRAME_GET_RESP && byte != FRAME_NOTIFY) {
          state_ = State::SYNC;
          return ParseResult::NEED_MORE;
        }
        type_ = byte;
        crc_ = crc16_update(CRC16_INIT, byte);
        header_pos_ = 0;
        state_ = State::HEADER;
        return ParseResult::NEED_MORE;

      case State::HEADER: {
        crc_ = crc16_update(crc_, byte);
        // status, lenL, lenH.  On a notification byte 0 is a fixed 0x00
        // placeholder rather than a status; parsing it uniformly and ignoring
        // the value is what lets one code path serve all three frame types.
        if (header_pos_ == 0) {
          status_ = byte;
        } else if (header_pos_ == 1) {
          len_ = byte;
        } else {
          len_ |= static_cast<uint16_t>(byte) << 8;
        }
        if (++header_pos_ < RESP_HEADER_LEN) {
          return ParseResult::NEED_MORE;
        }
        if (len_ > MAX_RX_PAYLOAD) {
          state_ = State::SYNC;
          return ParseResult::OVERSIZE;
        }
        payload_pos_ = 0;
        state_ = len_ ? State::PAYLOAD : State::CRC_LO;
        return ParseResult::NEED_MORE;
      }

      case State::PAYLOAD:
        crc_ = crc16_update(crc_, byte);
        payload_[payload_pos_++] = byte;
        if (payload_pos_ >= len_) {
          state_ = State::CRC_LO;
        }
        return ParseResult::NEED_MORE;

      case State::CRC_LO:
        rx_crc_ = byte;
        state_ = State::CRC_HI;
        return ParseResult::NEED_MORE;

      case State::CRC_HI: {
        rx_crc_ |= static_cast<uint16_t>(byte) << 8;
        state_ = State::SYNC;
        return (rx_crc_ == crc_) ? ParseResult::FRAME : ParseResult::CRC_FAILED;
      }
    }
    return ParseResult::NEED_MORE;
  }

  // Valid only immediately after feed() returns ParseResult::FRAME.
  uint8_t type() const { return type_; }
  uint8_t status() const { return status_; }
  uint16_t len() const { return len_; }
  const uint8_t *payload() const { return payload_; }

  // Drop any partial frame and wait for the next sync byte.  Used when the
  // transaction a frame would have belonged to has already been abandoned.
  void reset() { state_ = State::SYNC; }

 protected:
  enum class State : uint8_t { SYNC, TYPE, HEADER, PAYLOAD, CRC_LO, CRC_HI };

  State state_{State::SYNC};
  uint8_t type_{0};
  uint8_t status_{0};
  uint16_t len_{0};
  uint16_t crc_{CRC16_INIT};
  uint16_t rx_crc_{0};
  uint8_t header_pos_{0};
  uint16_t payload_pos_{0};
  uint8_t payload_[MAX_RX_PAYLOAD];
};

}  // namespace dspi
}  // namespace esphome
