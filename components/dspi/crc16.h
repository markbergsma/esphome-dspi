#pragma once

#include <cstddef>
#include <cstdint>

// CRC16-CCITT-FALSE, as required by the DSPi UART control transport.
//
//   polynomial 0x1021, initial value 0xFFFF, no input reflection,
//   no output reflection, no final XOR.
//
// On the wire the checksum is transmitted little-endian (low byte first) and
// covers every byte after the sync byte, i.e. the type byte through the last
// payload byte inclusive.  The sync byte 0xA5 is not included.
//
// Deliberately bitwise rather than table-driven: frames are at most ~17 bytes,
// so a table would cost 512 bytes of flash to save a handful of microseconds.
//
// This header has no ESPHome dependencies so the unit tests can compile it
// with a host compiler.

namespace esphome {
namespace dspi {

static constexpr uint16_t CRC16_INIT = 0xFFFF;

// Fold one byte into an accumulating CRC.  Used incrementally in both
// directions: the RX parser folds each byte as it arrives, the TX builder
// folds each byte as it writes it.  Neither ever makes a second pass.
inline uint16_t crc16_update(uint16_t crc, uint8_t byte) {
  crc ^= static_cast<uint16_t>(byte) << 8;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

// Convenience one-shot over a buffer.  The incremental form above is what the
// hot paths use; this exists for tests and for building short frames.
inline uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = CRC16_INIT;
  for (size_t i = 0; i < len; i++) {
    crc = crc16_update(crc, data[i]);
  }
  return crc;
}

}  // namespace dspi
}  // namespace esphome
