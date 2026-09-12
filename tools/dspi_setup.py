#!/usr/bin/env python3
"""One-time DSPi setup for use with the esphome-dspi component.

The DSPi ships with its UART control interface disabled, and it deliberately
refuses to configure that interface over the interface itself -- so an attached
controller can never reconfigure, move the pins of, or disable the very link it
is talking on, and can never lock itself out. Enabling it is therefore a
one-time job for a USB host: this script.

Two kinds of setting are involved, and they persist differently:

  - The UART interface config is stored by the firmware as soon as it is
    applied, in the DSPi's preset directory. It survives a reboot and even a
    factory reset. Nothing here chooses that; it is how the device behaves.
  - The audio settings (clock mode, clock-pin mode, slave BCK pin) live in the
    IO config, which is live-only until explicitly saved. Pass --persist to
    save it. Without that, the DSPi reverts on its next reboot and ends up
    driving BCK/LRCLK while the ESP32 is also driving them.

Usage:
    pip install pyusb
    python3 dspi_setup.py                           # show state, change nothing
    python3 dspi_setup.py --apply                   # enable UART control + notifications
    python3 dspi_setup.py --apply --audio           # ...and the I2S clock-slave input, live only
    python3 dspi_setup.py --apply --audio --persist # ...and write it to flash

Run with no arguments first: it prints what the device currently reports and
tells you what --apply would change.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import usb.core
    import usb.util
except ImportError:
    sys.exit("pyusb is required:  pip install pyusb")

VID, PID = 0x2E8B, 0xFEAA
VENDOR_INTERFACE = 2

# bmRequestType. The DSPi splits its handlers into a GET (IN) path and a SET
# (OUT) path, and a number of mutating commands live on the GET path with all
# their parameters packed into wValue -- "write-as-read". Which path an opcode
# uses is a property of that opcode, not of whether it reads or writes, so each
# command below states its own direction rather than inferring one.
OUT = 0x41  # host-to-device, vendor, interface
IN = 0xC1  # device-to-host, vendor, interface

REQ_GET_PLATFORM = 0x7F
REQ_SET_I2S_CLOCK_MODE = 0x88  # OUT, 1-byte payload
REQ_GET_I2S_CLOCK_MODE = 0x89
REQ_SET_I2S_BCK_PIN = 0xC2  # IN  (write-as-read), wValue = (role << 8) | gpio
REQ_GET_I2S_BCK_PIN = 0xC3  # wValue = role
REQ_SET_INPUT_SOURCE = 0xE0  # OUT, 1-byte payload
REQ_GET_INPUT_SOURCE = 0xE1
REQ_SAVE_OUTPUT_CONFIG = 0x52  # IN  (write-as-read), persists the live IO config
REQ_SET_UART_CONFIG = 0xF5  # OUT, 8-byte UartCtrlConfig
REQ_GET_UART_CONFIG = 0xF6
REQ_GET_CTRL_IFACE_STATUS = 0xF9
REQ_SET_I2S_CLOCK_PIN_MODE = 0xFE  # IN  (write-as-read), wValue = 0/1
REQ_GET_I2S_CLOCK_PIN_MODE = 0xFF

INPUT_SOURCE_I2S = 2
I2S_CLOCK_MODE_SLAVE = 1
I2S_CLOCK_PIN_MODE_SPLIT = 1

PIN_CONFIG_NAMES = {
    0x00: "SUCCESS",
    0x01: "INVALID_PIN",
    0x02: "PIN_IN_USE",
    0x03: "OUTPUT_ACTIVE",
    0x04: "INVALID_OUTPUT",
    0x05: "INVALID_PARAM",
}

# Defaults matching examples/control-only.yaml, and the DSPin player.
DEFAULT_UART_TX = 16
DEFAULT_UART_RX = 17
DEFAULT_BAUD = 460800
# Slave clock pair. The default of 26/27 collides with some builds' output
# wiring; 2/3 keeps the whole I2S input bundle on adjacent pins (data on GPIO
# 1) and is free whenever the I2S input is stereo. See the README.
DEFAULT_SLAVE_BCK = 2


class DSPi:
    def __init__(self) -> None:
        self.dev = usb.core.find(idVendor=VID, idProduct=PID)
        if self.dev is None:
            sys.exit(f"No DSPi found (looked for {VID:#06x}:{PID:#06x}). Is it plugged in?")

    def get(self, req: int, length: int, wvalue: int = 0) -> bytes:
        return bytes(self.dev.ctrl_transfer(IN, req, wvalue, VENDOR_INTERFACE, length))

    def set(self, req: int, payload: bytes, wvalue: int = 0) -> None:
        self.dev.ctrl_transfer(OUT, req, wvalue, VENDOR_INTERFACE, payload)

    def set_as_read(self, req: int, wvalue: int) -> int:
        """Issue a write-as-read command and return its 1-byte status."""
        return self.get(req, 1, wvalue)[0]


def status_name(code: int) -> str:
    return PIN_CONFIG_NAMES.get(code, f"UNKNOWN({code:#04x})")


def show(d: DSPi) -> dict:
    """Print what the device currently reports, and return the parsed state."""
    plat = d.get(REQ_GET_PLATFORM, 7)
    major = plat[1]
    minor, patch = (plat[4], plat[5]) if len(plat) >= 6 else (plat[2] >> 4, plat[2] & 0xF)
    print(f"DSPi firmware {major}.{minor}.{patch} (platform {plat[0]})")

    enabled, tx, rx, notify, baud = struct.unpack("<BBBBI", d.get(REQ_GET_UART_CONFIG, 8))
    st = d.get(REQ_GET_CTRL_IFACE_STATUS, 8)
    print("\nUART control interface:")
    print(f"  enabled:       {'yes' if enabled else 'NO'}")
    print(f"  live:          {'yes' if st[1] else 'NO'}")
    print(f"  pins:          TX=GPIO{tx}  RX=GPIO{rx}")
    print(f"  baud:          {baud}")
    print(f"  notifications: {'on' if notify else 'OFF'}")
    print(f"  last status:   {status_name(st[0])}")
    print(f"  protocol ver:  {st[4]}")

    if enabled and not st[1]:
        # The stored config says on, but the peripheral did not come up, which
        # at boot means its pins were already claimed by the output wiring.
        print("\n  ! Stored as enabled but not live: its pins collided at boot.")
        print("    Free the conflicting pin or choose a different valid pair.")

    src = d.get(REQ_GET_INPUT_SOURCE, 1)[0]
    mode = d.get(REQ_GET_I2S_CLOCK_MODE, 1)[0]
    pin_mode = d.get(REQ_GET_I2S_CLOCK_PIN_MODE, 1)[0]
    master_bck = d.get(REQ_GET_I2S_BCK_PIN, 1, wvalue=0)[0]
    slave_bck = d.get(REQ_GET_I2S_BCK_PIN, 1, wvalue=1)[0]
    print("\nAudio input:")
    print(f"  source:          {src} ({'I2S' if src == INPUT_SOURCE_I2S else 'not I2S'})")
    print(f"  I2S clock mode:  {'slave' if mode else 'master'}")
    print(f"  clock pin mode:  {'split' if pin_mode else 'unified'}")
    print(f"  master pair:     BCK=GPIO{master_bck}  LRCLK=GPIO{master_bck + 1}")
    print(f"  slave pair:      BCK=GPIO{slave_bck}  LRCLK=GPIO{slave_bck + 1}")

    return {
        "uart_enabled": bool(enabled),
        "uart_live": bool(st[1]),
        "notify": bool(notify),
        "baud": baud,
        "tx": tx,
        "rx": rx,
        "source": src,
        "clock_mode": mode,
        "pin_mode": pin_mode,
        "slave_bck": slave_bck,
    }


def apply_uart(d: DSPi, tx: int, rx: int, baud: int) -> bool:
    print(f"\nEnabling UART control on TX=GPIO{tx} RX=GPIO{rx} @ {baud} with notifications...")
    # UartCtrlConfig: enabled, tx_pin, rx_pin, notify_enable, baud (u32 LE).
    # notify_enable is what makes the link push state changes instead of
    # having to be polled, and it can only ever be set from here.
    d.set(REQ_SET_UART_CONFIG, struct.pack("<BBBBI", 1, tx, rx, 1, baud))

    st = d.get(REQ_GET_CTRL_IFACE_STATUS, 8)
    print(f"  result: {status_name(st[0])}, live={'yes' if st[1] else 'no'}")
    if st[0] != 0x00 or not st[1]:
        print("  ! Failed. A pin is invalid or already in use.")
        print("    TX must satisfy pin % 4 == 0 and RX pin % 4 == 1, on the same UART.")
        return False
    return True


def apply_audio(d: DSPi, slave_bck: int, persist: bool) -> bool:
    ok = True

    # Order matters. Put the slave pair somewhere safe and switch to split pins
    # before selecting slave clocking, so the device never briefly listens on
    # the same pins it drives.
    print(f"\nSetting slave clock pair to BCK=GPIO{slave_bck} LRCLK=GPIO{slave_bck + 1}...")
    rc = d.set_as_read(REQ_SET_I2S_BCK_PIN, (1 << 8) | slave_bck)
    print(f"  result: {status_name(rc)}")
    ok &= rc == 0x00

    # Split mode gives slave clocking its own pins. In unified mode the device
    # listens on the same pair it drives in every non-slave role, so a wired-up
    # external master and the DSPi can end up driving the same lines.
    print("Setting clock pin mode to split...")
    rc = d.set_as_read(REQ_SET_I2S_CLOCK_PIN_MODE, I2S_CLOCK_PIN_MODE_SPLIT)
    print(f"  result: {status_name(rc)}")
    ok &= rc == 0x00

    print("Setting I2S clock mode to slave...")
    d.set(REQ_SET_I2S_CLOCK_MODE, bytes([I2S_CLOCK_MODE_SLAVE]))

    print("Selecting the I2S input...")
    d.set(REQ_SET_INPUT_SOURCE, bytes([INPUT_SOURCE_I2S]))

    # Everything above is live-only. The clock mode, clock-pin mode and slave
    # BCK pin live in the IO config, which only becomes device-global once
    # saved; until then the device reverts to its stored values on the next
    # reboot, leaving the DSPi driving the clock while the ESP32 also drives it.
    if not persist:
        print("\n  Note: these audio settings are live only and will revert when the DSPi")
        print("  reboots, which would leave it driving BCK/LRCLK against the ESP32.")
        print("  Re-run with --persist once you are happy with them.")
        return ok

    # The clock-mode change is deferred to the device's main loop, so give it a
    # moment to apply; saving too early would persist the previous value.
    time.sleep(1.0)
    print("\nPersisting the IO config (--persist)...")
    rc = d.set_as_read(REQ_SAVE_OUTPUT_CONFIG, 0)
    print(f"  result: {'SUCCESS' if rc == 0 else f'status {rc}'}")
    ok &= rc == 0x00
    time.sleep(0.5)   # flash write: ~45 ms of deaf time, plus settle

    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--apply", action="store_true", help="enable the UART control interface")
    ap.add_argument("--audio", action="store_true", help="also configure the I2S clock-slave input")
    ap.add_argument(
        "--persist",
        action="store_true",
        help="write the audio settings to flash so they survive a reboot "
        "(the UART interface config always persists; the firmware stores it on apply)",
    )
    ap.add_argument("--tx", type=int, default=DEFAULT_UART_TX, help=f"DSPi UART TX GPIO (default {DEFAULT_UART_TX})")
    ap.add_argument("--rx", type=int, default=DEFAULT_UART_RX, help=f"DSPi UART RX GPIO (default {DEFAULT_UART_RX})")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"baud rate (default {DEFAULT_BAUD})")
    ap.add_argument(
        "--slave-bck",
        type=int,
        default=DEFAULT_SLAVE_BCK,
        help=f"slave-pair BCK GPIO, LRCLK is this + 1 (default {DEFAULT_SLAVE_BCK})",
    )
    args = ap.parse_args()

    d = DSPi()
    before = show(d)

    if not args.apply:
        print("\n(Read-only. Re-run with --apply to enable the UART control interface,")
        print(" or --apply --audio to set up the I2S clock-slave input as well.)")
        if before["uart_enabled"] and before["notify"]:
            print("\nThe UART interface already looks ready for esphome-dspi.")
            print(f"Set baud_rate: {before['baud']} in your ESPHome uart: block.")
        return 0

    ok = apply_uart(d, args.tx, args.rx, args.baud)
    if args.audio:
        ok &= apply_audio(d, args.slave_bck, args.persist)
    elif args.persist:
        print("\nNote: --persist only affects the audio settings, so it does nothing without --audio.")
        print("The UART interface config is stored by the firmware as soon as it is applied.")

    # Input-source and clock-mode changes are applied from the device's main
    # loop, not in the control handler, so a readback taken immediately still
    # shows the old value.
    time.sleep(1.5)
    print("\n--- after ---")
    show(d)

    if ok:
        print(f"\nDone. Set baud_rate: {args.baud} in your ESPHome uart: block.")
    else:
        print("\nSomething was rejected; see the messages above.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
