# esphome-dspi

An ESPHome component for controlling a [DSPi](https://github.com/WeebLabs/DSPi)
audio DSP over its UART control interface.

It depends on nothing but `uart`, so it is useful on any ESPHome device — a wall
knob, a display, a rack controller — not only in the
[Sendspin player](https://github.com/markbergsma/dspin) it was first written for.

> **Under active development.** The configuration interface may change without
> backwards compatibility, and there are no version tags yet, until further
notice.. Track `@main` to follow along, or pin a specific commit if you need
something that stays put.

## Supported functionality

Currently only basic functionality is supported:
- User & master volume (in dB)
- Mute (all outputs)
- Input source select
- Spectrum analyser (RTA) band data, for driving a display — see [docs/spectrum-analyzer.md](docs/spectrum-analyzer.md)

## Quick start

### 1. Wiring

Three wires between the two boards — two signals and a ground:

| Signal | DSPi | | ESP32 | ESPHome key |
|---|---|---|---|---|
| DSPi transmits | GPIO 16 | → | any free input-capable GPIO | `rx_pin` |
| DSPi receives | GPIO 17 | ← | any free output-capable GPIO | `tx_pin` |
| Ground | GND | — | GND | — |

**Cross TX and RX.** The DSPi's TX goes to the ESP32's RX and vice versa; wiring
them straight through is the usual reason nothing responds, and it looks
identical to a dead link from either side because the DSPi only ever transmits
in reply.

**A common ground is required.** Two separately powered boards with no shared
ground will not communicate reliably, if at all.

**Do not connect the 5 V or 3.3 V rails.** Each board is powered on its own
(the DSPi over USB); the link carries signals only.

Everything is 3.3 V logic — **never drive the DSPi's RX pin above 3.3 V**. Level
shift a 5 V host. No flow-control lines are used, so RTS/CTS can be left
unconnected.

GPIO 16 and 17 are the DSPi's defaults and can be moved with `--tx` / `--rx`,
but the RP2350 pin mux constrains the pair: TX must satisfy `pin % 4 == 0`, RX
`pin % 4 == 1`, and both must be on the same UART instance. Valid pairs are
0/1, 4/5, 8/9, 12/13, 16/17, 20/21 and 28/29. The ESP32 side has no such
constraint — any two free GPIOs will do.

### 2. One-time DSPi setup

DSPi ships with its UART control **disabled**, and first needs to be configured
before the ESPHome dspi component can control it. This can be done manually, or
using the supplied script.

#### dspi_setup.py

```bash
pip install pyusb
python3 tools/dspi_setup.py                     # read-only: show current state
python3 tools/dspi_setup.py --apply             # enable UART control + notifications
python3 tools/dspi_setup.py --apply --persist   # make changes and persist them
```

`--apply` is all this component needs. `--persist` also stores them to survive
for DSPi reboots. Adding `--audio` additionally switches the DSPi
to a clock-slave I2S input, which only matters when something is feeding it
audio over I2S — see [DSPin](https://github.com/markbergsma/dspin).

#### Manual configuration

The UART interface can also be configured manually, e.g. witin DSPi Console,
"Control Interfaces".

- Activate "Enable UART"
- Baud Rate: 460800
- Push Notifications: enabled

### 3. ESPHome config

```yaml
external_components:
  - source: github://markbergsma/esphome-dspi@main
    components: [dspi]

uart:
  - id: dspi_uart
    tx_pin: GPIO38      # -> DSPi GPIO 17 (its UART RX)
    rx_pin: GPIO21      # <- DSPi GPIO 16 (its UART TX)
    baud_rate: 460800
    data_bits: 8
    parity: NONE
    stop_bits: 1
    rx_buffer_size: 512  # see "Gotchas"

dspi:
  id: dspi_hub
  uart_id: dspi_uart

number:
  - platform: dspi
    dspi_id: dspi_hub
    type: user_volume        # what a listener turns
    name: Volume
  - platform: dspi
    dspi_id: dspi_hub
    type: master_volume      # the output ceiling; a config setting
    name: Master Volume

switch:
  - platform: dspi
    dspi_id: dspi_hub
    name: Mute

select:
  - platform: dspi
    dspi_id: dspi_hub
    name: Input Source
```


## Configuration

### `dspi` hub

| Option | Default | Meaning |
|---|---|---|
| `uart_id` | — | The UART bus. Must be 8N1; this is validated. |
| `request_timeout` | `400ms` | Per-request response timeout. |
| `flash_timeout` | `1500ms` | Timeout for commands that may write flash. |
| `max_retries` | `4` | Retries before a command is dropped. |
| `backoff_base` | `60ms` | First retry delay; doubles up to 4 steps. |
| `poll_interval` | `0s` | Fallback polling. Leave at 0 when notifications are on. |
| `refresh_debounce` | `250ms` | Coalescing window for notification-driven re-reads. |
| `max_bytes_per_loop` | `128` | UART bytes drained per `loop()` call. |
| `boot_input_source` | unset | `usb` / `spdif` / `i2s`. Claimed once at boot, if different. |
| `rta` | unset | Spectrum analyser. Absent means no RTA traffic at all. See [docs/spectrum-analyzer.md](docs/spectrum-analyzer.md). |

### `number` — the two volumes

The DSPi has two independent gain stages that multiply together, and they are
not interchangeable, so `type` is **required** rather than defaulted:

```
final = sample * output_gain * (user_volume * master_volume)
```

| `type` | What it is | Range |
|---|---|---|
| `user_volume` | What a listener turns. The same field the USB host's volume slider drives, and the one that feeds loudness compensation. | −60..0 dB |
| `master_volume` | The device-side output ceiling — how loud the system may ever get. A configuration setting. | −127..0 dB, −128 = mute |

**Bind `user_volume` to anything user-facing** — a knob, a media player's
volume, a dashboard slider. Master volume deliberately does *not* affect
loudness compensation, because it is a ceiling rather than a listening
position; turning it down would otherwise apply the wrong equal-loudness
contour.

`master_volume` defaults to `entity_category: config` so it stays out of the
way of day-to-day control. Override it if you disagree.

`min_value` defaults to **−60 dB** for both. For master volume the protocol
allows −127, but a slider spanning that is unusable since nearly all its travel
is inaudible; the firmware range is still enforced underneath, so a lambda can
use the whole span. For user volume −60 *is* the firmware's own clamp, and a
`min_value` below it is rejected at config time rather than silently ignored.

### `select` — input source

**The DSPi runs exactly one input source at a time.**  So this is a source selector
— closer to the source button on an AV receiver than to a mixer input. The DSPi's
**matrix mixer sits downstream** and routes whichever channels the active source
provides:

```
one active source ──▶ its 2–8 input channels ──▶ matrix mixer ──▶ 9 outputs
   (this select)                                  (routing/mixing)
```

Seven sources exist, and channel counts differ:

| `sources:` key | Wire value | Channels | Availability |
|---|---|---|---|
| `usb` | 0 | 2/4/6/8 | always |
| `spdif` | 1 | 2 | always |
| `i2s` | 2 | 2/4/6/8 | always |
| `adat` | 3 | 8 | RP2350 only, **off until enabled** |
| `spdif2` / `spdif3` / `spdif4` | 4 / 5 / 6 | 2 each | **off until enabled** |

`sources:` chooses which to expose and in what order, defaulting to
`[usb, spdif, i2s]` — the set every DSPi can always select. ADAT and S/PDIF 2–4
must be enabled on the device first (over USB), so listing them by default would
offer options that do nothing.

```yaml
select:
  - platform: dspi
    dspi_id: dspi_hub
    name: Input Source
    sources: [i2s, spdif, spdif2]
    spdif2: "Optical Rear"      # relabel any source freely
```

Labels are cosmetic; the wire value comes from the key, so reordering or
omitting sources is safe.

**The component asks the device what it can actually select.** At boot it reads
`REQ_GET_SPDIF_INPUT_CONFIG` (`0xEF`) and the ADAT enable and pin, and reports
the result in `dump_config`:

```
[C][dspi]:   Selectable input sources: USB, S/PDIF, I2S
```


## Using it without any entities

The hub exposes its control surface directly, so a lambda can drive it on a
device that configures no `number`, `switch` or `select` at all:

```yaml
  on_...:
    - lambda: |-
        id(dspi_hub).set_user_volume_db(-18.0f);     // the listening control
        id(dspi_hub).set_master_volume_db(-6.0f);    // the ceiling
        id(dspi_hub).set_user_mute(true);
        id(dspi_hub).set_input_source(2);   // I2S
```

This is the basis for adding a rotary encoder or a display later. Note that the
component never blocks in `loop()`, which is what makes that possible:
ESPHome's scheduler is cooperative, so a component that stalled would starve
whatever else the device is doing, audio and UI included.

## DSPin: the Sendspin player

[**DSPin**](https://github.com/markbergsma/dspin) is a Sendspin network audio
player built from an ESP32-S3-BOX-3 and a DSPi, using this component for
control. It lives in its own repository, along with everything audio-specific:
I2S clock-slave wiring, the PSRAM requirement, and the measured clock behaviour.

## Gotchas

**Set `rx_buffer_size: 512`.** ESPHome's UART defaults to 256 bytes and the
ESP-IDF driver drops silently on overflow; a burst of notifications during a
preset load can outrun it. There is no overflow signal to detect, only the
notification sequence gap after the fact.

**An `OK` on a write is not proof the value applied.** The DSPi answers OK for
any well-formed frame, including one whose value its own handler then rejects.
This component always reads back, which is also how it publishes state.

**An offline DSPi leaves entities stale.** ESPHome's `select` has no "unknown"
state, so there is currently no honest way to express "the link is down".

## Development

```bash
./tests/run.sh
```

Runs the framing unit tests with a host compiler and validates both example
configs. No hardware or ESP toolchain needed.

## License

GNU General Public License v3.0 — see [LICENSE](LICENSE).
