# Agent Guidelines & Repository Rules

This document defines architectural standards, protocol constraints, and coding guidelines for AI coding assistants working in the `esphome-dspi` repository.

This repo holds **one ESPHome component**, `dspi`, which controls a [DSPi](https://github.com/WeebLabs/DSPi) audio DSP over its UART control interface. The Sendspin player built on it lives separately, in [`dspin`](https://github.com/markbergsma/dspin).

**Breaking changes are acceptable, for now.** The component is under active development and carries no backwards-compatibility obligation: there are no version tags, and consumers track `@main`. So when an interface turns out to be wrong, change it outright — rename the config key, reshape the state, drop the old spelling. Do **not** add deprecation shims, accept both an old and a new key name, or keep a superseded code path alive out of caution; that accumulates cruft that serves nobody. Update every call site and example here and in `dspin` instead, and say in the commit message that it is a breaking change. This will change once the interface stabilises and tags appear.

---

## 1. The Standalone Invariant

**The component depends on `uart` and nothing else.** It must compile and run on a device with no audio path whatsoever — a wall knob, a rack panel, a display.

- Never add a dependency on `speaker`, `i2s_audio`, `media_player`, `sendspin`, or any other audio component, in `DEPENDENCIES`, in an `AUTO_LOAD`, or via an `#include`.
- [`tests/standalone.yaml`](tests/standalone.yaml) contains no audio components at all and is compiled in CI precisely to enforce this. If a change makes it fail, the change is wrong, not the test.
- Audio-adjacent *control* features are fine: the input-source `select` and `boot_input_source` speak the DSPi's control protocol and carry no audio dependency.

---

## 2. Protocol Fidelity

The wire protocol is **upstream**, defined by the DSPi firmware. This repo transcribes it; it does not design it.

- **Mirror the firmware's identifier names exactly** — opcodes, struct fields, constants — so [`protocol.h`](components/dspi/protocol.h) can be read against the firmware's `config.h` without translation. Carry that name through every layer, including the user-facing YAML key. Put clarifying description in prose and comments, never in a renamed identifier. Only the Home Assistant display string is free to choose.
- **Authoritative sources**, in the DSPi repo:
  - `Documentation/Features/control_interfaces_spec.md` — the UART transport
  - `Documentation/commands.md` — the opcode catalogue
  - `Documentation/Features/notification_protocol_v2_spec.md` — notification packets
  - `firmware/DSPi/config.h` — opcode definitions, the real source of truth
- **The docs lag the firmware.** `commands.md` documents `WireBulkParams` at V14 while the firmware is at V30, and its write-as-read opcode list omits eight entries. When docs and source disagree, **the source wins** — verify against `config.h` and `vendor_commands.c`.
- **Frame type is a property of the opcode, not of the verb.** A set of mutating commands are dispatched on the DSPi's GET path with parameters packed into `wValue` ("write-as-read"); sending them as SET frames stalls. Keep a per-opcode table; never infer the frame type from "am I reading or writing". Determine membership empirically: anything below `vendor_handle_get()` in `vendor_commands.c` is write-as-read.
- **Never assume a struct layout.** Derive offsets from the firmware headers and verify against an independent command path before trusting a parse.

---

## 3. Design Invariants

These are deliberate and were each arrived at for a reason. Do not "optimise" them away without reading the reason first.

- **Notifications are a hint, not a payload.** `NOTIFY_EVT_PARAM_CHANGED` addresses its field by byte offset into `WireBulkParams`, whose layout is tied to a wire-format version that has moved repeatedly. The component decodes only the 4-byte v2 header and responds by re-reading the handful of values it publishes. Parsing those offsets would be faster and would silently publish garbage after a firmware update.
- **Publish state on readback, never optimistically from `control()`.** This is what lets changes made elsewhere — a USB host, a knob on a DSPi control surface — appear in Home Assistant. A `CTRL_STATUS_OK` on a write is *not* proof the value applied; the DSPi answers OK for any well-formed frame, including one its own handler then rejects.
- **`loop()` must never block.** No `delay()`, no `yield()`, no spinning on `available()`, no blocking read. Timeouts and retries are `millis()` comparisons, and RX drain is bounded per call. ESPHome's scheduler is cooperative, so a stall here starves the audio path and the UI on the same chip.
- **Keep probing until the device answers.** If the component has not identified the DSPi, it must keep retrying regardless of state. Dropping the probe and waiting leaves it silent forever, and also breaks the cases where the DSPi is powered up after the ESP32, or where a wiring fault is repaired on a running device.
- **Retry only retryable statuses.** `BUSY`, `BULK_LOCKED`, `CRC_ERROR` and timeouts are retryable. `ERROR`, `BLOCKED`, `OVERSIZE`, `FRAME_ERROR` are permanent — retrying `BLOCKED` in particular is an infinite loop, since it means the command is refused on this transport by design.
- **One request in flight.** The spec is explicit that pipelining is undefined.

---

## 4. Layering

- [`protocol.h`](components/dspi/protocol.h), [`crc16.h`](components/dspi/crc16.h) and [`framing.h`](components/dspi/framing.h) carry **no ESPHome dependencies**. This is what lets [`tests/test_framing.cpp`](tests/test_framing.cpp) exercise the real framing and parser code with a host compiler rather than a reimplementation. Keep it that way.
- The hub exposes state to entities through the abstract `DSPiStateListener` rather than holding `number::Number` / `switch_::Switch` / `select::Select` pointers, so it still compiles on a device that configures none of them.
- Entity platforms are thin: translate a Home Assistant action into a hub call, and publish what the hub reports.

---

## 5. Build & Verification Workflows

- **Host tests** need no hardware and no ESP toolchain:
  ```bash
  ./tests/run.sh
  ```
  This compiles the framing tests with `-Werror` and validates both YAML configs. Run it after any change.
- **Compile the standalone guard** for a real target:
  ```bash
  esphome compile tests/standalone.yaml
  ```
- **Local builds of `examples/control-only.yaml`** need the `github://` source rewritten to this working tree; `tests/run.sh` shows the substitution. Until the repo is published, that rewrite is required for any local flash.
- **Hardware verification**, when a DSPi is attached over USB:
  ```bash
  python3 tools/dspi_setup.py           # read-only; shows interface + audio state
  ```
  The strongest end-to-end check is the notification path: change a value from the USB host and confirm the Home Assistant entity follows without polling.
- **Credential Safety**: never commit network credentials or keys. Keep them in `secrets.yaml` (excluded by `.gitignore`) and provide sanitized templates in `secrets.yaml.example`.

---

## 6. Git

- **Never make a git commit without explicit approval from the user.** Suggest commit messages freely; they are for the user to review or edit.
- Add an **`Assisted-by: <model name>`** trailer when you contributed to a change, e.g. `Assisted-by: Claude Opus 5`.
- Commit messages should summarize *what* and *why*, but not explain the implementation in extensive detail. 2-3 Paragraphs should usually suffice. Record findings either in comments in the source where appropriate, or write a separate design/documentation file.
