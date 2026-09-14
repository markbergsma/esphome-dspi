# Spectrum analyser (RTA)

The DSPi carries an onboard FFT engine that publishes a third-octave band table
per channel — 34 bands from 10 Hz to 20 kHz at 44.1/48 kHz, 37 up to 40 kHz at
96 kHz. This component reads those bands and hands each new frame to a trigger,
for a display to draw.

It reads the band table only. The raw FFT bins are a separate product, up to 529
bytes, which does not fit this transport in one read and is not implemented.

The device side is specified in `Documentation/Features/spectrum_analyser_spec.md`
in the [DSPi firmware repository](https://github.com/WeebLabs/DSPi); this
document covers only what the component does with it.

## Configuration

```yaml
dspi:
  id: dspi_hub
  uart_id: dspi_uart
  rta:
    tap: input          # or output
    channel_mask: 0x01  # bit N = channel N at that tap
    fft_order: 10       # 8..10 -> 256/512/1024 points
    avg_ms: 250
    peak_decay_db_s: 20
    interval: 50ms
  on_rta_band_frame:
    - lambda: |-
        // x.frame.avg[i] / x.frame.peak[i] are level bytes;
        // x.fraction(v, -60.0f) maps one onto 0.0 .. 1.0 for a bar height.
        ESP_LOGD("rta", "band 0: %.1f dBFS", x.db(x.frame.avg[0]));
```

Omitting the `rta:` block entirely means the component never sends a single RTA
byte, so a device with nothing to draw on pays nothing for the feature.

| Option | Default | Meaning |
|---|---|---|
| `tap` | `input` | `input` = after per-input PEQ, before the matrix. `output` = after gain and delay, exactly what the slots transmit. |
| `channel_mask` | `0x01` | Which channels to analyse at that tap. Must be non-zero. |
| `fft_order` | `10` | 8, 9 or 10 → 256, 512 or 1024 points. Re-checked against what the device reports. |
| `avg_ms` | `250` | Averaging time constant. Clamped by the device to 0..10000. |
| `peak_decay_db_s` | `20` | Peak-hold fall rate; 0 disables peak hold. |
| `interval` | `50ms` | How often to poll one channel. |
| `stale_timeout` | `500ms` | Frame age past which the spectrum is reported as not live. |
| `status_interval` | `2s` | How often to check the device has not lost our config. |
| `auto_enable` | `false` | Start polling as soon as the device is identified. |

Which tap you want depends on what the display is for. The **input** tap sits
before the matrix and master volume, so the bars show what is playing and do not
shrink when the volume is turned down. The **output** tap shows what the DACs
actually receive, including EQ, crossover and volume.

`avg_ms` deserves a look before you settle for the default. It is the DSPi's
own, and it is slow for a display: the band EMA advances `dt / (tau + dt)` per
frame, so at 250 ms against a ~22 ms frame time each frame travels 8% of the way
to its new value and transients are averaged away before they reach the bars.
Something in the 50–100 ms range looks considerably more alive. Values near 0
expose the per-frame variance of a single ~21 ms window as visible noise. Bands
below 250 Hz ignore this setting: they come from a continuous filter bank whose
time constant is the longer of `avg_ms` and one period of the band centre.

## It is off until something asks for it

The analyser is transient on the device: off at boot, never persisted, not part
of presets, and it switches itself off five seconds after the last band read.
Enable it when a spectrum is actually visible, and disable it again afterwards:

```yaml
  - lambda: id(dspi_hub)->set_rta_enabled(true);
```

Disabling sends a courtesy stop and ceases polling. It deliberately does not use
the device's manual-run flag, which would disable that idle timeout: if this
component then crashed or lost the link with the spectrum still enabled, the
DSPi would keep running the FFT and its per-channel bass filter bank — real
audio-path work — for nobody.

## The analyser is shared, and this component does not fight for it

The DSPi has one FFT engine with one global config, so a USB host — DSPi-Console,
most likely — can reconfigure it underneath us. When that happens the component
follows along and draws whatever the device is actually analysing, logging the
change once:

```
[I][dspi]: RTA is configured by another client (tap 1, we asked for 0); following it
[I][dspi]: RTA is back on our own configuration (tap 0)
```

Reasserting our own config instead would be a config war: with Console running,
both sides rewrite the other's tap every couple of seconds, restarting the frame
and clearing the averaging each time. There is no way to tell another client's
deliberate change from a device that rebooted and lost our settings, and of the
two, fighting is much the worse failure. A reboot is caught anyway when the link
drops and the probe re-runs. Our config is reapplied on the next enable, so
disabling and re-enabling takes the analyser back.

One consequence worth knowing: only `tap`, `channel_mask`, `fft_order` and
`flags` are compared. A foreign `avg_ms` or `peak_decay_db_s` is left alone,
because the device clamps those rather than rejecting them, so another client's
averaging can outlive the client that set it — until the DSPi loses power, since
none of this is persisted.

## Refresh rate

**`interval` × the number of channels in `channel_mask`,** since one FFT engine
rotates through them. The device publishes a frame roughly every 21 ms per
channel at 1024 points / 48 kHz, so polling faster than that just re-reads a
frame already seen. The trigger compares sequence numbers and only fires on a
genuinely new frame, so a consumer's redraw follows the device's frame rate
rather than the poll rate.

`interval` is also baud-sensitive. A band frame is 89 bytes on the wire: about
2 ms at 460800 baud, but 8 ms at 115200, where a fast interval takes a
noticeable share of the link away from notifications and control commands.

## Reading the levels

Each band is one byte, half a dB per step, with the zero point reported by the
device (`level_zero`, normally 243 = 0 dBFS, so 255 is +6 dBFS). A byte of `0`
means *floor* — no data, or below the bottom of the scale — and is not −121.5 dB
of signal. Use the helpers on the trigger argument rather than doing this
arithmetic in a lambda:

| Helper | Returns |
|---|---|
| `x.db(v)` | Level in dBFS. |
| `x.fraction(v, floor_db)` | 0.0 … 1.0 across `[floor_db, 0 dBFS]`, clamped, with floor pinned to 0.0. |
| `x.is_floor(v)` | Whether the band is empty. |
| `x.live` | False when the analyser is idle or the data has gone stale. |

`x.live` is worth honouring: the trigger fires once more on the way into that
state, carrying the last good frame, so a display can dim what it has instead of
showing a frozen spectrum that looks live.

The hub also exposes `rta_band_centre_hz(i)` for axis labels, read from the
device rather than hard-coded, plus `rta_available()`, `rta_caps()`,
`rta_status()` and `rta_foreign_config()`. If the band-centre table cannot be
read the spectrum still works; only the labels are missing.

Choosing a floor for a bar graph is worth measuring rather than assuming. Real
per-band swings run 20–44 dB, with minima reaching −73 dB around 10 Hz and
−82 dB at 20 kHz, so a −60 dB floor does clip both extremes — while a wider one
costs resolution in the middle octaves, where most of the music is.

## Edge cases

**At `fft_order` 8 or 9 some bands are permanently empty.** Bands up to 200 Hz
come from a continuous filter bank and are always populated, but above that a
band with no FFT bin inside it reads the floor and is never faked from a
neighbour. At 48 kHz the lowest FFT-resolved band is 200 Hz at 256 points and
100 Hz at 512; at 1024 points there is no gap.

**Unsupported firmware is detected once.** A firmware without the analyser, or
one speaking a different RTA protocol version, is reported at enable time;
`rta_available()` then stays false and nothing further is sent. The protocol
version is checked strictly rather than parsed best-effort, because V3
renumbered the band indices relative to V2 — a mismatched parse would draw a
plausible but wrong spectrum.

**The first frames after a start carry no bands.** The engine reports zero bands
while idle, since its band table is only built once it runs, so the read that
starts it returns the frame from before that happened. Those frames are dropped
rather than published, to avoid blanking a display for a frame.
