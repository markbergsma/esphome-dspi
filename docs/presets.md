# Presets

The DSPi stores ten preset slots. A slot is not a scene of a few settings — it
is the **complete DSP state**: every EQ band, the crossovers, the delays, the
matrix mixer, output gains and mutes, the output configuration, channel names,
and master volume too if the device is in per-preset master-volume mode.

So one Home Assistant `select` reconfigures the entire processor. That is what
makes this worth an entity where individual parameters mostly are not.

```yaml
select:
  - platform: dspi
    dspi_id: dspi_hub
    type: preset
    name: Preset
    slots:
      0: Default
      1: Movie
      3: Night

text_sensor:
  - platform: dspi
    dspi_id: dspi_hub
    type: preset_name
    name: Preset Name
```

Only loading is supported. Saving, deleting and renaming presets are not
exposed: they write flash, and an accidental tap in Home Assistant would
overwrite a stored configuration with no way back. Use DSPi Console for that.

## `slots:` is required, and the labels are yours

`slots:` is a mapping of slot number to label. The order you write is the order
Home Assistant shows, the set may be sparse, and the slot number — not the
position in the list — is what goes on the wire.

**The labels do not come from the device**, even though the device stores a name
for every slot. ESPHome sends a select's options to Home Assistant once, in the
entity-list message at connect, and its `SelectTraits` is documented as "set
once at startup". A name read asynchronously over the UART link arrives after
that moment has passed, and would go stale again every time a preset was
renamed. Rather than have an option list that is occasionally a lie, the labels
are fixed at compile time and the device's own name is published separately by
the `preset_name` text sensor, where it can be updated honestly.

There is deliberately no default. A fresh DSPi names only slot 0 and marks none
occupied, so a generated list would be ten indistinguishable entries — and
picking one that has never been saved is not a no-op, it loads factory
defaults. Listing the presets you actually use costs one line each and makes
the entity readable.

To see what your device holds:

```console
$ python3 tools/dspi_setup.py
...
Presets:
  startup:       slot 0
  active:        slot 1
    0  saved  'Default'
    1  saved  'Movie' <- active
    2  empty  ''
    ...
```

That is read-only, and it reads over USB, so it works before the UART link is
configured.

## What the entities do

| Entity | Shows |
|---|---|
| `select` / `type: preset` | The slot the device is on, as one of **your** labels. Setting it loads that preset. |
| `text_sensor` / `type: preset_name` | The **device's** name for whichever slot is active. |

The text sensor is the one that stays honest when a preset is loaded from
somewhere this config does not know about — DSPi Console, or a knob wired to
the DSPi itself. A slot the device has never named publishes as `Slot N` rather
than as a blank entity.

If the DSPi is sitting on a slot your `slots:` list omits, the select is left
alone rather than made to show something false. `dump_config` names any saved
slot you have not listed, which is the likely explanation if the entity ever
looks stuck.

## Loading is deferred, and confirmed by reading back

The DSPi does not apply a preset in the command handler. It sets a pending flag
and does the work later in its main loop, behind a flash write and a
synchronised pipeline reset — roughly 45 ms during which the control link may
time out or answer BUSY. The status byte it returns immediately means
*accepted*, not *applied*.

This component therefore never publishes the slot you picked. It re-reads the
active slot after a settle delay and publishes only what the device confirms,
so a load that is rejected or silently ignored shows up as the entity springing
back rather than as a UI that disagrees with the hardware. Retries and the
blackout are handled by the hub's normal backoff.

The settle delay is not padding. The firmware pushes its `PRESET_LOADED`
notification at the *start* of the load and only updates the active slot at the
end, so a read issued the instant the command is accepted would return the slot
you were on before.

Because a preset can also move master volume, the input source and the output
configuration, the device follows a load with a `BULK_INVALIDATED` notification
and this component re-reads everything it publishes. The volume, mute and
source entities update on their own; nothing extra is needed.

## Loading an unsaved slot resets the DSP

A slot that has never been saved is still loadable, and loading it applies
**factory defaults** rather than failing. That is the firmware's behaviour, not
this component's. If you list such a slot, selecting it will reset the live DSP
configuration — the component logs a warning when it happens, but it does not
refuse, because on a device where you have deliberately kept a slot empty as a
"reset to defaults" entry that is exactly what you want.

The safe habit is to list only slots you have saved. `dump_config` and
`dspi_setup.py` both tell you which those are.
