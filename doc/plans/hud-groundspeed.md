# Plan: HUD — GPS ground speed under the airspeed box ("G 42")

## Context

The HUD airspeed tape shows pitot IAS (km/h). GPS ground speed is already
measured on the air side (`gs_mps` in `TelemetryMessage`), forwarded by
`espnow_source.py` into `VFR_HUD.groundspeed` and `GLOBAL_POSITION_INT`
vx/vy, and stored in the snapshot — but nothing displays it. Goal: show it
under the airspeed value box with a `G` prefix (e.g. `G 42`), mirroring the
`R`-prefixed radar altitude drawn under the altitude box. IAS vs GS
comparison gives an instant wind read and a sanity check on the pitot.

No firmware or wire-protocol change — everything needed is already on the
wire.

## Design decisions

- **Source: `GLOBAL_POSITION_INT` velocities, not `VFR_HUD.groundspeed`.**
  `espnow_source.py` maps NaN `gs_mps` (no fix) to `0.0` in VFR_HUD
  (`espnow_source.py:217`), so a VFR-based value cannot distinguish
  "no fix" from "standing still". GPI is only published while there is a
  fix (`_publish_gps`, `espnow_source.py:264`), so its staleness naturally
  encodes "no GPS ground speed" → the `G` line simply disappears, exactly
  like the `R` line disappears without terrain/calibration.
- **Units: km/h**, same as the airspeed tape (`ground speed = hypot(vn, ve) * 3.6`).
- **Layout mirrors the altitude side**: the `R` text sits at
  `(x_axis + w*0.105, cy + h*0.065)` (`hud.py:260-266`); the `G` text goes
  to the symmetric spot on the left, `(x_axis - w*0.105, cy + h*0.065)`,
  regular font, `anchor="center"`.
- **Format `f"G {int(round(gs_kmh)):3d}"`** — same 3-digit width as the
  airspeed box (`hud.py:246`), so `G  42` per the user's example.

## Changes

### 1. `cocpitView/cockpitview/telemetry.py` — new snapshot property

Next to `velocity_ned` (line 156-161), add:

```python
@property
def groundspeed_kmh(self):
    """GPS ground speed in km/h, or None without a (fresh) fix."""
    if not self._fresh(self.gpi_ts):
        return None
    return math.hypot(self.vn, self.ve) * 3.6
```

### 2. `cocpitView/cockpitview/hud.py` — draw the G line

- `draw_airspeed(self, surface, kmh)` → `draw_airspeed(self, surface, kmh,
  gs_kmh=None)` (`hud.py:242`); after the value box, mirror the radar-altitude
  idiom from `draw_altitude` (`hud.py:257-266`):

```python
if gs_kmh is not None:
    # GPS ground speed ("G" prefix); IAS vs GS difference = wind component
    self._text(
        surface,
        f"G {int(round(gs_kmh)):3d}",
        (x_axis - self.w * 0.105, self.cy + self.h * 0.065),
        anchor="center",
    )
```

- Call site `hud.py:368`:
  `self.draw_airspeed(surface, snap.airspeed_kmh, snap.groundspeed_kmh)`.

### 3. Docs

- `CLAUDE.md` "Current state": mention the `G` ground-speed line under the
  airspeed box in the HUD status description.
- Save this plan as `doc/plans/hud-groundspeed.md` (repo convention, cf.
  `doc/plans/pitot-ms4525do.md`).

## Verification

1. `cd cocpitView && ./run.sh --emulator` — the emulator publishes
   GLOBAL_POSITION_INT with velocities (`emulator_source.py:60,79`), so
   `G <n>` must appear under the airspeed box. The emulator's airspeed is a
   constant while its ground speed varies along the flight profile, so the
   `G` value must visibly differ from the tape and change over time.
2. Hardware bench test (air board without a GPS fix indoors): the `G` line
   is absent while `GPS --`/acquiring, appears once the HUD shows `GPS 3D`.
3. Nothing else changes: tape, `---` no-data box, altitude side untouched.
