# Cockpit View

FPV video z USB kamery (5.8G přijímač → "AV TO USB2.0" grabber) s HUD overlayem
ve stylu F-16 (NAV mód). Telemetrie přichází jako MAVLink forward
z QGroundControl přes UDP.

## HUD prvky

- **airspeed** (vlevo, km/h) — páska + rámeček s aktuální hodnotou
- **altitude** (vpravo, m, relativní) — páska + rámeček
- **magnetic heading** (nahoře) — páska s desítkami stupňů (`09` = 090°) + rámeček
- **horizont + pitch ladder** — příčky à 5°, plné nad / čárkované pod horizontem,
  rotované a posunuté podle roll/pitch; měřítko odpovídá vertikálnímu FOV kamery (`--fov`)
- **flight path marker** — kroužek s křidélky, poloha z vektoru rychlosti
  (GLOBAL_POSITION_INT); bez rychlostních dat (HW bridge) se místo něj kreslí boresight kříž
- **roll indicator** — oblouk dole s ticky 0/±10/±20/±30/±45°
- **výška nad terénem** (`R 123` pod rámečkem výšky, styl radar altimeter) —
  baro AMSL výška mínus elevace terénu z ČÚZK DMR 5G gridu (viz níže);
  bez pozice nebo mimo stažený grid se údaj nekreslí
- ztráta telemetrie (>1 s) → blikající `NO TELEMETRY`; chybějící údaje → `---`

## Setup

```bash
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt
```

## Spuštění

```bash
./run.sh                        # venv python, OTG kamera, MAVLink :14445
./run.sh --fullscreen --fov 45  # argumenty se předávají dál
./.venv/bin/python cockpit_view.py --no-camera   # jen HUD bez kamery
```

| Volba | Default | Význam |
|---|---|---|
| `--mavlink-port` | 14445 | UDP port, kam QGC forwarduje MAVLink |
| `--camera-name` | `AV TO USB2.0` | jméno kamery, na kterou se aplikace výhradně váže |
| `--camera` | — | vynutit cv2 index (přeskočí hledání podle jména) |
| `--width`/`--height` | 960×640 | velikost okna (okno lze měnit myší) |
| `--fullscreen` | — | start přes celou obrazovku |
| `--fov` | 40 | vertikální FOV kamery [°] — měřítko pitch ladderu |
| `--no-camera` | — | přeskočit kameru (test pattern) |
| `--terrain` | `terrain/data/terrain.npz` | grid elevací terénu pro AGL (z `terrain/terrain_fetch.py`) |
| `--dump-frame PATH` | — | debug: po 2 s uloží PNG snímek a skončí (headless) |

Klávesy: `ESC`/`Q` konec, `F` fullscreen.

Kamera se vybírá **podle jména** (AVFoundation), ne podle indexu — po vypojení
OTG kamery se aplikace nikdy nepřepne na jinou (vestavěnou) kameru; ukazuje
`NO VIDEO` a čeká, až se zařízení znovu objeví.

Při prvním spuštění si macOS vyžádá povolení kamery pro terminál
(Nastavení → Soukromí → Kamera).

## Zapojení telemetrie

```
model / emulátor --MAVLink UDP:14550--> QGroundControl --forward UDP:14445--> cockpit_view.py
```

V QGC: **Application Settings → MAVLink → MAVLink Forwarding**:
Enable + host `localhost:14445`.

### Test bez QGC (emulátor přímo)

```bash
cd ../emulator && ./.venv/bin/python mpu6050_emulator.py --port 14445
```

## Výška nad terénem (AGL)

Terén se čte offline z předstažené mřížky ČÚZK **DMR 5G** (otevřená data,
vertikální přesnost ~0,2 m; výšky Bpv ≈ AMSL). Stažení pro létací oblast
(default: letiště ze sekce [site] v esp32/local.ini ±10 km, krok 5 m, ~20 MB
do `terrain/data/terrain.npz`):

```bash
./.venv/bin/python terrain/terrain_fetch.py
./.venv/bin/python terrain/terrain_fetch.py --lat 50.0 --lon 15.0 --radius-km 10
```

Skript si data vyžádá z veřejného ImageServeru ČÚZK rovnou ve WGS84 mřížce
a po stažení ji ověří proti bodovým dotazům na server. Za letu pak
`AGL = AMSL (baro) − elevace(lat, lon)` s bilineární interpolací; pozice
z `GLOBAL_POSITION_INT`, výška preferenčně z `ALTITUDE.altitude_amsl`.

Pozor: DMR 5G pokrývá jen ČR — nad slovenskou stranou hranice údaj zmizí.

### Reálný MPU-6050 (jen attitude)

`../emulator/mpu6050_hw_bridge.py` posílá jen ATTITUDE — HUD pak ukazuje
horizont/roll/heading (z yaw), rychlost a výška zůstanou `---` a místo
flight path markeru je boresight kříž.

## Soubory

- `cockpit_view.py` — main, pygame render loop
- `hud.py` — F-16 HUD renderer
- `telemetry.py` — MAVLink receiver (vlákno, thread-safe snapshot)
- `camera.py` — USB kamera přes OpenCV/AVFoundation (vlákno, poslední frame)
- `terrain/__init__.py` — elevace terénu z mřížky, bilineární lookup
- `terrain/terrain_fetch.py` — jednorázové stažení mřížky z ČÚZK DMR 5G
- `terrain/data/` — stažené mřížky (`terrain.npz`)
