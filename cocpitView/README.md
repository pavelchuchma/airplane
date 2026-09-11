# Cockpit View

FPV video z USB kamery (5.8G přijímač → "AV TO USB2.0" grabber) s HUD overlayem
ve stylu F-16 (NAV mód). Telemetrii si aplikace čte sama — defaultně ze
sériového portu ground bridge ESP32, s `--emulator` ze syntetického letového
profilu, s `--replay` ze zaznamenaného letu (`logs/*.cvlog`) —
a přeposílá ji jako MAVLink do QGroundControl (UDP, default `127.0.0.1:14550`).
QGC je tedy jen volitelný downstream konzument; HUD běží i bez něj.

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
- **vzdálenost od místa startu** (`DST 1.5km (+358m)` vlevo dole nad GPS řádkem) —
  přepona (šikmá vzdálenost) k místu, kde se zamkla první důvěryhodná GPS pozice
  za stání, a rozdíl barometrické výšky proti němu; před zámkem se řádek nekreslí,
  bez GPS pozice zmizí jen vzdálenost (`DST --- (+358m)`), bez bara jen výškový rozdíl
- **stavové řádky** — `GPS 3D 12 ±4M` (vlevo dole), `LNK -72 U2% D0%` a
  `BAT 11.4 3.80/3.79/3.81 Δ20` (vpravo dole; bliká při poklesu článku pod 3.2 V
  nebo rozvážení > 150 mV)
- **čas v záznamu** (`00:02:59.9` dole na střed) — jen při `--replay`: pozice
  v logu od jeho začátku, tedy číslo, které se dá vložit do `--replay-start`
- ztráta telemetrie (>1 s) → blikající `NO TELEMETRY`; chybějící údaje → `---`

## Setup

```bash
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt
```

## Spuštění

```bash
./run.sh                        # venv python, OTG kamera, telemetrie z air ESP32
./run.sh --fullscreen --fov 45  # argumenty se předávají dál
./.venv/bin/python -m cockpitview --emulator --no-camera   # HUD nad emulátorem, bez kamery
./run.sh --replay logs/2026-08-29_123339.cvlog --replay-start 3:15  # přehrání letu
```

| Volba | Default | Význam |
|---|---|---|
| `--emulator` | — | syntetický letový profil místo čtení air ESP32 |
| `--replay FILE.cvlog` | — | přehrání zaznamenaného letu místo živé linky (nezapisuje nový log, implikuje `--no-camera`) |
| `--replay-speed` | 1.0 | násobič rychlosti přehrávání |
| `--replay-start T` | 0 | skok na čas `SS`/`MM:SS`/`HH:MM:SS` logu (časy z `flightlog info`); to před ním se přetočí přes kalibrátory |
| `--replay-end T` | — | konec přehrávání |
| `--replay-cold` | — | část před `--replay-start` zahodit místo přetočení (simuluje start aplikace uprostřed letu) |
| `--replay-max-gap` | 3 | strop čekání přes výpadek telemetrie [s] (logový čas přeskočí celý) |
| `--serial-port` | autodetekce | sériový port air ESP32 (default: podle factory MAC z `esp32/platformio.ini`) |
| `--baud` | 460800 | baud rate sériové linky |
| `--rate` | 20 | frekvence telemetrie emulátoru [Hz] |
| `--qgc-host` | 127.0.0.1 | IP adresa QGroundControl pro MAVLink forward |
| `--qgc-port` | 14550 | UDP port QGroundControl |
| `--system-id` | 1 | MAVLink system id vozidla |
| `--camera-name` | `AV TO USB2.0` | jméno kamery, na kterou se aplikace výhradně váže |
| `--camera` | — | vynutit cv2 index (přeskočí hledání podle jména) |
| `--width`/`--height` | 960×640 | velikost okna (okno lze měnit myší) |
| `--fullscreen` | — | start přes celou obrazovku |
| `--fov` | 40 | vertikální FOV kamery [°] — měřítko pitch ladderu |
| `--no-camera` | — | přeskočit kameru (test pattern) |
| `--terrain` | `data/terrain.npz` | grid elevací terénu pro AGL (z `cockpitview/terrain_fetch.py`) |
| `--dump-frame PATH` | — | debug: po 2 s uloží PNG snímek a skončí (headless) |

Klávesy: `ESC`/`Q` konec, `F` fullscreen.

Kamera se vybírá **podle jména** (AVFoundation), ne podle indexu — po vypojení
OTG kamery se aplikace nikdy nepřepne na jinou (vestavěnou) kameru; ukazuje
`NO VIDEO` a čeká, až se zařízení znovu objeví.

Při prvním spuštění si macOS vyžádá povolení kamery pro terminál
(Nastavení → Soukromí → Kamera).

## Zapojení telemetrie

```
ground ESP32 (USB, ESP-NOW) ──┐
--emulator (letový profil) ───┤──> cockpit view ──MAVLink UDP:14550──> QGroundControl
--replay (logs/*.cvlog) ──────┘         │
                                        └──> HUD (přímo, bez QGC)
```

Defaultní režim čte CSV řádky z air ESP32 (port se hledá podle factory MAC,
probe board resetuje — viz `esp32/tools/board_id.py`). Když board chybí nebo
linka spadne, aplikace to zkouší dál (HUD ukazuje `NO TELEMETRY`) a po
připojení se sama chytne. QGC stačí spustit — na UDP 14550 poslouchá
automaticky a vozidlo se objeví samo.

## Výška nad terénem (AGL)

Terén se čte offline z předstažené mřížky ČÚZK **DMR 5G** (otevřená data,
vertikální přesnost ~0,2 m; výšky Bpv ≈ AMSL). Stažení pro létací oblast
(default: letiště ze sekce [site] v esp32/local.ini ±10 km, krok 5 m, ~20 MB
do `data/terrain.npz`):

```bash
./.venv/bin/python -m cockpitview.terrain_fetch
./.venv/bin/python -m cockpitview.terrain_fetch --lat 50.0 --lon 15.0 --radius-km 10
```

Skript si data vyžádá z veřejného ImageServeru ČÚZK rovnou ve WGS84 mřížce
a po stažení ji ověří proti bodovým dotazům na server. Za letu pak
`AGL = AMSL (baro) − elevace(lat, lon)` s bilineární interpolací; pozice
z `GLOBAL_POSITION_INT`, výška preferenčně z `ALTITUDE.altitude_amsl`.

Pozor: DMR 5G pokrývá jen ČR — nad slovenskou stranou hranice údaj zmizí.

### Reálný MPU-6050 (jen attitude)

Air ESP32 zatím posílá jen attitude (roll/pitch/yaw + úhlové rychlosti) —
HUD ukazuje horizont/roll/heading (z yaw), rychlost a výška zůstanou `---`
a místo flight path markeru je boresight kříž.

## Soubory

- `run.sh` — launcher (venv python, `python -m cockpitview`)
- `cockpitview/__main__.py` — main, pygame render loop
- `cockpitview/telemetry.py` — TelemetryHub: thread-safe snapshot pro HUD + MAVLink forward do QGC
- `cockpitview/serial_source.py` — čtení air ESP32 (CSV → ATTITUDE), autodetekce portu, reconnect
- `cockpitview/emulator_source.py` — syntetická telemetrie z letového profilu (`--emulator`)
- `cockpitview/publisher.py` — telemetrický rámec → MAVLink; sdílí ho živá linka i replay
- `cockpitview/replay_source.py` — přehrání `.cvlog` (`--replay`), logový čas pro kalibrátory
- `cockpitview/flight_profile.py` — skriptovaný letový profil (přímý let, zatáčky, loop)
- `cockpitview/quaternion.py` — kvaternionové výpočty orientace pro emulátor
- `cockpitview/hud.py` — F-16 HUD renderer
- `cockpitview/camera.py` — USB kamera přes OpenCV/AVFoundation (vlákno, poslední frame)
- `cockpitview/terrain.py` — elevace terénu z mřížky, bilineární lookup
- `cockpitview/terrain_fetch.py` — jednorázové stažení mřížky z ČÚZK DMR 5G
- `data/` — stažené mřížky (`terrain.npz`)
