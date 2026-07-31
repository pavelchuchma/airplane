# Connecting a camera to QGroundControl (macOS)

Reference notes for getting a video input (FPV / capture card) working in QGC with HUD and telemetry.
Environment: **macOS 26.5.2 (Apple Silicon)**, QGroundControl.

---

## TL;DR

The native local (UVC) camera does **not** work in QGC on macOS 26 (see why below).
The working path is to send the camera into QGC as a **UDP h.264 stream** via GStreamer.

```bash
./qgc-camera-stream.sh          # finds camera by name, detects resolution, streams
```
In QGC: **Video → Source = UDP h.264 Video Stream**, **URL = 0.0.0.0:5600**, enable **Low Latency Mode**.

---

## Why the native UVC camera does not work

Three QGC builds were tested:

| Build | Local UVC camera in "Source"? | Note |
|-------|-------------------------------|------|
| Stable v5.0 (`org.qgroundcontrol.QGroundControl`) | ❌ no | UVC disabled at compile time |
| Stable v4.4.5 (`org.mavlink.qgroundcontrol`) | ❌ no | older build, not notarized |
| Daily / master (`org.mavlink.qgroundcontrol`) | ✅ yes | but "Failed to get camera permission" |

The daily build does support UVC (added in [PR #11095](https://github.com/mavlink/qgroundcontrol/pull/11095)),
but it requests the permission through Qt, and that path is broken on macOS 26: the system
dialog never appears and QGC is **never registered** under System Settings → Privacy → Camera.
The author of the PR notes the path is flaky himself ("requires QGC reboot… maybe a Qt bug").

Confirmed the cause is **not** on our side:
- app signature is valid and notarized (`codesign`, `spctl` → accepted),
- `Info.plist` contains `NSCameraUsageDescription` ("Camera devices used for FPV"),
- TCC reset (`tccutil reset Camera org.mavlink.qgroundcontrol`) completed cleanly.

### Camera permission cannot be granted manually
macOS treats camera and microphone as "request-only":
- there is **no "+" button** under Settings → Camera,
- `tccutil` can only reset, not grant,
- even an MDM/PPPC profile cannot pre-approve the camera (only deny).

The only "hard" workaround would be disabling SIP and manually writing into `TCC.db` —
not recommended (permanent security downgrade). → We stick with streaming.

---

## Working solution: GStreamer UDP stream

### Prerequisites
GStreamer (via Homebrew):
```bash
brew install gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly gst-libav
```

### Script `qgc-camera-stream.sh`
Finds the camera by name, auto-detects a supported resolution/fps, and starts the stream
(hardware encoder `vtenc_h264`, `sync=false` for low latency).

```bash
chmod +x qgc-camera-stream.sh

./qgc-camera-stream.sh --list          # list cameras and their indexes/caps
./qgc-camera-stream.sh                  # stream the default camera (CAMERA_NAME)
./qgc-camera-stream.sh "AV TO USB2.0"   # stream a camera by name
```

Override via environment variables:
```bash
BITRATE=1500 FPS=20 ./qgc-camera-stream.sh
WIDTH=640 HEIGHT=480 PORT=5600 ./qgc-camera-stream.sh
```

The default searched camera is `AV TO USB2.0` (change via `CAMERA_NAME` or the 1st argument).

### QGC settings
1. Application Settings → **Video**
2. **Source** = `UDP h.264 Video Stream`
3. **UDP URL** = `0.0.0.0:5600`
4. Enable **Low Latency Mode** (shortens QGC's jitter buffer)

### Manual command (without the script)
```bash
gst-launch-1.0 avfvideosrc device-index=2 do-timestamp=true ! \
  video/x-raw,width=720,height=480,framerate=25/1 ! videoconvert ! \
  vtenc_h264 realtime=true allow-frame-reordering=false max-keyframe-interval=15 bitrate=2000 ! \
  h264parse config-interval=1 ! rtph264pay config-interval=1 pt=96 mtu=1400 ! \
  udpsink host=127.0.0.1 port=5600 sync=false
```

---

## Specific hardware: "AV TO USB2.0" grabber

HDMI/AV → USB capture card, **MACROSILICON** chip (`UVC Camera VendorID_21325 ProductID_33`).
Supported resolutions (SD):

- 720x480, 640x480 → 10/15/20/25 fps
- 480x320 → 25 fps

It does **not** support 1280x720 or 30 fps → forcing higher values fails with
`not-negotiated (-4)`. The script auto-detects the resolution to avoid this.

---

## Latency

- Native preview (QuickTime / "Quick Camera") = ~0 s, but a separate window without HUD.
- Streaming into QGC = encode + RTP + decode → practical floor **~0.15 s**.
- Helps: `Low Latency Mode` in QGC, `vtenc_h264`, `sync=false`, lower resolution/bitrate.
- Reaching 0 s inside QGC would only be possible with a QGC build from source with working
  UVC (native display without encoding) — not pursued yet.

---

## Troubleshooting

- **`not-negotiated (-4)`** → requested resolution/fps not supported by the camera. Run
  `--list` and set `WIDTH/HEIGHT/FPS` to a value from the output.
- **`gst-device-monitor` spews warnings** (GObject introspection, duplicate GTK) →
  harmless noise on stderr; the script silences it with `2>/dev/null`.
- **Wrong camera** → `avfvideosrc device-index` matches the order in
  `gst-device-monitor Video/Source` (so list with the same tool, not ffmpeg).
- **Verify the camera outside QGC** (also triggers the macOS camera dialog for Terminal):
  ```bash
  gst-launch-1.0 avfvideosrc device-index=2 ! videoconvert ! autovideosink
  ```

---

## Open / to revisit next time

- Consider building QGC (master) from source for native UVC without latency.
- Optionally file a GitHub issue: native UVC camera on macOS 26 — permission dialog never
  appears, QGC does not show up under Privacy → Camera (signature/plist/TCC all OK). No
  issue covers exactly this yet.

## Links
- Download & install (v5.0): https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html
- Video guide (v5.0): https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/fly_view/video.html
- PR #11095 (UVC video fixes): https://github.com/mavlink/qgroundcontrol/pull/11095
- Releases: https://github.com/mavlink/qgroundcontrol/releases
