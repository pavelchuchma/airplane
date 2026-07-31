#!/usr/bin/env bash
# QGC low-latency camera -> UDP h.264 stream (macOS)
#
# Finds a camera BY NAME in the GStreamer device list, auto-detects a supported
# resolution/framerate, and streams it to QGroundControl.
#
# Usage:
#   ./qgc-camera-stream.sh --list            # list available cameras
#   ./qgc-camera-stream.sh                    # stream default camera (CAMERA_NAME)
#   ./qgc-camera-stream.sh "AV TO USB2.0"     # stream camera by name
#
# Override anything via env vars, e.g.:
#   WIDTH=640 HEIGHT=480 FPS=20 PORT=5600 ./qgc-camera-stream.sh
#
# In QGC: Video -> Source = "UDP h.264 Video Stream", URL = 0.0.0.0:5600
# Enable "Low Latency Mode" in QGC Video settings for lowest delay.

set -euo pipefail

# --- config ---------------------------------------------------------------
CAMERA_NAME="${CAMERA_NAME:-AV TO USB2.0}"   # substring match on device name
PORT="${PORT:-5600}"
BITRATE="${BITRATE:-2000}"                    # kbps

list_cameras() {
  echo "Available video devices (same GStreamer backend used for streaming):"
  echo
  gst-device-monitor-1.0 Video/Source 2>/dev/null
}

# --- arg handling ---------------------------------------------------------
if [[ "${1:-}" == "--list" || "${1:-}" == "-l" ]]; then
  list_cameras
  exit 0
fi
if [[ -n "${1:-}" ]]; then
  CAMERA_NAME="$1"
fi

# --- find the device block by name ---------------------------------------
MON="$(gst-device-monitor-1.0 Video/Source 2>/dev/null)"

# Each device is one record when splitting on "Device found:".
# Keep the record that contains the wanted name AND is a Video/Source.
BLOCK="$(awk -v RS='Device found:' -v cam="$CAMERA_NAME" \
  'index($0, cam) && index($0, "Video/Source"){print; exit}' <<<"$MON")"

if [[ -z "$BLOCK" ]]; then
  echo "ERROR: camera matching \"$CAMERA_NAME\" not found." >&2
  echo "Run: $0 --list" >&2
  exit 1
fi

# device-index (as printed on the gst-launch hint line)
INDEX="$(grep -oE 'device-index=[0-9]+' <<<"$BLOCK" | head -1 | grep -oE '[0-9]+' || true)"
if [[ -z "$INDEX" ]]; then
  echo "ERROR: could not determine device-index for \"$CAMERA_NAME\"." >&2
  exit 1
fi

# --- auto-detect best supported resolution/framerate ----------------------
# Look at plain (non-GLMemory) "video/x-raw, width=..." caps lines, pick max width.
best_w=0; best_h=0; fr_line=""
while IFS= read -r line; do
  w="$(grep -oE 'width=[0-9]+'  <<<"$line" | grep -oE '[0-9]+' | head -1)"
  h="$(grep -oE 'height=[0-9]+' <<<"$line" | grep -oE '[0-9]+' | head -1)"
  [[ -z "${w:-}" || -z "${h:-}" ]] && continue
  if (( w > best_w )); then best_w="$w"; best_h="$h"; fr_line="$line"; fi
done < <(grep -E 'video/x-raw, width=' <<<"$BLOCK")

# max framerate (numerator of N/1) advertised on the chosen caps line
fps_det="$(grep -oE '[0-9]+/1' <<<"$fr_line" | cut -d/ -f1 | sort -n | tail -1 || true)"
[[ -z "${fps_det:-}" ]] && fps_det=30

# env overrides win over auto-detected values
WIDTH="${WIDTH:-$best_w}"
HEIGHT="${HEIGHT:-$best_h}"
FPS="${FPS:-$fps_det}"

# --- stream ---------------------------------------------------------------
echo "Camera : $CAMERA_NAME (device-index=$INDEX)"
echo "Format : ${WIDTH}x${HEIGHT}@${FPS}  ->  udp://127.0.0.1:$PORT  (${BITRATE} kbps)"
echo "In QGC : Video Source = UDP h.264 Video Stream, URL = 0.0.0.0:$PORT"
echo "Press Ctrl+C to stop."
echo

exec gst-launch-1.0 avfvideosrc device-index="$INDEX" do-timestamp=true ! \
  "video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" ! videoconvert ! \
  vtenc_h264 realtime=true allow-frame-reordering=false max-keyframe-interval=15 bitrate="$BITRATE" ! \
  h264parse config-interval=1 ! rtph264pay config-interval=1 pt=96 mtu=1400 ! \
  udpsink host=127.0.0.1 port="$PORT" sync=false
