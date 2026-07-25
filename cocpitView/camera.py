"""USB camera capture, locked to one specific device.

The FPV grabber ("AV TO USB2.0") is selected by its AVFoundation device
name, not by a bare OpenCV index: indexes shift when a camera is unplugged,
which would silently switch the view to the Mac's built-in camera mid-flight.

OpenCV can only open by index and its index order does not match the
AVFoundation discovery order, so the mapping is resolved by probing: the
device's native (max format) resolution is read via AVFoundation and the
cv2 index that reports the same default resolution is the one (verified
unique among connected cameras). While the target device is unplugged no
other camera is ever opened - the view shows NO VIDEO until it comes back.

A background thread always keeps only the latest frame - no queue - so
display latency stays minimal.
"""

import sys
import threading

import cv2
import numpy as np
import pygame

REOPEN_PERIOD_S = 3.0

try:
    import AVFoundation as AVF
    import CoreMedia as CM
except ImportError:  # non-macOS or pyobjc missing: index-only mode
    AVF = None


def discover_devices():
    """[(name, (max_w, max_h))] of connected cameras, or None without pyobjc."""
    if AVF is None:
        return None
    type_names = (
        "AVCaptureDeviceTypeBuiltInWideAngleCamera",
        "AVCaptureDeviceTypeExternal",
        "AVCaptureDeviceTypeExternalUnknown",
    )
    types = [getattr(AVF, n) for n in type_names if hasattr(AVF, n)]
    session = AVF.AVCaptureDeviceDiscoverySession.discoverySessionWithDeviceTypes_mediaType_position_(
        types, AVF.AVMediaTypeVideo, AVF.AVCaptureDevicePositionUnspecified
    )
    devices = []
    for dev in session.devices():
        dims = []
        for f in dev.formats():
            d = CM.CMVideoFormatDescriptionGetDimensions(f.formatDescription())
            dims.append((d.width, d.height))
        devices.append((str(dev.localizedName()), max(dims) if dims else (0, 0)))
    return devices


class Camera:
    def __init__(self, name="AV TO USB2.0", index=None):
        """`index` forces a cv2 device index and skips the by-name lookup."""
        self._name = name
        self._forced_index = index
        self._lock = threading.Lock()
        self._frame = None  # latest RGB frame (numpy, HxWx3)
        self._ok = False
        self._status = "starting"
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _open_index(self, index):
        if sys.platform == "darwin":
            cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
        else:
            cap = cv2.VideoCapture(index)
        return cap if cap.isOpened() else None

    def _open_target(self):
        """Open the configured camera, or (None, reason) when unavailable."""
        if self._forced_index is not None:
            cap = self._open_index(self._forced_index)
            return (cap, None) if cap else (None, f"camera index {self._forced_index} not available")

        devices = discover_devices()
        if devices is None:
            return None, "pyobjc not available - use --camera <index>"
        present = [d for d in devices if d[0] == self._name]
        if not present:
            return None, f"waiting for {self._name}"
        expected = present[0][1]
        if [dims for name, dims in devices if name != self._name].count(expected):
            return None, f"another camera has the same native resolution {expected} - use --camera <index>"

        for i in range(len(devices)):
            cap = self._open_index(i)
            if cap is None:
                continue
            dims = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            if dims == expected and cap.read()[0]:
                print(f"Camera '{self._name}' opened at cv2 index {i} ({dims[0]}x{dims[1]})")
                return cap, None
            cap.release()
        return None, f"{self._name} listed but no cv2 index matches {expected}"

    def _run(self):
        cap = None
        while not self._stop.is_set():
            if cap is None:
                cap, reason = self._open_target()
                if cap is None:
                    self._ok = False
                    self._status = reason
                    self._stop.wait(REOPEN_PERIOD_S)
                    continue
            ret, frame = cap.read()
            if not ret:
                cap.release()
                cap = None
                self._ok = False
                self._status = f"signal lost, waiting for {self._name}"
                continue
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            with self._lock:
                self._frame = rgb
                self._ok = True
        if cap is not None:
            cap.release()

    def get_surface(self, size):
        """Latest frame aspect-fitted into a pygame surface of `size`."""
        with self._lock:
            frame = self._frame if self._ok else None
        status = self._status

        surface = pygame.Surface(size)
        if frame is None:
            surface.fill((28, 30, 32))
            font = pygame.font.SysFont("Menlo", max(16, size[1] // 24))
            text = font.render("NO VIDEO", True, (110, 110, 110))
            surface.blit(text, text.get_rect(center=(size[0] // 2, size[1] // 2)))
            small = pygame.font.SysFont("Menlo", max(12, size[1] // 40))
            detail = small.render(status, True, (80, 80, 80))
            surface.blit(detail, detail.get_rect(center=(size[0] // 2, size[1] // 2 + size[1] // 16)))
            return surface

        h, w = frame.shape[:2]
        scale = min(size[0] / w, size[1] / h)
        target = (max(1, int(w * scale)), max(1, int(h * scale)))
        # pygame.surfarray expects (width, height) ordering, hence the swap
        frame_surface = pygame.surfarray.make_surface(np.swapaxes(frame, 0, 1))
        frame_surface = pygame.transform.smoothscale(frame_surface, target)
        surface.fill((0, 0, 0))
        surface.blit(frame_surface, frame_surface.get_rect(center=(size[0] // 2, size[1] // 2)))
        return surface
