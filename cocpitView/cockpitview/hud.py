"""F-16 style HUD renderer (NAV mode).

Layout follows the classic F-16 HUD diagram (hudgor.jpg reference):
airspeed tape on the left, altitude tape on the right, magnetic heading tape
at the top, pitch ladder + horizon line in the center, flight path marker,
and a roll indicator arc at the bottom. Metric units: km/h and meters.

Every primitive is drawn twice - a black halo underneath, HUD green on
top - so the symbology stays readable over bright video (white walls, sky).

All geometry is derived from the surface size; the pitch ladder scale is
tied to the camera's vertical field of view so the drawn horizon matches
the real horizon in the video.
"""

import math

import pygame

GREEN = (0, 255, 65)
BLACK = (0, 0, 0)
HALO = 1  # extra half-width of the black underlay


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


class Hud:
    def __init__(self, size, fov_deg=40.0):
        self.w, self.h = size
        self.cx, self.cy = self.w // 2, self.h // 2
        self.px_per_deg = self.h / fov_deg

        base = max(13, int(self.h * 0.028))
        self.font = pygame.font.SysFont("Menlo", base, bold=True)
        self.font_big = pygame.font.SysFont("Menlo", int(base * 1.25), bold=True)
        self.lw = max(2, round(self.h / 260))
        self.blink = 0.0

    # ------------------------------------------------------------- helpers

    def _line(self, surface, p1, p2):
        pygame.draw.line(surface, BLACK, p1, p2, self.lw + 2 * HALO)
        pygame.draw.line(surface, GREEN, p1, p2, self.lw)

    def _polygon(self, surface, points, filled=True):
        pygame.draw.polygon(surface, BLACK, points, self.lw + 2 * HALO)  # halo along edges
        if filled:
            pygame.draw.polygon(surface, GREEN, points)
        else:
            pygame.draw.polygon(surface, GREEN, points, self.lw)

    def _circle(self, surface, center, radius):
        pygame.draw.circle(surface, BLACK, center, radius + HALO, self.lw + 2 * HALO)
        pygame.draw.circle(surface, GREEN, center, radius, self.lw)

    def _text(self, surface, text, pos, font=None, anchor="center", angle=0.0):
        font = font or self.font
        halo = font.render(text, True, BLACK)
        img = font.render(text, True, GREEN)
        if angle:
            halo = pygame.transform.rotate(halo, angle)
            img = pygame.transform.rotate(img, angle)
        rect = img.get_rect(**{anchor: pos})
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, -1), (-1, 1), (1, 1)):
            surface.blit(halo, rect.move(dx, dy))
        surface.blit(img, rect)
        return rect

    def _dashed_line(self, surface, p1, p2, dash=10, gap=7):
        x1, y1 = p1
        x2, y2 = p2
        length = math.hypot(x2 - x1, y2 - y1)
        if length < 1:
            return
        ux, uy = (x2 - x1) / length, (y2 - y1) / length
        pos = 0.0
        while pos < length:
            end = min(pos + dash, length)
            self._line(surface, (x1 + ux * pos, y1 + uy * pos), (x1 + ux * end, y1 + uy * end))
            pos = end + gap

    # ------------------------------------------------------ pitch ladder

    def _ladder_point(self, x, y, roll):
        """Transform ladder-space (x along horizon, y toward ground) to screen."""
        s, c = math.sin(roll), math.cos(roll)
        return (self.cx + x * c + y * s, self.cy - x * s + y * c)

    def draw_pitch_ladder(self, surface, roll, pitch):
        pitch_deg = math.degrees(pitch)
        ppd = self.px_per_deg
        gap = self.w * 0.085
        rung = self.w * 0.10
        hook = self.h * 0.018
        horizon_len = self.w * 0.16

        clip = surface.get_clip()
        surface.set_clip(
            pygame.Rect(int(self.w * 0.24), int(self.h * 0.12), int(self.w * 0.52), int(self.h * 0.76))
        )

        for theta in range(-90, 95, 5):
            dy = (pitch_deg - theta) * ppd
            if abs(dy) > self.h * 0.6:
                continue
            if theta == 0:
                for side in (-1, 1):
                    p1 = self._ladder_point(side * gap, dy, roll)
                    p2 = self._ladder_point(side * (gap + horizon_len), dy, roll)
                    self._line(surface, p1, p2)
                continue

            hook_dir = 1 if theta > 0 else -1  # hooks bend toward the horizon
            for side in (-1, 1):
                inner = self._ladder_point(side * gap, dy, roll)
                outer = self._ladder_point(side * (gap + rung), dy, roll)
                if theta > 0:
                    self._line(surface, inner, outer)
                else:
                    self._dashed_line(surface, inner, outer)
                hook_end = self._ladder_point(side * (gap + rung), dy + hook_dir * hook, roll)
                self._line(surface, outer, hook_end)

                label_pos = self._ladder_point(side * (gap + rung + self.w * 0.02), dy, roll)
                self._text(surface, str(abs(theta)), label_pos, angle=math.degrees(roll))

        surface.set_clip(clip)

    # -------------------------------------------------- flight path marker

    def draw_flight_path_marker(self, surface, snap):
        vel = snap.velocity_ned
        if vel is None or not snap.has_attitude:
            # attitude-only source (HW bridge): boresight cross instead of FPM
            r = self.h * 0.012
            self._line(surface, (self.cx - r, self.cy), (self.cx + r, self.cy))
            self._line(surface, (self.cx, self.cy - r), (self.cx, self.cy + r))
            return

        vn, ve, vd = vel
        vh = math.hypot(vn, ve)
        if vh < 0.5 and abs(vd) < 0.5:
            gamma_deg, drift_deg = 0.0, 0.0
        else:
            gamma_deg = math.degrees(math.atan2(-vd, max(vh, 0.1)))
            track_deg = math.degrees(math.atan2(ve, vn))
            heading = snap.heading_deg if snap.heading_deg is not None else track_deg
            drift_deg = wrap180(track_deg - heading)

        pitch_deg = math.degrees(snap.pitch)
        x = drift_deg * self.px_per_deg
        y = (pitch_deg - gamma_deg) * self.px_per_deg
        px, py = self._ladder_point(x, y, snap.roll)
        px = min(max(px, self.w * 0.28), self.w * 0.72)
        py = min(max(py, self.h * 0.16), self.h * 0.82)

        r = self.h * 0.014
        self._circle(surface, (px, py), r)
        self._line(surface, (px - r, py), (px - 2.6 * r, py))  # wings
        self._line(surface, (px + r, py), (px + 2.6 * r, py))
        self._line(surface, (px, py - r), (px, py - 2.2 * r))  # tail

    # ------------------------------------------------------- heading tape

    def draw_heading_tape(self, surface, heading):
        tape_w = self.w * 0.42
        y = self.h * 0.075
        left = self.cx - tape_w / 2
        span = 30.0  # degrees visible
        ppd = tape_w / span
        tick_h = self.h * 0.014

        if heading is None:
            self._boxed_value(surface, "---", (self.cx, y - self.h * 0.028))
            return

        clip = surface.get_clip()
        surface.set_clip(pygame.Rect(int(left), 0, int(tape_w), int(self.h * 0.13)))

        first = int(math.floor((heading - span / 2) / 5.0)) * 5
        for t in range(first, first + int(span) + 10, 5):
            x = self.cx + (t - heading) * ppd
            major = t % 10 == 0
            hgt = tick_h * (1.6 if major else 1.0)
            self._line(surface, (x, y), (x, y + hgt))
            if major:
                label = f"{(t % 360) // 10:02d}"
                self._text(surface, label, (x, y + tick_h * 1.6 + self.h * 0.014))

        surface.set_clip(clip)

        # caret above the tape + boxed current heading
        self._polygon(surface, [(self.cx, y), (self.cx - 7, y - 11), (self.cx + 7, y - 11)])
        self._boxed_value(surface, f"{int(round(heading)) % 360:03d}", (self.cx, y - self.h * 0.034))

    # ------------------------------------------------- airspeed / altitude

    def _boxed_value(self, surface, text, pos):
        """Value in a green box over a solid black background."""
        img = self.font_big.render(text, True, GREEN)
        rect = img.get_rect(center=pos).inflate(12, 6)
        pygame.draw.rect(surface, BLACK, rect.inflate(2 * HALO, 2 * HALO))
        pygame.draw.rect(surface, GREEN, rect, self.lw)
        surface.blit(img, img.get_rect(center=rect.center))
        return rect

    def _draw_vertical_tape(self, surface, x_axis, value, side, unit_span, tick_step, label_step):
        """side=-1: airspeed (left, box left of scale); side=+1: altitude (right)."""
        tape_h = self.h * 0.5
        top = self.cy - tape_h / 2
        ppu = tape_h / unit_span
        tick_len = self.w * 0.012

        clip = surface.get_clip()
        surface.set_clip(pygame.Rect(0 if side < 0 else self.cx, int(top) - 14, self.w // 2, int(tape_h) + 28))

        first = int(math.floor((value - unit_span / 2) / tick_step)) * tick_step
        for v in range(first, first + int(unit_span) + 2 * tick_step, tick_step):
            if v < 0:
                continue
            y = self.cy - (v - value) * ppu
            major = v % label_step == 0
            length = tick_len * (1.7 if major else 1.0)
            # ticks extend outward (away from the screen center), labels outside them
            self._line(surface, (x_axis, y), (x_axis + side * length, y))
            if major:
                anchor = "midright" if side < 0 else "midleft"
                self._text(surface, str(v), (x_axis + side * (tick_len * 1.7 + 6), y), anchor=anchor)

        surface.set_clip(clip)

    def _value_box(self, surface, x_axis, value_text, side):
        box_offset = self.w * 0.105
        rect = self._boxed_value(surface, value_text, (x_axis + side * box_offset, self.cy))
        # caret pointing from the box toward the scale
        tip_x = x_axis + side * self.w * 0.012
        base_x = rect.right if side < 0 else rect.left
        self._polygon(surface, [(tip_x, self.cy), (base_x, self.cy - 8), (base_x, self.cy + 8)])

    def draw_airspeed(self, surface, kmh):
        x_axis = self.w * 0.17
        if kmh is not None:
            self._draw_vertical_tape(surface, x_axis, kmh, side=-1, unit_span=60, tick_step=5, label_step=20)
            self._value_box(surface, x_axis, f"{int(round(kmh)):3d}", side=-1)
        else:
            self._value_box(surface, x_axis, "---", side=-1)

    def draw_altitude(self, surface, alt_m, agl_m=None, climb_mps=None):
        x_axis = self.w * 0.83
        if alt_m is not None:
            self._draw_vertical_tape(surface, x_axis, alt_m, side=1, unit_span=100, tick_step=10, label_step=50)
            self._value_box(surface, x_axis, f"{int(round(alt_m)):4d}", side=1)
        else:
            self._value_box(surface, x_axis, "---", side=1)
        if agl_m is not None:
            # height above terrain, F-16 radar-altitude style ("R" prefix);
            # negative values are shown as-is - they expose a QNH/terrain error
            self._text(
                surface,
                f"R {int(round(agl_m)):4d}",
                (x_axis + self.w * 0.105, self.cy + self.h * 0.065),
                anchor="center",
            )
        if climb_mps is not None:
            # vario in m/s (unitless on purpose), under the radar altitude
            self._text(
                surface,
                f"{climb_mps:+.1f}",
                (x_axis + self.w * 0.105, self.cy + self.h * 0.105),
                anchor="center",
            )

    # --------------------------------------------------------- GPS status

    def draw_gps_status(self, surface, gps):
        """Fix quality + satellite count + 1-sigma horizontal error, e.g.
        "GPS 3D 12 ±4M", in the very bottom-left screen corner (outside the
        HUD symbology); hidden when the source sends no GPS_RAW_INT at all."""
        if gps is None:
            return
        fix_type, sats, hacc_m = gps
        # MAVLink GPS_FIX_TYPE: 2 = 2D, 3 = 3D, 4+ = differential/RTK
        label = {2: "2D", 3: "3D"}.get(fix_type, "DGPS" if fix_type > 3 else "--")
        text = f"GPS {label} {sats:d}"
        if hacc_m is not None:
            text += f" ±{max(1, math.ceil(hacc_m)):d}M"
        self._text(surface, text, (8, self.h - 6), anchor="bottomleft")

    # ------------------------------------------------------ roll indicator

    def draw_roll_indicator(self, surface, roll):
        radius = self.h * 0.36
        tick_out = self.h * 0.02

        for a_deg in (-45, -30, -20, -10, 0, 10, 20, 30, 45):
            a = math.radians(a_deg)
            dx, dy = math.sin(a), math.cos(a)
            r1 = radius
            r2 = radius + tick_out * (1.6 if a_deg in (0, -45, 45) else 1.0)
            p1 = (self.cx + dx * r1, self.cy + dy * r1)
            p2 = (self.cx + dx * r2, self.cy + dy * r2)
            self._line(surface, p1, p2)

        # pointer triangle tracks the world-down direction (same sense as horizon),
        # sits below the scale and points up at it
        a = max(-math.radians(50), min(math.radians(50), roll))
        dx, dy = math.sin(a), math.cos(a)
        offset = radius + self.h * 0.016
        tx, ty = self.cx + dx * offset, self.cy + dy * offset
        px, py = -dy, dx  # perpendicular
        size = self.h * 0.014
        self._polygon(
            surface,
            [
                (tx, ty),
                (tx + dx * 2 * size + px * size, ty + dy * 2 * size + py * size),
                (tx + dx * 2 * size - px * size, ty + dy * 2 * size - py * size),
            ],
            filled=False,
        )

    # --------------------------------------------------------------- draw

    def draw(self, surface, snap, dt):
        self.blink += dt

        if not snap.connected:
            if int(self.blink * 2) % 2 == 0:
                self._text(
                    surface,
                    "NO TELEMETRY",
                    (self.cx, self.cy - self.h * 0.10),
                    font=self.font_big,
                )

        if snap.has_attitude:
            self.draw_pitch_ladder(surface, snap.roll, snap.pitch)
            self.draw_roll_indicator(surface, snap.roll)
        self.draw_flight_path_marker(surface, snap)
        self.draw_heading_tape(surface, snap.heading_deg)
        self.draw_airspeed(surface, snap.airspeed_kmh)
        self.draw_altitude(surface, snap.altitude_m, snap.agl_m, snap.vario_mps)
        self._text(surface, "NAV", (self.w * 0.11, self.h * 0.88), anchor="midleft")
        self.draw_gps_status(surface, snap.gps_status)
