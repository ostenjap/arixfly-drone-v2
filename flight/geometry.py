"""World frame, rotations and the pinhole camera model shared by every v2 layer.

World frame (cm): origin = centre of table A's top, x = toward table B, y = LEFT, z = UP (right-handed).
Heading psi (rad): 0 = nose toward B (+x), + = turned LEFT (counter-clockwise seen from above).
Sticks keep the old meaning (power units, see flight_control.to_bytes): pitch + = forward, roll + = right,
yaw + = turn right, throttle + = up. So roll + moves toward -y (body), yaw + makes psi smaller.
Camera frames use the OpenCV convention: x right, y down, z along the optical axis.
"""
import math

import numpy as np


def wrap(a):
    """Angle to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def rot2(psi):
    """Body (forward, left) -> world (x, y)."""
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s], [s, c]])


def look_rotation(pan, tilt_up, roll=0.0):
    """Camera -> world rotation (columns = camera x, y, z axes in world) for a camera whose optical axis points at
    heading `pan` (rad, + = left) and `tilt_up` (rad, + = up)."""
    ct, st = math.cos(tilt_up), math.sin(tilt_up)
    z = np.array([ct * math.cos(pan), ct * math.sin(pan), st])
    x = np.array([math.sin(pan), -math.cos(pan), 0.0])       # image right = horizontal, right of the view
    y = np.cross(z, x)                                        # image down
    if roll:
        c, s = math.cos(roll), math.sin(roll)
        x, y = c * x + s * y, -s * x + c * y
    return np.column_stack([x, y, z])


class Camera:
    """Pinhole camera at world position `pos` with camera->world rotation `R` (no lens distortion)."""

    def __init__(self, f, w, h, pos, R, cx=None, cy=None):
        self.f, self.w, self.h = float(f), int(w), int(h)
        self.cx = w / 2 if cx is None else cx
        self.cy = h / 2 if cy is None else cy
        self.pos, self.R = np.asarray(pos, float), np.asarray(R, float)

    @classmethod
    def from_pose(cls, f, w, h, pos, pan_deg=0.0, tilt_deg=0.0, roll_deg=0.0):
        return cls(f, w, h, pos, look_rotation(math.radians(pan_deg), math.radians(tilt_deg), math.radians(roll_deg)))

    def scaled(self, w):
        """Same camera, image resized to width w."""
        k = w / self.w
        return Camera(self.f * k, w, round(self.h * k), self.pos, self.R, self.cx * k, self.cy * k)

    def to_cam(self, p):
        return self.R.T @ (np.asarray(p, float) - self.pos)

    def project(self, p):
        """World point -> (u, v, depth) or None when behind the camera."""
        q = self.to_cam(p)
        if q[2] < 1.0:
            return None
        return self.cx + self.f * q[0] / q[2], self.cy + self.f * q[1] / q[2], q[2]

    def ray(self, u, v):
        """Pixel -> unit direction in world."""
        d = self.R @ np.array([(u - self.cx) / self.f, (v - self.cy) / self.f, 1.0])
        return d / np.linalg.norm(d)

    def inside(self, u, v, margin=0.0):
        return margin * self.w <= u <= (1 - margin) * self.w and margin * self.h <= v <= (1 - margin) * self.h


def focal_from_hfov(width, hfov_deg):
    return (width / 2) / math.tan(math.radians(hfov_deg) / 2)


def camera_from_tag(tag_cam_xyz, tag_cam_R, tag_world_pos, tag_world_R):
    """Camera pose in world from one tag seen by it.
    tag_cam_*: tag pose in camera coordinates (solvePnP tvec, rotation tag->camera);
    tag_world_*: tag pose in world (position, rotation tag->world). Returns (camera position, camera->world R)."""
    R_wc = tag_world_R @ tag_cam_R.T
    pos = tag_world_pos - R_wc @ tag_cam_xyz
    return pos, R_wc


def tag_world_rotation(facing):
    """Upright tag whose printed face looks along world direction `facing` (unit-ish 3-vector, horizontal).
    Tag axes as OpenCV/aruco: x = right on the print, y = up on the print (IPPE_SQUARE object points
    (-s, s), (s, s), (s, -s), (-s, -s) = top-left, top-right, bottom-right, bottom-left), z = out of the print."""
    n = np.asarray(facing, float)
    n = n / np.linalg.norm(n)
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(up, n)          # right on the print as seen by someone looking at it
    x = x / np.linalg.norm(x)
    y = np.cross(n, x)           # up on the print
    return np.column_stack([x, y, n])
