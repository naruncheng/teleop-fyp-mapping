"""Where the leader's pose comes from.

Anything with a latest() that returns a pose tuple works:

    (position, rotation)
      position: shape (3,), metres, in the leader's own base frame
      rotation: shape (3, 3) rotation matrix, same frame

or None while no pose is available yet. To plug in a new leader arm,
subclass PoseSource and have latest() return its FK result.
"""

import json
import logging
import socket
import threading

import mujoco
import numpy as np

log = logging.getLogger("pose_sources")


def _rot(axis, angle):
    """Rotation matrix for `angle` radians about a unit `axis`."""
    quat = np.zeros(4)
    mujoco.mju_axisAngle2Quat(quat, np.asarray(axis, dtype=float), angle)
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, quat)
    return mat.reshape(3, 3)


class PoseSource:
    """Interface every pose source implements."""

    def latest(self):
        """Most recent (position, rotation), or None if nothing yet."""
        raise NotImplementedError

    def stop(self):
        """Release any threads/sockets. Safe to call more than once."""


class DemoPose(PoseSource):
    """A slow figure-eight with some tilt and roll around a centre pose,
    just to show the follower can be driven from code.

    size scales the motion; pass 1/scale so the KUKA moves the same amount
    whatever the follower's scale is.
    """

    def __init__(self, center_pos, center_rot, size=1.0, clock=None):
        import time
        self._clock = clock or time.time
        self._start = self._clock()
        self._center_pos = np.asarray(center_pos, dtype=float)
        self._center_rot = np.asarray(center_rot, dtype=float)
        self._size = size

    def latest(self):
        t = self._clock() - self._start
        position = self._center_pos + self._size * np.array(
            [0.0, 0.15 * np.sin(t), 0.08 * np.sin(2 * t)])
        # Rotations about the tool's own axes: tilt about its X, then roll
        # about its pointing axis (Z).
        rotation = (self._center_rot
                    @ _rot([1.0, 0.0, 0.0], 0.3 * np.sin(t))
                    @ _rot([0.0, 0.0, 1.0], 0.6 * np.sin(0.5 * t)))
        return position, rotation


class UdpJsonListener:
    """Receives JSON datagrams on its own thread and keeps the latest one,
    as turned into a value by parse(). Never raises into the caller:
    malformed packets are logged and dropped.

    For a leader that runs in another process, on another machine or in
    another language. Subclasses define parse().
    """

    def __init__(self, port, host="0.0.0.0"):
        self.port = port
        self._lock = threading.Lock()
        self._value = None
        self._stop = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.settimeout(0.5)  # so stop() is noticed promptly
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def latest(self):
        with self._lock:
            return self._value

    def stop(self):
        if self._stop.is_set():
            return
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._sock.close()

    def _run(self):
        while not self._stop.is_set():
            try:
                message, addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break  # socket closed
            try:
                value = self.parse(message)
            except (ValueError, KeyError, TypeError) as exc:
                log.warning("dropped malformed packet from %s: %s", addr, exc)
                continue
            with self._lock:
                self._value = value

    @staticmethod
    def parse(message):
        """JSON bytes/str -> the value latest() returns."""
        raise NotImplementedError


class UdpPoseSource(UdpJsonListener, PoseSource):
    """Poses sent as JSON over UDP, one per datagram:

        {"position": [x, y, z], "rotation": [[...], [...], [...]]}

    or with "quaternion": [w, x, y, z] in place of "rotation".
    """

    @staticmethod
    def parse(message):
        """JSON bytes/str -> (position (3,), rotation (3, 3))."""
        msg = json.loads(message)
        position = np.asarray(msg["position"], dtype=float)
        if position.shape != (3,):
            raise ValueError(f"position must have 3 values, got shape {position.shape}")

        if "rotation" in msg:
            rotation = np.asarray(msg["rotation"], dtype=float)
            if rotation.shape != (3, 3):
                raise ValueError(f"rotation must be 3x3, got shape {rotation.shape}")
        elif "quaternion" in msg:
            quat = np.asarray(msg["quaternion"], dtype=float)
            norm = np.linalg.norm(quat) if quat.shape == (4,) else 0.0
            if norm == 0.0:
                raise ValueError("quaternion must be 4 values [w, x, y, z], not all zero")
            mat = np.zeros(9)
            mujoco.mju_quat2Mat(mat, quat / norm)
            rotation = mat.reshape(3, 3)
        else:
            raise KeyError("need 'rotation' or 'quaternion'")

        return position, rotation
