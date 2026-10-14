"""Leader joint readings from miniarm_v2.py's OPC UA server, unchanged.

miniarm_v2.py runs on the Pi, reads the mini-arm's encoders, and serves the
calibrated angles over OPC UA (port 4840) as ns=5;s=Rc_Joint1..7 (the
005-MiniArm object; 006-KukaArm carries the same values). This connects as
just another client, like fri/opcua_reader.py does, so the Pi side needs no
changes.

What those numbers are, and how they're converted here:

  * Degrees -> converted to radians.
  * KUKA joint convention: miniarm_v2's per-joint JOINT_DIR is chosen so the
    values can drive the KUKA directly (the FRI app feeds them straight to
    it). The leader model turns some joints the other way, so by default
    they're multiplied by scene.JOINT_SIGNS to get back to the model's
    convention. Pass convention="model" if your readings already match it.
  * Zero = the midpoint between the two mechanical stops marked during
    calibration, and the stops map to +/- miniarm_v2's JOINT_RANGE_DEG. So
    these are true joint angles only if each joint's stops really sit at
    those angles, symmetric about straight up.
"""

import asyncio
import logging
import threading

import numpy as np

from leader import NUM_JOINTS, JointSource
from scene import JOINT_SIGNS

log = logging.getLogger("opcua_source")
# asyncua logs every single read at INFO -- 50 lines a second.
logging.getLogger("asyncua").setLevel(logging.WARNING)

# The mini-arm's own namespace in miniarm_v2.py's build_server().
NODE_IDS = [f"ns=5;s=Rc_Joint{i + 1}" for i in range(NUM_JOINTS)]

CONVENTIONS = ("kuka", "model")


class OpcUaJointSource(JointSource):
    """Polls the joint nodes on its own thread with its own asyncio loop.

    Never raises into the caller: if the Pi isn't reachable it keeps
    retrying, and latest() returns None while disconnected, so the KUKA
    holds its pose rather than chasing a stale reading.
    """

    def __init__(self, url, convention="kuka", poll_hz=50.0):
        if convention not in CONVENTIONS:
            raise ValueError(f"convention must be one of {CONVENTIONS}, got {convention!r}")
        self.url = url
        self._signs = JOINT_SIGNS if convention == "kuka" else np.ones(NUM_JOINTS)
        self._period = 1.0 / poll_hz
        self._lock = threading.Lock()
        self._joints = None
        self._connected = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @property
    def connected(self):
        return self._connected

    def latest(self):
        with self._lock:
            return None if self._joints is None else self._joints.copy()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=3.0)

    def _set(self, joints):
        with self._lock:
            self._joints = joints

    def _run(self):
        asyncio.run(self._loop())

    async def _loop(self):
        from asyncua import Client  # imported here so the rest runs without it

        backoff = 1.0
        while not self._stop.is_set():
            try:
                log.info("connecting to %s", self.url)
                async with Client(url=self.url, timeout=4) as client:
                    # Resolve the nodes once; then one batched read per poll
                    # (one round trip for all 7 joints, not seven).
                    nodes = [client.get_node(node_id) for node_id in NODE_IDS]
                    self._connected = True
                    backoff = 1.0
                    log.info("connected, polling at %.0f Hz", 1.0 / self._period)

                    while not self._stop.is_set():
                        degrees = await client.read_values(nodes)
                        self._set(self._signs * np.radians(np.asarray(degrees, dtype=float)))
                        await asyncio.sleep(self._period)
            except Exception as exc:  # noqa: BLE001 - this is a reconnect loop
                log.warning("OPC UA error (%s); retrying in %.0fs", exc, backoff)
            finally:
                self._connected = False
                self._set(None)
            # Sleep in small steps so stop() isn't held up by the backoff.
            for _ in range(int(backoff / 0.1)):
                if self._stop.is_set():
                    break
                await asyncio.sleep(0.1)
            backoff = min(backoff * 2.0, 10.0)
