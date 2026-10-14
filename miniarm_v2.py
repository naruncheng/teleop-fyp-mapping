#!/usr/bin/env python3
"""
miniarm.py -- mini-arm encoders -> calibrated joint angles -> OPC UA,
with a small web page for watching and calibrating them.

ONE process does everything:

    7x AMT222B-V encoders (SPI0) --.
    deadman button (GPIO16) -------+--> calibration --> OPC UA server (:4840)
    XIAO ESP32C3 trigger pot ------'                      ns=5  005-MiniArm
    (USB serial, P,raw,mv,ms)      '--> web page (:8080)  ns=6  006-KukaArm

This replaces dashboard_calibration.py + encoder_opcua_test.py +
KEP_OPCUA_Server1.py for the joint path.  Because it hosts the OPC UA server
itself, KEP_OPCUA_Server1.py must NOT be running at the same time -- they
would fight over port 4840.

The node ids it serves are byte-for-byte the ones the old server served
(ns=5;s=Rc_Joint1 .. ns=6;s=Rc_Joint7, ns=N;s=start), so the KUKA-side
application and opcua_viewer.py keep working unchanged.  The gripper adds one
new id per system, ns=5;s=Rc_GripperPos and ns=6;s=Rc_GripperPos, a Float in
0.0 .. 1.0 where 0 is the trigger released and 1 is fully squeezed.

Run:
    python3 miniarm.py            # on the Pi, real hardware
    python3 miniarm.py --demo     # anywhere, fake encoders + pot, to try the UI
then open http://<pi>:8080
"""

import argparse
import glob
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from opcua import Server, ua

# Hardware libraries are optional so --demo runs on a laptop.
try:
    import spidev
except ImportError:
    spidev = None
try:
    from gpiozero import Button, DigitalOutputDevice
except ImportError:
    Button = DigitalOutputDevice = None
try:
    import serial                        # pyserial, for the trigger pot
except ImportError:
    serial = None


# ============================================================= CONFIGURATION ==
# Everything you are likely to want to change lives in this block.

# KUKA joint ranges, in degrees.  Calibration maps the mini-arm's full travel
# onto +/- this number: the stop you mark "lower" becomes -RANGE, the stop you
# mark "upper" becomes +RANGE, and the midpoint between them becomes 0.
JOINT_RANGE_DEG = [170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0]

# Per-joint sign.  Normally leave these at 1 -- which stop you press
# "Set lower" at already decides the direction.  Flip one to -1 if a joint
# comes out backwards and you would rather not recalibrate it.
JOINT_DIR = [-1, -1, -1, 1, -1, -1, -1]

CS_PINS = [17, 27, 22, 5, 6, 23, 26]   # encoder chip-select GPIOs, joint order
DEADMAN_PIN = 16                        # deadman pushbutton GPIO
DEADMAN_ACTIVE_HIGH = True              # False if your module idles high

SPI_BUS, SPI_DEVICE, SPI_HZ = 0, 0, 500_000
BITS = 14                               # AMT222B-V is 14-bit single-turn
COUNTS = 1 << BITS                      # 16384 counts per revolution
DEG_PER_COUNT = 360.0 / COUNTS
N = len(CS_PINS)

# -- end-effector trigger potentiometer, read from the XIAO ESP32C3 ------------
# The sketch (esp32c3_pot_trigger.ino) prints one line per sample at SAMPLE_HZ:
#
#       P,<raw 0-4095>,<millivolts>,<board millis>
#
# and prefixes anything else with '#'.  We only ever keep the newest line.
#
# The XIAO's USB-C port is the C3's native USB Serial/JTAG peripheral, so it
# enumerates as /dev/ttyACM0 (CDC), NOT /dev/ttyUSB0.  The by-id path is the
# stable one once the encoders and other USB gear are also plugged in, so it
# is tried first; --pot-port overrides both.
POT_PORT_GLOBS = [
    "/dev/serial/by-id/*Espressif*",
    "/dev/serial/by-id/*USB_JTAG*",
    "/dev/ttyACM*",
]
POT_BAUD = 115200          # ignored by native USB CDC, kept for parity
POT_ADC_MAX = 4095         # analogReadResolution(12) in the sketch
POT_STALE_S = 0.5          # no line for this long = treat the pot as absent
POT_REOPEN_S = 2.0         # how often to retry a missing/failed port

OPCUA_ENDPOINT = "opc.tcp://0.0.0.0:4840"   # 0.0.0.0 = whichever NIC is up
OPCUA_WRITE_HZ = 50                          # how often values are published
HTTP_PORT = 8080
CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "calibration.json")

JOINT_NAMES = [f"Joint {i + 1}" for i in range(N)]


# ================================================================ SPI HELPERS ==
# Three small functions the AMT22 protocol needs.  Lifted unchanged from
# dashboard_calibration.py because they are pure protocol, not policy.

def delay_us(microseconds):
    """Busy-wait.  time.sleep() is far too coarse for microsecond gaps."""
    end = time.perf_counter_ns() + int(microseconds * 1000)
    while time.perf_counter_ns() < end:
        pass


def checkbits_valid(word):
    """True if the 16-bit response passed the AMT22 check-bit (parity) test.
    A failed word means a corrupt read; we simply drop it and read again."""
    odd = (word >> 13) ^ (word >> 11) ^ (word >> 9) ^ (word >> 7) \
        ^ (word >> 5) ^ (word >> 3) ^ (word >> 1)
    even = (word >> 12) ^ (word >> 10) ^ (word >> 8) ^ (word >> 6) \
        ^ (word >> 4) ^ (word >> 2) ^ word
    return ((word >> 15) & 1) == (~odd & 1) and ((word >> 14) & 1) == (~even & 1)


def word_to_counts(word):
    """Strip the two check bits off the top, leaving the 14-bit position."""
    return word & 0x3FFF


# ============================================================ ENCODER READING ==

class Encoders(threading.Thread):
    """Reads all encoders round-robin forever, publishing the newest raw count
    per joint in self.counts.  Everything downstream only ever reads that list,
    so nothing else in the program needs to know SPI exists.

    self.counts[i] is None until that encoder has produced one good word."""
    daemon = True

    def __init__(self, demo=False):
        super().__init__()
        self.demo = demo
        self.counts = [None] * N
        self._running = True
        if demo:
            return

        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.mode = 0
        self.spi.max_speed_hz = SPI_HZ
        # All 7 encoders share one bus with software chip selects, so the
        # kernel must not drive CE0/CE1 itself. Some kernel/driver configs
        # refuse this unless dtoverlay=spi0-0cs is set in
        # /boot/firmware/config.txt (then reboot) -- warn rather than crash
        # if it's not set yet, since reads will just glitch, not fail outright.
        try:
            self.spi.no_cs = True
        except OSError:
            print("WARNING: driver refused no_cs -- add 'dtoverlay=spi0-0cs' "
                  "to /boot/firmware/config.txt and reboot, or reads will "
                  "glitch.", file=sys.stderr)
        # active_high=False means .on() pulls the pin LOW = encoder selected.
        self.cs = [DigitalOutputDevice(p, active_high=False, initial_value=False)
                   for p in CS_PINS]
        self._last_ns = [0] * N

    def _read(self, i):
        """One 16-bit transaction against encoder i, with the datasheet's
        minimum timing gaps honoured."""
        since = time.perf_counter_ns() - self._last_ns[i]
        if since < 45_000:                 # 45us minimum between reads
            delay_us(45.0 - since / 1000)
        self.cs[i].on()
        delay_us(3.0)                      # CS low -> first clock
        high = self.spi.xfer2([0x00])[0]
        delay_us(3.0)                      # between the two bytes
        low = self.spi.xfer2([0x00])[0]
        delay_us(3.5)                      # last clock -> CS released
        self.cs[i].off()
        self._last_ns[i] = time.perf_counter_ns()
        return (high << 8) | low

    def run(self):
        while self._running:
            if self.demo:
                # Fake motion so the page and the calibration flow can be
                # exercised without a Pi: each joint drifts at its own speed.
                t = time.time()
                for i in range(N):
                    self.counts[i] = int(
                        (math.sin(t * (0.2 + 0.05 * i)) * 0.45 + 0.5) * COUNTS
                    ) % COUNTS
                time.sleep(0.02)
                continue
            for i in range(N):
                word = self._read(i)
                if checkbits_valid(word):
                    self.counts[i] = word_to_counts(word)
            time.sleep(0.001)              # leave some CPU for the web server

    def stop(self):
        self._running = False
        time.sleep(0.05)
        if not self.demo:
            for line in self.cs:
                line.off()
                line.close()


# ============================================================ DEADMAN SWITCH ==

class Deadman(threading.Thread):
    """Samples the deadman pushbutton and publishes a plain boolean.

    Written to fail closed: if the pin cannot be claimed or a read throws,
    pressed() is False, so the arm is told to stop rather than run away."""
    daemon = True

    def __init__(self, demo=False):
        super().__init__()
        self.pressed = False
        self._running = True
        self._device = None
        if demo:
            return
        # pull_up=None + explicit active_state is the right idiom for a
        # powered button module that drives the line itself.
        self._device = Button(DEADMAN_PIN, pull_up=None,
                              active_state=bool(DEADMAN_ACTIVE_HIGH),
                              bounce_time=0.02)

    def run(self):
        while self._running:
            if self._device is not None:
                try:
                    self.pressed = bool(self._device.is_pressed)
                except Exception:
                    self.pressed = False
            time.sleep(0.005)

    def stop(self):
        self._running = False


# ========================================================== TRIGGER POT (USB) ==

class Pot(threading.Thread):
    """Reads the end-effector trigger pot from the XIAO ESP32C3 over USB CDC.

    Mirrors Encoders/Deadman: it owns the link, and everything downstream just
    looks at self.raw (0-4095) and self.mv.  Both are None whenever we do not
    have a fresh sample, so a pulled cable makes the gripper go quiet rather
    than freeze at whatever value was in flight when it was unplugged.

    The board is free-running -- it prints whether or not anyone is listening,
    and it is never commanded -- so there is nothing to do on a dropped link
    except reopen the port and carry on.  Only the newest line is kept: the
    sketch sends 100 Hz and the publisher wants 50 Hz, so a queue would only
    add latency."""
    daemon = True

    def __init__(self, demo=False, port=None):
        super().__init__()
        self.demo = demo
        self.raw = None             # 0-4095, newest good sample
        self.mv = None              # same instant, through the factory ADC cal
        self.port = None            # the device path actually opened
        self.connected = False
        self.error = ""             # last open failure, shown on the page
        self._hint = port           # --pot-port, if given
        self._ser = None
        self._last_rx = 0.0
        self._running = True

    # -- port handling --

    def _find_port(self):
        if self._hint:
            return self._hint
        for pattern in POT_PORT_GLOBS:
            hits = sorted(glob.glob(pattern))
            if hits:
                return hits[0]
        return None

    def _open(self):
        """Try to open the board.  False (quietly) if it simply is not there."""
        path = self._find_port()
        if path is None:
            self.error = "no ESP32 serial port found"
            return False
        try:
            # timeout keeps readline() from blocking past shutdown; at 100 Hz
            # a whole line has normally arrived within 10 ms anyway.
            self._ser = serial.Serial(path, POT_BAUD, timeout=0.2)
            self._ser.reset_input_buffer()
        except Exception as exc:
            self._ser = None
            # Nearly always either the Arduino Serial Monitor still holding the
            # port, or this user not being in the dialout group.
            self.error = f"{path}: {exc}"
            return False
        self.port = path
        self.error = ""
        return True

    def _close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.connected = False
        self.raw = self.mv = None

    # -- the wire protocol, P,<raw>,<mv>,<ms> --

    def _parse(self, line):
        text = line.decode("ascii", "ignore").strip()
        if not text.startswith("P,"):
            return                      # '#' banner, or a torn partial line
        parts = text.split(",")
        if len(parts) < 3:
            return
        try:
            raw = int(parts[1])
            mv = int(parts[2])
        except ValueError:
            return                      # half a line; the next one will be fine
        if not 0 <= raw <= POT_ADC_MAX:
            return
        self.raw, self.mv = raw, mv
        self._last_rx = time.time()
        self.connected = True

    def run(self):
        if self.demo:
            while self._running:
                # Sweep the trigger back and forth over a plausible slice of
                # the ADC range so the calibration flow can be exercised.
                t = time.time()
                self.raw = int((math.sin(t * 0.6) * 0.5 + 0.5) * 3300) + 250
                self.mv = int(self.raw * 2480 / POT_ADC_MAX)
                self.port = "demo"
                self.connected = True
                self._last_rx = time.time()
                time.sleep(0.01)
            return

        if serial is None:
            self.error = "pyserial not installed"
            return

        while self._running:
            if self._ser is None:
                if not self._open():
                    time.sleep(POT_REOPEN_S)
                    continue
            try:
                line = self._ser.readline()
            except Exception as exc:
                self.error = f"read failed: {exc}"
                self._close()
                time.sleep(POT_REOPEN_S)
                continue
            if line:
                self._parse(line)
            if time.time() - self._last_rx > POT_STALE_S:
                # Board still enumerated but silent (reset, reflash, dead
                # sketch).  Drop the readings; keep the port open and wait.
                self.connected = False
                self.raw = self.mv = None

    def stop(self):
        self._running = False
        time.sleep(0.05)
        self._close()


# =============================================================== CALIBRATION ==

class Calibration:
    """Turns a raw encoder count into a signed KUKA joint angle, and the raw
    pot count into a 0..1 gripper position.

    You calibrate a joint by parking it against one mechanical stop and
    recording the raw count ("lower"), then the other stop ("upper").  From
    those two numbers everything else follows:

        * the arc between them is the joint's full travel
        * its midpoint is angle 0
        * its ends are -JOINT_RANGE_DEG[i] and +JOINT_RANGE_DEG[i]

    The only subtlety is that the encoder is SINGLE-TURN: counts wrap at
    16384, so "lower -> upper" could be either of the two arcs round the
    circle.  We pick whichever arc is closer to the travel we expect (the
    joint's KUKA range), which is right for a roughly 1:1 coupling and is
    what resolves e.g. a 340-degree joint correctly instead of reading it as
    the 20-degree arc the other way.

    The pot is much simpler -- it does not wrap, so its two marks are just the
    ends of a straight line."""

    def __init__(self):
        self.lo = [None] * N        # raw count at the lower stop
        self.hi = [None] * N        # raw count at the upper stop
        self.pot_lo = None          # raw ADC with the trigger released
        self.pot_hi = None          # raw ADC with the trigger fully squeezed
        self.load()

    # -- the joint maths --

    def _arc(self, i):
        """(span_in_counts, direction) for joint i, or (None, 0) if unset.

        direction is +1 when counts increase from the lower stop toward the
        upper one, -1 when they decrease."""
        lo, hi = self.lo[i], self.hi[i]
        if lo is None or hi is None:
            return None, 0
        forward = (hi - lo) % COUNTS            # going up in counts
        backward = (lo - hi) % COUNTS           # going down in counts
        want = (2 * JOINT_RANGE_DEG[i]) / DEG_PER_COUNT      # expected travel
        if abs(forward - want) <= abs(backward - want):
            return forward, +1
        return backward, -1

    def calibrated(self, i):
        span, _ = self._arc(i)
        return span is not None and span > 0

    def angle(self, i, raw):
        """Raw count -> signed joint degrees.  None if uncalibrated/no reading.

        Deliberately NOT clamped to the limits: a joint pushed slightly past a
        stop reports 171 rather than silently sitting on 170, so drift stays
        visible instead of being hidden."""
        if raw is None:
            return None
        span, direction = self._arc(i)
        if not span:
            return None
        # How far we have travelled from the lower stop, in counts.
        t = (direction * (raw - self.lo[i])) % COUNTS
        # Counts beyond the far side of the unreachable arc belong *below* the
        # lower stop, so report them as negative rather than as a full turn.
        if t > span + (COUNTS - span) / 2.0:
            t -= COUNTS
        rng = JOINT_RANGE_DEG[i]
        return JOINT_DIR[i] * (-rng + (t / span) * 2.0 * rng)

    # -- the gripper maths --

    def pot_calibrated(self):
        return (self.pot_lo is not None and self.pot_hi is not None
                and self.pot_lo != self.pot_hi)

    def pot_span(self):
        """Signed distance between the two marks, in ADC counts.  Useful as a
        sanity check: a couple of hundred counts means one of the marks was
        taken in the wrong place, or the pot is wired to 5 V and saturating."""
        if not self.pot_calibrated():
            return None
        return self.pot_hi - self.pot_lo

    def gripper(self, raw):
        """Raw ADC -> 0.0 .. 1.0.  None if uncalibrated or no reading.

        Whichever mark you took first decides the direction, so a pot wired
        backwards needs no sign flag: if pot_hi < pot_lo the denominator is
        negative too and the fraction still comes out the right way up.

        Unlike joint angles this IS clamped.  The joints stay unclamped so
        mechanical drift is visible, but the gripper is a command the KUKA
        side consumes directly, and 1.02 "closed" is not a thing -- the ADC
        also flattens out near the top of its range (see the ADC RANGE note
        in esp32c3_pot_trigger.ino), so values past the marks are noise
        rather than information."""
        if raw is None or not self.pot_calibrated():
            return None
        value = (raw - self.pot_lo) / (self.pot_hi - self.pot_lo)
        return min(1.0, max(0.0, value))

    # -- persistence, so a power cycle does not mean recalibrating --

    def save(self):
        with open(CAL_FILE, "w") as f:
            json.dump({"lo": self.lo, "hi": self.hi,
                       "pot_lo": self.pot_lo, "pot_hi": self.pot_hi},
                      f, indent=2)

    def load(self):
        if not os.path.exists(CAL_FILE):
            return
        with open(CAL_FILE) as f:
            d = json.load(f)
        self.lo, self.hi = d.get("lo", self.lo), d.get("hi", self.hi)
        # .get() defaults mean a calibration.json written before the gripper
        # existed still loads; the pot just comes up uncalibrated.
        self.pot_lo = d.get("pot_lo", self.pot_lo)
        self.pot_hi = d.get("pot_hi", self.pot_hi)


# ============================================================= OPC UA SERVER ==

def build_server():
    """Stand up the OPC UA server and return (server, nodes).

    The layout deliberately mirrors KEP_OPCUA_Server1.py exactly:

        Objects
          2:005-MiniArm            <- object, namespace index 2
            0:start                <- ns=5;s=start
            5:Control              <- folder
              0:Rc_Joint1 .. 7     <- ns=5;s=Rc_Joint1 ...
              0:Rc_GripperPos      <- ns=5;s=Rc_GripperPos   (new)
          2:006-KukaArm
            0:start                <- ns=6;s=start
            6:Control
              0:Rc_Joint1 .. 7     <- ns=6;s=Rc_Joint1 ...
              0:Rc_GripperPos      <- ns=6;s=Rc_GripperPos   (new)

    Note the node ids use namespace indexes 5 and 6, which are NOT in the
    server's namespace array -- that is what the old server did, and clients
    are already written against it, so it is reproduced rather than fixed."""
    server = Server()
    server.set_endpoint(OPCUA_ENDPOINT)
    server.set_server_name("KEP OPCUA Server")
    idx = server.register_namespace("http://kep.com/server")   # -> 2

    objects = server.get_objects_node()
    nodes = {"joints": {}, "gripper": {}, "start": {}, "heartbeat": {}}

    for sys_id, name in ((5, "MiniArm"), (6, "KukaArm")):
        obj = objects.add_object(idx, f"{sys_id:03}-{name}")

        def var(parent, key, value, vtype):
            """Add one variable with the same node id scheme the old server
            used: ns=<system id>;s=<name>, and writable."""
            node = parent.add_variable(
                ua.NodeId(key, sys_id, ua.NodeIdType.String),
                key, ua.Variant(value, vtype))
            node.set_writable(True)
            return node

        # Top-level status fields.  Only `start` is actually driven by this
        # script, but the others exist because the old server had them and a
        # client may be reading them.
        nodes["start"][sys_id] = var(obj, "start", False, ua.VariantType.Boolean)
        nodes["heartbeat"][sys_id] = var(obj, "heartbeat", 0, ua.VariantType.Byte)
        var(obj, "status", "online (alive)", ua.VariantType.String)
        var(obj, "connected", True, ua.VariantType.Boolean)
        var(obj, "progID", 0, ua.VariantType.Int32)

        control = obj.add_folder(sys_id, "Control")
        var(control, "Rc_numJoints", N, ua.VariantType.Int32)
        nodes["joints"][sys_id] = [
            var(control, f"Rc_Joint{j + 1}", 0.0, ua.VariantType.Float)
            for j in range(N)
        ]
        # Normalised trigger: 0.0 = released (open), 1.0 = fully squeezed
        # (closed).  Float like the joints, so the KUKA side reads it the same
        # way; it starts at 0.0 so a client that connects before the pot is
        # calibrated sees "open" rather than a stale or garbage number.
        nodes["gripper"][sys_id] = var(control, "Rc_GripperPos", 0.0,
                                       ua.VariantType.Float)

    server.start()
    return server, nodes


def publisher(nodes, encoders, deadman, pot, cal, stop):
    """Background thread: every 1/OPCUA_WRITE_HZ seconds, push the current
    joint angles, gripper position and deadman state into the server's own
    nodes.

    Because the server lives in this same process these are plain in-memory
    writes -- no network round trip per joint, unlike the old client-side
    encoder_opcua_test.py."""
    beat = 0
    while not stop.is_set():
        for i in range(N):
            angle = cal.angle(i, encoders.counts[i])
            if angle is None:
                continue                # uncalibrated or no reading: leave it
            value = ua.Variant(float(angle), ua.VariantType.Float)
            for sys_id in (5, 6):
                nodes["joints"][sys_id][i].set_value(value)

        # Read pot.raw once so both systems get the same sample, and so it
        # cannot go None between the check and the write.
        grip = cal.gripper(pot.raw)
        if grip is not None:
            value = ua.Variant(float(grip), ua.VariantType.Float)
            for sys_id in (5, 6):
                nodes["gripper"][sys_id].set_value(value)

        held = ua.Variant(bool(deadman.pressed), ua.VariantType.Boolean)
        beat = (beat + 1) % 256
        for sys_id in (5, 6):
            nodes["start"][sys_id].set_value(held)
            nodes["heartbeat"][sys_id].set_value(
                ua.Variant(beat, ua.VariantType.Byte))

        stop.wait(1.0 / OPCUA_WRITE_HZ)

    # On the way out, tell anything downstream that nobody is driving.  The
    # gripper is left where it is on purpose: start=False already says the
    # operator has let go, and slamming Rc_GripperPos to 0 would command the
    # jaws open, which is how you drop whatever is being held.
    off = ua.Variant(False, ua.VariantType.Boolean)
    for sys_id in (5, 6):
        nodes["start"][sys_id].set_value(off)


# ==================================================================== WEB UI ==
# One page, no framework.  It polls /state ten times a second and redraws
# seven SVG dials plus the gripper bar; the calibration buttons are plain GETs
# to /cal (joints) and /potcal (gripper).

PAGE = """<!doctype html>
<meta charset=utf-8><title>Mini-arm</title>
<style>
body{font:14px system-ui,sans-serif;background:#111;color:#eee;margin:14px}
#dm{padding:8px 14px;border-radius:4px;display:inline-block;font-weight:700}
.on{background:#1a7f37}.off{background:#5a1e1e}
#grid{display:flex;flex-wrap:wrap;gap:10px;margin-top:12px}
.j{border:1px solid #333;padding:8px;width:184px}
.nm{color:#aaa;font-size:12px}
.deg{font-size:22px;font-weight:700}
.rad{font-size:12px;color:#999;margin-bottom:4px}
.note{font-size:11px;color:#888;min-height:26px}
button{font:11px system-ui;width:100%;margin-top:2px;padding:3px}
#grip{border:1px solid #333;padding:8px;width:372px;margin-top:10px}
#gbar{height:18px;background:#191919;border:1px solid #333;margin:2px 0 4px}
#gfill{height:100%;width:0;background:#2d6cdf;transition:width .08s linear}
#gbtns{display:flex;gap:4px}
#gbtns button{margin-top:4px}
</style>
<div id=dm class=off>DEADMAN</div>
<div id=grid></div>

<div id=grip>
  <div class=nm>Gripper trigger (Rc_GripperPos)</div>
  <div class=deg id=gval>--</div>
  <div id=gbar><div id=gfill></div></div>
  <div class=rad id=graw></div>
  <div class=note id=gnote></div>
  <div id=gbtns>
    <button onclick="potcal('open')">Set released = 0.00</button>
    <button onclick="potcal('closed')">Set squeezed = 1.00</button>
    <button onclick="potcal('confirm')">Confirm calibration</button>
    <button onclick="potcal('clear')">Clear</button>
  </div>
</div>

<script>
// Convert a dial angle (0 = up, positive = clockwise) into an x/y on a circle.
function pt(a, r){ a = a * Math.PI / 180; return [r * Math.sin(a), -r * Math.cos(a)]; }

// SVG arc path spanning a0..a1 degrees, used to shade the legal joint range.
function arc(a0, a1, r){
  const [x0, y0] = pt(a0, r), [x1, y1] = pt(a1, r);
  return `M${x0} ${y0}A${r} ${r} 0 ${(a1 - a0) > 180 ? 1 : 0} 1 ${x1} ${y1}`;
}

// Build the seven cards once, from the first /state response.
function build(joints){
  grid.innerHTML = joints.map((j, i) => `
    <div class="j">
      <div class="nm">${j.name}</div>
      <svg viewBox="-60 -60 120 120" width="168" height="168">
        <circle r="52" fill="none" stroke="#333" />
        <path d="${arc(-j.range, j.range, 52)}" fill="none" stroke="#2d6cdf" stroke-width="5" />
        <line transform="rotate(${-j.range})" y1="-44" y2="-60" stroke="#e0a030" stroke-width="2" />
        <line transform="rotate(${j.range})" y1="-44" y2="-60" stroke="#e0a030" stroke-width="2" />
        <line id="nd${i}" y1="4" y2="-48" stroke="#eee" stroke-width="2" />
        <circle r="3" fill="#eee" />
      </svg>
      <div class="deg" id="deg${i}">--</div>
      <div class="rad" id="rad${i}"></div>
      <div class="note" id="note${i}"></div>
      <button onclick="cal(${i},'lower')">Set lower limit</button>
      <button onclick="cal(${i},'upper')">Set upper limit</button>
      <button onclick="cal(${i},'confirm')">Confirm calibration</button>
      <button onclick="cal(${i},'clear')">Clear</button>
    </div>`).join('');
}

function cal(i, action){ fetch(`/cal?j=${i}&a=${action}`).then(tick); }
function potcal(action){ fetch(`/potcal?a=${action}`).then(tick); }

async function tick(){
  const s = await (await fetch('/state')).json();
  if (!grid.children.length) build(s.joints);
  dm.className = s.deadman ? 'on' : 'off';
  dm.textContent = s.deadman ? 'DEADMAN: HELD' : 'DEADMAN: RELEASED';
  s.joints.forEach((j, i) => {
    document.getElementById('nd' + i).setAttribute('transform', `rotate(${j.dial})`);
    document.getElementById('deg' + i).textContent =
      j.deg === null ? '--' : j.deg.toFixed(1) + '\\u00b0';
    document.getElementById('rad' + i).textContent =
      j.deg === null ? '' : j.rad.toFixed(3) + ' rad';
    document.getElementById('note' + i).textContent = j.note;
  });

  // The bar shows the normalised value once calibrated; before that it shows
  // the raw ADC across its full 0-4095 span, so you can watch the trigger
  // move while marking the two ends.
  const g = s.gripper;
  gval.textContent = g.value === null ? '--' : g.value.toFixed(3);
  gfill.style.width = (g.bar * 100).toFixed(1) + '%';
  gfill.style.background = g.value === null ? '#555' : '#2d6cdf';
  graw.textContent = g.raw === null
    ? (g.port ? 'no data on ' + g.port : 'no board')
    : `raw ${g.raw} / 4095   ${g.mv} mV`;
  gnote.textContent = g.note;
}
setInterval(tick, 100); tick();
</script>
"""


class Handler(BaseHTTPRequestHandler):
    """Four routes: the page, the live state, and the two calibration actions."""

    # Class attributes, filled in by main() before the server starts.
    encoders = deadman = pot = cal = None
    pending = None          # per-joint {"lo": raw, "hi": raw} awaiting Confirm
    pot_pending = None      # {"lo": raw, "hi": raw} awaiting Confirm

    def log_message(self, *args):
        pass                # silence per-request logging; we poll at 10Hz

    def _send(self, body, content_type):
        body = body.encode()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def joint_state(self):
        joints = []
        for i in range(N):
            raw = self.encoders.counts[i]
            deg = self.cal.angle(i, raw)
            p = self.pending[i]

            # The needle shows the calibrated angle once we have one, and the
            # bare encoder position before that, so you can watch it move
            # while you are marking the two stops.
            if deg is not None:
                dial = deg
            elif raw is not None:
                dial = (raw * DEG_PER_COUNT) % 360.0
            else:
                dial = 0.0

            if p["lo"] is not None and p["hi"] is not None:
                note = "both stops marked - press Confirm"
            elif p["lo"] is not None:
                note = "lower marked - move to the other stop"
            elif p["hi"] is not None:
                note = "upper marked - move to the other stop"
            elif self.cal.calibrated(i):
                note = "calibrated"
            else:
                note = "not calibrated"

            joints.append({
                "name": JOINT_NAMES[i],
                "range": JOINT_RANGE_DEG[i],
                "deg": None if deg is None else round(deg, 2),
                "rad": 0.0 if deg is None else round(math.radians(deg), 4),
                "dial": round(dial, 2),
                "note": note,
            })
        return joints

    def gripper_state(self):
        raw = self.pot.raw
        value = self.cal.gripper(raw)
        p = self.pot_pending

        if value is not None:
            bar = value
        elif raw is not None:
            bar = raw / POT_ADC_MAX     # uncalibrated: show the raw sweep
        else:
            bar = 0.0

        if not self.pot.connected:
            note = self.pot.error or "waiting for the board"
        elif p["lo"] is not None and p["hi"] is not None:
            note = f"both ends marked ({p['lo']} -> {p['hi']}) - press Confirm"
        elif p["lo"] is not None:
            note = "released end marked - squeeze fully, then mark closed"
        elif p["hi"] is not None:
            note = "squeezed end marked - release fully, then mark released"
        elif self.cal.pot_calibrated():
            note = (f"calibrated: {self.cal.pot_lo} -> {self.cal.pot_hi} "
                    f"({abs(self.cal.pot_span())} counts)")
        else:
            note = "not calibrated"

        return {
            "raw": raw,
            "mv": self.pot.mv,
            "value": None if value is None else round(value, 3),
            "bar": round(bar, 4),
            "port": self.pot.port,
            "connected": bool(self.pot.connected),
            "note": note,
        }

    def state(self):
        """Everything the page needs, in one object."""
        return {"deadman": bool(self.deadman.pressed),
                "joints": self.joint_state(),
                "gripper": self.gripper_state()}

    def do_GET(self):
        url = urlparse(self.path)

        if url.path == "/":
            self._send(PAGE, "text/html; charset=utf-8")

        elif url.path == "/state":
            self._send(json.dumps(self.state()), "application/json")

        elif url.path == "/cal":
            q = parse_qs(url.query)
            i = int(q["j"][0])
            action = q["a"][0]
            raw = self.encoders.counts[i]
            if action == "lower":
                self.pending[i]["lo"] = raw
            elif action == "upper":
                self.pending[i]["hi"] = raw
            elif action == "confirm":
                # Commit only when both stops have been marked.
                if self.pending[i]["lo"] is not None and self.pending[i]["hi"] is not None:
                    self.cal.lo[i] = self.pending[i]["lo"]
                    self.cal.hi[i] = self.pending[i]["hi"]
                    self.cal.save()
                    self.pending[i] = {"lo": None, "hi": None}
            elif action == "clear":
                self.pending[i] = {"lo": None, "hi": None}
                self.cal.lo[i] = self.cal.hi[i] = None
                self.cal.save()
            self._send("{}", "application/json")

        elif url.path == "/potcal":
            # Same two-marks-then-Confirm flow as a joint.  "open" is the
            # trigger fully released (-> 0.0), "closed" fully squeezed (-> 1.0).
            action = parse_qs(url.query)["a"][0]
            raw = self.pot.raw
            if action == "open" and raw is not None:
                self.pot_pending["lo"] = raw
            elif action == "closed" and raw is not None:
                self.pot_pending["hi"] = raw
            elif action == "confirm":
                lo, hi = self.pot_pending["lo"], self.pot_pending["hi"]
                # Refuse two marks that are the same (or nearly), because that
                # divides by ~0 and makes the gripper flick between 0 and 1.
                if lo is not None and hi is not None and abs(hi - lo) >= 50:
                    self.cal.pot_lo, self.cal.pot_hi = lo, hi
                    self.cal.save()
                    # Mutate in place: pot_pending is a CLASS attribute shared
                    # by every request, and BaseHTTPRequestHandler builds a new
                    # instance per request, so `self.pot_pending = {...}` would
                    # only rebind on the throwaway instance and the marks would
                    # never actually clear.
                    self.pot_pending.update(lo=None, hi=None)
            elif action == "clear":
                self.pot_pending.update(lo=None, hi=None)
                self.cal.pot_lo = self.cal.pot_hi = None
                self.cal.save()
            self._send("{}", "application/json")

        else:
            self.send_error(404)


# ====================================================================== MAIN ==

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true",
                        help="fake encoders and pot, no hardware needed")
    parser.add_argument("--pot-port", default=None,
                        help="serial port of the XIAO ESP32C3 "
                             "(default: auto-detect /dev/serial/by-id, "
                             "then /dev/ttyACM*)")
    args = parser.parse_args()

    if not args.demo and (spidev is None or Button is None):
        sys.exit("Missing hardware libraries.  Run:\n"
                 "  sudo apt install python3-spidev python3-gpiozero python3-lgpio\n"
                 "or pass --demo to try the UI without hardware.")

    # The pot is optional: without pyserial the arm still works, the gripper
    # just never publishes.  Warn instead of exiting.
    if not args.demo and serial is None:
        print("WARNING: pyserial missing -- gripper disabled.  Run:\n"
              "  sudo apt install python3-serial", file=sys.stderr)

    # 1. hardware readers
    encoders = Encoders(demo=args.demo)
    deadman = Deadman(demo=args.demo)
    pot = Pot(demo=args.demo, port=args.pot_port)
    encoders.start()
    deadman.start()
    pot.start()

    # 2. calibration (restores calibration.json if one exists)
    cal = Calibration()

    # 3. OPC UA server + the thread that keeps its values fresh
    server, nodes = build_server()
    stop = threading.Event()
    pub = threading.Thread(target=publisher,
                           args=(nodes, encoders, deadman, pot, cal, stop),
                           daemon=True)
    pub.start()

    # 4. web page
    Handler.encoders, Handler.deadman = encoders, deadman
    Handler.pot, Handler.cal = pot, cal
    Handler.pending = [{"lo": None, "hi": None} for _ in range(N)]
    Handler.pot_pending = {"lo": None, "hi": None}
    http = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)

    print(f"OPC UA  {OPCUA_ENDPOINT}")
    print(f"web     http://localhost:{HTTP_PORT}")
    print("Ctrl-C to stop")
    try:
        http.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        pub.join(timeout=1.0)
        http.server_close()
        server.stop()
        encoders.stop()
        deadman.stop()
        pot.stop()
        print("stopped")


if __name__ == "__main__":
    main()
