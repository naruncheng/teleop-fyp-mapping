# n-m mapping: leader arm → KUKA iiwa / UR5e / Taipan / SCARA followers

Simulated follower robots (MuJoCo) that track a leader arm's end-effector pose: a 7-DOF
KUKA iiwa 14 (7→7), a 6-DOF UR5e (7→6), the Nova rover's Taipan arm (7→6, four-bar elbow),
a 4-DOF SCARA (7→4), or any combination at once. (The SCARA is a generic model, not a real
robot, and is due to be archived.)

```
leader joint readings -> simulated leader (1:1) -> FK (MuJoCo, leader model) -> pose rel. leader base
    -> per follower: offset + scale into its workspace -> damped least-squares IK -> position servos
```

A leader that does its own FK can skip the first three steps and send a pose directly.

The IK solver (`ik.py`) and the KUKA model are carried over unchanged from `sagittarius-teleop/sim`.

## Setup

```bash
pip install -r requirements.txt
```

## Run

**With the leader arm in the scene:**

```bash
python run_sim.py --joints-demo          # leader sways from code, KUKA follows
python run_sim.py --joints-demo --followers kuka ur5e taipan  # several followers at once
python run_sim.py --joints-demo --followers taipan       # one follower alone
python run_sim.py --joints-opcua opc.tcp://<pi>:4840 --followers kuka ur5e taipan   # real mini-arm via miniarm_v2.py
python run_sim.py --joints-udp 9500      # leader follows joint readings over UDP
python send_test_joints.py               # stand-in physical leader for --joints-udp
```

**Pose only, no leader drawn:**

```bash
python run_sim.py                        # followers hold their home poses
python run_sim.py --demo                 # target sweeps a figure-eight
python run_sim.py --udp 9400             # follow poses over UDP
python send_test_pose.py                 # stand-in leader for --udp
```

Each follower has its own target arrow (KUKA yellow, UR5e orange, Taipan pink, SCARA cyan); its
tool chases it. Once a second the console logs each follower's IK solve error and actual
tool-to-target gap.

The viewer opens with shadows, reflections and haze off (turn them back on in its Rendering
panel). Physics runs at 500 Hz and IK at 50 Hz, stepped to keep up with the wall clock, and the
window redraws at 60 Hz. If the computer can't keep up for a whole second, the console warns
"simulation running slower than real time": use fewer followers. All three of KUKA + UR5e +
Taipan only just fit (about 1.1× real time headless on a laptop), because each IK solve runs over
every joint in the scene. The `--joints-demo` and `--demo` motions run on simulation time, so if
the sim slows they slow with it; a real leader doesn't, so its followers fall behind.

| Option | Default | Meaning |
|---|---|---|
| `--followers NAME...` | `kuka` | Which robots follow: any of `kuka`, `ur5e`, `taipan`, `scara` |
| `--kuka-scale` / `--ur5e-scale` / `--taipan-scale` / `--scara-scale S` | `2` / `2` / `2.8` / `1.35 1.35 0.4` | Multiplier on leader **position**: one value, or X Y Z |
| `--kuka-origin` / `--ur5e-origin` / `--taipan-origin` / `--scara-origin X Y Z` | `0 0 0` / `0 0 0` / `0 0 -0.135` / `0 0 0.03` | Where the leader's base maps to, relative to that robot's base |
| `--leader-offset X Y Z` | `0 0 0` | Where the leader is **drawn** |
| `--position-only` | off | Match position only, ignore orientation |
| `--rot-weight W` | `1.0` | KUKA: orientation vs position trade-off when both can't be met (~0.02 favours position) |

The defaults are tuned for the mini-arm leader. For another leader (e.g. `--udp`), set the
scale for it, e.g. `--kuka-scale 1` for life size.

Where things are drawn (leader at the origin, KUKA at y = -0.6 m, UR5e at y = +0.6 m, Taipan at y = +1.8 m, SCARA at y = +3.0 m) has no
effect on the mapping: each origin is relative to its own robot's base.

## Feeding in the physical leader

### From miniarm_v2.py (what runs on the Pi today)

No changes on the Pi. Run `miniarm_v2.py` there as usual, then on this machine:

```bash
python run_sim.py --joints-opcua opc.tcp://<pi-hostname-or-ip>:4840
```

This connects to its OPC UA server as one more client (alongside the FRI app, if that's
running) and reads `ns=5;s=Rc_Joint1..7`. `opcua_source.py` converts what it publishes:

- **degrees → radians**
- **KUKA convention → leader model convention**, × `[-1, +1, -1, -1, -1, +1, -1]`.
  miniarm_v2's `JOINT_DIR` makes its angles KUKA-ready (the FRI app feeds them straight to the
  KUKA). If a joint comes out mirrored in the sim, check this first. `--opcua-convention model`
  turns the conversion off.

Before trusting the FK, check:

- **Zero = straight up.** miniarm_v2 puts 0 at the midpoint between the two calibration stops.
  That only equals the model's zero (arm straight up) if the stops are symmetric about it.
- **Angles aren't stretched.** Calibration maps each joint's stop-to-stop travel onto
  ±`JOINT_RANGE_DEG` (the KUKA's ranges). If a joint's real travel is different, its reported
  angles are scaled by the ratio, and the FK pose is off by the same amount.
- **Joints must be calibrated.** An uncalibrated joint is never written, so it reads 0.0.

Quick test: hold the arm straight up, so the sim leader should be straight up too. Then move
one joint at a time by a known amount, and compare the sim against the real arm.

If the Pi drops off the network, the followers hold their last pose and the reader reconnects by
itself. `miniarm_v2.py --demo` works as a stand-in Pi on any machine, but it needs a
`calibration.json` beside it before it publishes anything.

### Over UDP

Send one UDP datagram per reading to `run_sim.py --joints-udp PORT`:

```json
{"joints": [q1, q2, q3, q4, q5, q6, q7]}
```

Radians, in the **leader model's convention**: all zeros = arm straight up, positive = the
direction the model turns that joint. Your encoder zeros and directions must match it, or
the simulated leader won't mirror the real one. To check: hold the physical arm straight up
(should read all zeros), then move one joint at a time and confirm the sim turns the same way.

Out-of-range readings are clipped to the model's joint limits; malformed packets are dropped.

## How the two arms line up

- **Model:** `models/leader_arm/urdf/miniarm.xml` (MuJoCo conversion of `main_assembly.urdf`;
  the two agree to 1.5 µm in FK).
- **Base yaw:** the leader's CAD frame is turned 90° about Z relative to the KUKA's (its pitch
  joints rotate about X, the KUKA's about Y), so it's mounted at -90°. After that, at matching
  joint angles both tools have identical orientations in their own base frames.
- **Joint signs:** leader angle × `[-1, +1, -1, -1, -1, +1, -1]` = the KUKA angle that turns the
  same way (`scene.JOINT_SIGNS`). The FK+IK pipeline doesn't use this; it's the direct
  joint-space mapping, if you ever want it.
- **Tool point:** on the joint 7 axis at the top of the Link7_Base disc, 11 mm past joint 7
  (`scene.LEADER_TOOL_POS`), the leader's equivalent of the KUKA flange.
- **Proportions differ:** shoulder height / upper arm / forearm are 0.12 / 0.16 / 0.15 m on
  the leader vs 0.36 / 0.42 / 0.40 m on the KUKA, i.e. 2.6–3.0×, so at 2× the KUKA's elbow
  won't exactly copy the leader's.

## The SCARA follower (n-m)

`models/scara/scara.xml`: a generic 4-axis SCARA sized after the **ABB IRB 910SC-3/0.55**,
using its product specification (3HAC056431-001): 300 mm + 250 mm arms (550 mm reach),
axis 1 ±140°, axis 2 ±150°, 180 mm Z stroke, axis 4 ±400°. Heights, masses and shapes are
plausible primitives, not ABB's. To match a real SCARA, edit those numbers in the XML.

A SCARA can do less than the leader, so the mapping drops what it can't do:

- **Orientation:** its tool always points straight down. Only the leader's twist about
  vertical is kept: the closest tool-down orientation, Rz(ψ)·down, least-squares
  (`follower.project_to_yaw`). Tilt and roll are dropped. When the leader's tool points
  nearly straight up, yaw is meaningless, so the last good yaw is held.
- **Position:** horizontal and vertical are scaled separately, because its vertical travel
  (180 mm) is much smaller than its reach (550 mm). The leader's 0.40 m max reach × 1.35 fits
  inside 0.55 m. Its tool height (0.04–0.45 m, 5th–95th percentile) × 0.4 + 0.03 m fits the
  0.04–0.22 m stroke.
- **IK:** position comes from the same DLS solver. Yaw is then set on axis 4 directly, using
  whichever ±360° equivalent is in range and nearest its current angle. Leaving yaw to IK let
  axis 4 ratchet into its limit, after which IK gave up hundreds of mm of position (10% of
  solves ok in a wide-motion test, 71% after the fix).

Known limits, from a wide-motion test where every leader joint sweeps most of its range:

- **15% of solves:** the target needs the elbow bent the other way. Local IK can't flip a
  SCARA's elbow, because that means passing through the straight-arm singularity.
- **12% of solves:** the target is inside the 150 mm hole around its own axis, or outside the
  Z stroke.

With the normal demo motion it tracks 100% of solves within 10 mm / 5°.

## The Taipan follower (7→6, four-bar elbow)

`models/taipan/`: the Nova rover's Taipan arm, generated from its Onshape URDF. The generated
`taipan.xml` and `assets/` are committed, so running the sim needs nothing else. To regenerate
(after re-exporting the arm), the build reads `nova/taipan_mujoco.urdf` and the meshes from a
sparse checkout of MonashNovaRover/nova beside this folder (`nova/` is gitignored):

```bash
# from the repo root, NOT from inside mapping/ (that would make this repo sparse instead)
git clone --filter=blob:none --no-checkout https://github.com/MonashNovaRover/nova/
cd nova
git config core.protectNTFS false   # Windows: the repo has files with ':' in the name, outside this folder
git sparse-checkout set src/ros/rover/rover_description/taipan/meshes
git checkout main
# put taipan_mujoco.urdf in nova/, then:
cd ../mapping
python models/taipan/build_taipan.py
```

`taipan_mujoco.urdf` is Nova's `taipan/urdf/arm.urdf` with three edits: mesh paths made relative
(`file://$(arg rover_description_dir)/` stripped), the xacro namespace dropped so the root tag is
exactly `<robot name="Arm">` (the build looks for that), and the two tensioner brackets
(`tensioner_v2___bracket`, `..._mirrored`) removed.

It isn't a plain serial arm: both shoulder motors sit on one axis. `j2` drives the upper arm and
`j3` drives a 0.1 m crank, and a parallelogram (crank → 0.5 m rod → 0.1 m lever on the forearm)
carries `j3` up to the elbow. The URDF can't express a closed loop, so the exporter faked it with
mimic joints. The build replaces those with the real loop: the elbow (`l1_l2_pivot`) and the rod's
top pivot (`l1_p_upper`) are passive hinges, and a `connect` constraint pins the rod to the crank
tip. So the elbow is moved by the two shoulder motors through the linkage, as on the hardware:
elbow = `j2 − j3`, and `j2` alone swings the upper arm while the forearm keeps its angle.

In the follower (`FollowerSpec.four_bar`), IK solves the shoulder and the passive elbow like any
other joints (the crank doesn't move the tool), then the crank motor is sent `j2 − elbow`. The
gravity feedforward for the passive pivots goes to the two shoulder motors (added to `j2`,
subtracted from `j3`).

Other changes made by the build:

- Root body `base`, turned +90° so the arm faces +x like the others (the URDF faces −y).
  Stands on its base box, 0.163 m below `base`.
- `attachment_site` on the `j6` axis at the flange, +Z out toward the gripper, the same point
  the KUKA, UR5e and leader use. (Nova's own end-effector frame is 0.2225 m further out, at the
  fingers.)
- Collision boxes in geom group 3, hidden by default: they're solid stand-ins that otherwise
  cover the detailed meshes.
- Meshes over 20k faces simplified (0.5 mm vertex merge): 57 → 25 MB, and MuJoCo can't load the
  212k-face lead screw at all.
- `armature` 0.01 on every joint and servo gains of 200 (`j1`–`j3`) / 100 (wrist). With no motor
  inertia the light wrist shook; stiffer shoulder gains (500) set up a slow wobble through the
  linkage.

**Not from Nova, to replace with real values:** joint limits (`JOINT_RANGE_DEG` in the build
script: elbow ±75° keeps the parallelogram clear of folding flat at about ±88°), servo gains, and
armature. Masses and inertias are Onshape's, including the two tensioner brackets removed from
the visuals.

Scale 2.8× maps the leader's 0.40 m reach onto the Taipan's ~1.17 m (shoulder to flange). The
origin's −0.135 m puts the leader's shoulder (0.12 m up × 2.8) on the Taipan's (0.20 m up).

## The UR5e follower (7→6)

`models/universal_robots_ur5e/`: the MuJoCo Menagerie model, unchanged. With 6 joints it can
still match the leader's full pose; there's just no spare joint, so each pose has a fixed
arm shape (up to elbow-up/down and wrist-flip choices). Scale 2× fits the leader's 0.40 m
reach inside its 0.85 m. 1.6×, 1.8× and 2.0× all measured the same on the demo motion.

## Adding another follower

Add a model under `models/` that names its base body `base`, its tool site `attachment_site`
(+Z out of the flange), and a keyframe `home` covering every joint (passive ones too: the
follower starts from it), with one position actuator per motor (any names). Menagerie arms
mostly follow this already, as the UR5e did. Then add a `FollowerSpec` to `scene.FOLLOWERS`. Use
`orientation="yaw"` plus `yaw_joint` for any robot whose tool can only turn about vertical, and
`four_bar` for a parallelogram-driven elbow like the Taipan's.

## Plugging in a pose-only leader

Subclass `PoseSource` and return its FK result as `(position (3,), rotation (3, 3))` in metres,
relative to the leader's base, with the base axes matching the KUKA's (Z up) and the tool's
+Z pointing out of the flange:

```python
from pose_sources import PoseSource

class MyLeader(PoseSource):
    def latest(self):
        T = my_forward_kinematics(read_my_arm())   # 4x4
        return T[:3, 3], T[:3, :3]                 # or None if no reading yet
```

Or send UDP to `--udp PORT`: `{"position": [x, y, z], "rotation": [[...], [...], [...]]}`,
or `"quaternion": [w, x, y, z]` in place of `"rotation"`.

## Files

| File | Role |
|---|---|
| `follower.py` | `Follower` (any robot): pose mapping, yaw projection, IK, servos with gravity feedforward |
| `ik.py` | Damped least-squares IK (joint-limit aware, step-capped, seeded from current pose) |
| `leader.py` | `LeaderArm` (shows readings, does FK), `DemoJoints`, `UdpJointSource` |
| `opcua_source.py` | `OpcUaJointSource`: reads miniarm_v2.py's OPC UA joint nodes |
| `pose_sources.py` | `PoseSource` interface, `DemoPose`, `UdpPoseSource`, shared UDP listener |
| `scene.py` | Builds the model: world + chosen followers + leader; `FOLLOWERS` registry |
| `run_sim.py` | MuJoCo viewer entry point: real-time loop, IK every 10th step, 60 Hz redraw |
| `send_test_joints.py` / `send_test_pose.py` | Stand-in leaders over UDP |
| `world.xml` | Empty world: floor, lights, target-marker arrowhead |
| `models/kuka_iiwa_14/` | MuJoCo Menagerie iiwa 14 (see its LICENSE) |
| `models/taipan/` | Nova Taipan arm: `build_taipan.py` makes `taipan.xml` + `assets/` from the URDF |
| `models/scara/` | Generic SCARA (IRB 910SC-3/0.55 dimensions), to be archived |
| `models/universal_robots_ur5e/` | MuJoCo Menagerie UR5e, unchanged (see its LICENSE) |
| `models/leader_arm/` | Leader arm URDF, MJCF and meshes (copied from `../sim/Main Assembly_3`) |

## Measured behaviour (headless, 60 s of `--joints-demo`)

- KUKA (2×): 100% of IK solves within 10 mm / 5°. Tool gap about 10 mm mean and 21 mm max
  while moving (servo lag), about 0.02 mm once the target stops.
- SCARA: 100% within 10 mm / 5°, tool gap about 2 mm mean and 6 mm max.
- Taipan (2.8×): 100% within 10 mm / 5°, tool gap about 14 mm mean and 29 mm max, tool never
  below 0.15 m, no contacts. The four-bar holds elbow = `j2 − j3` to within 0.004 rad. Live in
  the viewer with KUKA + UR5e alongside, gaps were 7–23 mm across all three.
- UR5e (2×): 100% within 10 mm / 5° after the first 0.34 s, tool gap about 18 mm mean and
  39 mm max. Its home pose is about 0.5 m from where the leader maps, so it swings there at
  startup. Its wrist servos are softer than the KUKA's, hence the larger lag.
- The leader stays exactly on its readings.
- KUKA known limits:
  - When the leader reaches down to its own base level, the 2× target is at the floor, and
    the KUKA (whose IK doesn't know about the floor) pushes into it.
  - Upright, close-in leader poses map near the KUKA's shoulder at 2×, where it misses up to
    ~17% of solves. 2.4× had none in the same test.
  - The first reading makes the KUKA jump from its home pose to wherever that reading maps.
