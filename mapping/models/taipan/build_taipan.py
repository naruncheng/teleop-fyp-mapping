"""Convert the Nova rover's Taipan arm URDF into this project's MuJoCo follower
model: taipan.xml plus its meshes in assets/.

    python models/taipan/build_taipan.py [path/to/taipan_mujoco.urdf]

The URDF (an onshape-to-robot export) is read from the nova checkout beside
this repo by default, with its meshes resolved against
src/ros/rover/rover_description/ there. Rerun after re-exporting the arm.

What changes on the way, and why:

  * Four-bar. The URDF can't express the closed loop between the shoulder
    and the elbow, so the exporter faked it with mimic joints that lock the
    elbow to j2 - j3. Here the mimics are removed and the loop is closed for
    real with a connect constraint (crank tip pinned to the coupler rod): j2
    drives the upper arm, j3 drives the crank, and the elbow (l1_l2_pivot)
    and the rod's top pivot (l1_p_upper) are passive.
  * Follower conventions (see scene.FollowerSpec): the root body is "base",
    turned so the arm faces +x like the other followers (the URDF faces -y);
    the tool site "attachment_site" sits on the j6 axis at the flange with +Z
    pointing out of it; keyframe "home" is the CAD pose; one position servo
    per motor (j1..j6).
  * Collision boxes go in geom group 3, hidden by default: they're solid
    stand-ins that otherwise cover the detailed meshes.
  * armature on every joint: the light wrist with no motor inertia made the
    servos shake.
  * Meshes over MAX_FACES are simplified (MuJoCo can't load STLs over 200k
    faces, and the full set is ~57 MB).
"""

import pathlib
import shutil
import sys

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).parent
DEFAULT_URDF = HERE.parents[2] / "nova" / "taipan_mujoco.urdf"
MESH_ROOT = pathlib.Path("src/ros/rover/rover_description")  # relative to the URDF's folder
ASSETS = HERE / "assets"
OUT = HERE / "taipan.xml"

# Meshes above this are simplified by merging vertices within DECIMATE_CELL.
MAX_FACES = 20000
DECIMATE_CELL = 0.0005  # m

# Not from Nova: the ROS config allows +/-4 pi everywhere. These keep the arm
# out of itself and the four-bar away from folding flat (it does at an elbow
# of about +/-88 deg from the CAD pose, where upper arm and crank line up).
# Replace with the real hardware limits.
JOINT_RANGE_DEG = {
    "j1": (-170, 170),
    "j2": (-90, 90),
    "l1_l2_pivot": (-75, 75),  # elbow, passive
    "j4": (-170, 170),
    "j5": (-110, 110),
    "j6": (-180, 180),
}

# Position servos: (kp, kv). The scene's implicitfast integrator keeps kv stable.
# kp 500 on j2/j3 set up a sustained ~0.25 rad/s wobble through the four-bar
# whenever the elbow was near its CAD angle; 200 settles everywhere tested.
SERVO = {"j1": (200, 20), "j2": (200, 20), "j3": (200, 20),
         "j4": (100, 10), "j5": (100, 10), "j6": (100, 10)}
ARMATURE = 0.01

CRANK_LENGTH = 0.1  # m: crank tip, along l2_prime's x, where the coupler rod pins on


def load_urdf(urdf):
    xml = urdf.read_text(encoding="utf-8")
    meshdir = (urdf.parent / MESH_ROOT).as_posix()
    # discardvisual: MuJoCo drops URDF visuals by default. fusestatic off keeps
    # arm_link as a body, to become "base". boundmass: the exporter's dummy
    # links are massless, which MuJoCo rejects for moving bodies.
    compiler = (f'<mujoco><compiler meshdir="{meshdir}" discardvisual="false" fusestatic="false" '
                'balanceinertia="true" boundmass="1e-3" boundinertia="1e-6"/></mujoco>')
    xml = xml.replace('<robot name="Arm">', '<robot name="Arm">' + compiler, 1)
    return mujoco.MjSpec.from_string(xml), urdf.parent / MESH_ROOT


def close_four_bar(spec):
    for eq in list(spec.equalities):  # the exporter's mimic couplings
        spec.delete(eq)
    # Each mimic pair is two coaxial hinges; keep one of each as a passive pivot.
    for name in ("l1_l2_pivot_mimic", "l1_p_upper_mimic"):
        spec.delete(spec.joint(name))
    # The rod's lower end hangs off the crank as massless dummy bodies; the
    # connect constraint replaces them.
    spec.delete(spec.body("l1_p_lower_j3_mimic"))
    # MuJoCo places the matching point on the rod from the CAD pose (qpos0),
    # where it coincides with the crank tip.
    spec.add_equality(type=mujoco.mjtEq.mjEQ_CONNECT, name="four_bar",
                      objtype=mujoco.mjtObj.mjOBJ_BODY, name1="l2_prime", name2="l1_prime",
                      data=[CRANK_LENGTH, 0, 0] + [0] * 8,
                      # Stiffer than default, which let the pin stretch ~1 mm under the arm's weight.
                      solref=[0.004, 1], solimp=[0.99, 0.999, 0.001, 0.5, 2])


def apply_conventions(spec):
    base = spec.body("arm_link")
    base.name = "base"
    base.quat = [np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)]  # +90 deg about z: URDF -y -> +x

    # Flange: on the j6 axis at ee_base's origin. The gripper extends along
    # ee_base -z (Nova's own end-effector frame is 0.2225 m down it), so flip
    # about x to make +Z point out of the flange.
    spec.body("ee_base").add_site(name="attachment_site", quat=[0, 1, 0, 0],
                                  size=[0.01, 0, 0], group=5)

    for j in spec.joints:
        j.armature = ARMATURE
        if j.name in JOINT_RANGE_DEG:
            j.range = np.radians(JOINT_RANGE_DEG[j.name])
            j.limited = mujoco.mjtLimited.mjLIMITED_TRUE

    for g in spec.geoms:
        if g.type != mujoco.mjtGeom.mjGEOM_MESH:
            g.group = 3

    for name, (kp, kv) in SERVO.items():
        act = spec.add_actuator(name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=kp, kv=kv)
        act.ctrlrange = [-np.pi, np.pi]
        act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE

    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST


def exclude_resting_contacts(spec):
    """The exported collision boxes overlap where links meet (l1 and l2 by
    ~18 mm), which would push the joints apart forever. Exclude every body
    pair already touching in the CAD pose."""
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    pairs = set()
    for c in d.contact[:d.ncon]:
        b1, b2 = (m.body(m.geom_bodyid[g]).name for g in (c.geom1, c.geom2))
        pairs.add(tuple(sorted((b1, b2))))
    for b1, b2 in sorted(pairs):
        spec.add_exclude(bodyname1=b1, bodyname2=b2)
    return pairs


def decimate_stl(src, dst, cell):
    """Vertex clustering: snap vertices to a cell-sized grid, average each
    cluster, and drop faces that collapse. Returns (faces_in, faces_out)."""
    raw = src.read_bytes()
    n = int.from_bytes(raw[80:84], "little")
    rec_t = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec = np.frombuffer(raw[84:84 + 50 * n], dtype=rec_t)
    v = rec["v"].reshape(-1, 3).astype(np.float64)
    _, inv = np.unique(np.round(v / cell).astype(np.int64), axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((inv.max() + 1, 3))
    np.add.at(sums, inv, v)
    verts = sums / np.bincount(inv)[:, None]
    f = inv.reshape(-1, 3)
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    _, keep = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
    f = f[np.sort(keep)]
    tri = verts[f].astype(np.float32)
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    out = np.zeros(len(f), dtype=rec_t)
    out["n"], out["v"] = nrm, tri
    dst.write_bytes(b"decimated".ljust(80, b" ") + len(f).to_bytes(4, "little") + out.tobytes())
    return n, len(f)


def copy_meshes(spec, mesh_root):
    """Copy every mesh into assets/ (simplifying big ones) and point the spec there."""
    if ASSETS.exists():
        shutil.rmtree(ASSETS)
    ASSETS.mkdir()
    for mesh in spec.meshes:
        src = mesh_root / mesh.file
        dst = ASSETS / src.name
        if not dst.exists():
            faces = int.from_bytes(src.read_bytes()[80:84], "little")
            if faces > MAX_FACES:
                n, k = decimate_stl(src, dst, DECIMATE_CELL)
                print(f"  simplified {src.name}: {n} -> {k} faces")
            else:
                shutil.copyfile(src, dst)
        mesh.file = src.name
    spec.meshdir = ASSETS.as_posix()  # absolute while building; made relative before writing


def main():
    urdf = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_URDF
    spec, mesh_root = load_urdf(urdf)
    close_four_bar(spec)
    apply_conventions(spec)
    copy_meshes(spec, mesh_root)  # first: the raw lead screw is too big for MuJoCo to compile
    pairs = exclude_resting_contacts(spec)
    print(f"excluded resting contacts: {sorted(pairs)}")

    m = spec.compile()
    spec.add_key(name="home", qpos=[0.0] * m.nq, ctrl=[0.0] * m.nu)
    # to_xml recompiles, so the meshdir stays absolute until the text is out.
    xml = spec.to_xml().replace(f'meshdir="{ASSETS.as_posix()}/"', 'meshdir="assets/"')
    xml = xml.replace(f'meshdir="{ASSETS.as_posix()}"', 'meshdir="assets"')
    # to_xml adds <size nkey>, which clashes with the scene's when attached.
    xml = xml.replace('  <size nkey="1"/>\n', '')
    OUT.write_text(xml, encoding="utf-8")

    m = mujoco.MjModel.from_xml_path(str(OUT))  # check the written file loads on its own
    size = sum(p.stat().st_size for p in ASSETS.iterdir()) / 1e6
    print(f"wrote {OUT.name}: {m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, "
          f"{m.nmesh} meshes ({size:.1f} MB)")


if __name__ == "__main__":
    main()
