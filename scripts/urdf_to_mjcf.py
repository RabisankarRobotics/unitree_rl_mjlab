"""Convert the tahiti_c1 URDF into an mjlab-ready MJCF.

The URDF carries kinematics, inertials and visual meshes. mjlab additionally needs:

  * a free joint on the root body
  * named collision geoms (``*_collision``) so CollisionCfg regexes can target them
  * foot sites (``left_foot`` / ``right_foot``) for foot_height / clearance / slip
  * an IMU site plus four sensors: imu_ang_vel, imu_lin_vel, imu_lin_acc, root_angmom

Run from the repo root:

    python scripts/urdf_to_mjcf.py

Re-run it whenever the URDF is revised; do not hand-edit the generated XML.

Self-collision
--------------
Every collision geom is emitted with ``contype=0, conaffinity=1``: it collides with
the terrain (contype 1) but not with other robot geoms. This mirrors G1's
``FULL_COLLISION_WITHOUT_SELF``. It is required here because the capsules are fitted
to visual-mesh bounding boxes, and adjacent joint housings genuinely overlap --
``hip_yaw`` and ``hip_roll`` interpenetrate by ~28 mm at the nominal pose. They are
not a parent-child pair, so MuJoCo does not exclude them automatically, and the
solver would fling the robot over at t=0.

To enable self-collision later, tighten the capsules to real CAD envelopes first,
then flip the flags from the CollisionCfg in the constants file rather than here.
"""

from __future__ import annotations

import pathlib

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
XMLS = ROOT / "src/assets/robots/tahiti_c1/xmls"
URDF = XMLS / "tahiti_c1.urdf"
OUT = XMLS / "tahiti_c1.xml"

ROOT_BODY = "base_link"
IMU_POS = (0.07, 0.0, 0.04)  # from the URDF's imu_in_pelvis fixed joint

# Foot plate. The ankle_roll_link STL does NOT contain the foot (its mesh is only
# ~0.067 m in x), but the URDF inertial tensor does: it is equivalent to a solid
# box 0.235 x 0.096 x 0.034 m centred on the link COM. These numbers are derived
# from that tensor -- replace them with the real CAD dimensions when available.
FOOT_CENTER = (0.0303, 0.0003, -0.0126)
FOOT_HALF = (0.1175, 0.0480, 0.0170)
FOOT_FRICTION = 0.6

# Links that get a collision capsule. Feet are handled separately.
NO_COLLISION = ("ankle_roll_link", "ankle_pitch_link")


def mesh_extent(model: mujoco.MjModel, geom_id: int) -> tuple[np.ndarray, np.ndarray]:
  mid = model.geom_dataid[geom_id]
  adr, num = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
  verts = model.mesh_vert[adr:adr + num]
  return verts.min(axis=0), verts.max(axis=0)


def collision_capsule(lo: np.ndarray, hi: np.ndarray) -> tuple[list[float], float]:
  """Capsule fromto + radius inscribed in a bounding box."""
  size = hi - lo
  axis = int(np.argmax(size))
  others = [i for i in range(3) if i != axis]
  radius = 0.5 * min(size[others[0]], size[others[1]])
  centre = 0.5 * (lo + hi)
  a, b = centre.copy(), centre.copy()
  half = max(0.5 * size[axis] - radius, 1e-4)
  a[axis], b[axis] = centre[axis] - half, centre[axis] + half
  return [*a.tolist(), *b.tolist()], float(radius)


def main() -> None:
  assets = {p.name: p.read_bytes() for p in (XMLS / "meshes").glob("*.STL")}
  spec = mujoco.MjSpec.from_file(str(URDF), assets=assets)

  spec.modelname = "tahiti_c1"
  spec.compiler.degree = False  # radians
  spec.compiler.autolimits = True
  spec.meshdir = "meshes"

  # Measure meshes from a throwaway compile before we start editing the spec.
  probe = mujoco.MjSpec.from_file(str(URDF), assets=assets).compile()
  extents: dict[str, tuple[np.ndarray, np.ndarray]] = {}
  for g in range(probe.ngeom):
    if probe.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
      continue
    bname = mujoco.mj_id2name(probe, mujoco.mjtObj.mjOBJ_BODY, probe.geom_bodyid[g])
    extents.setdefault(bname, mesh_extent(probe, g))

  root = next(b for b in spec.bodies if b.name == ROOT_BODY)
  root.add_freejoint()

  # --- geoms ---------------------------------------------------------------
  for body in spec.bodies:
    if body.name in ("world", ""):
      continue
    for idx, geom in enumerate(body.geoms):
      geom.group = 2
      geom.contype = 0
      geom.conaffinity = 0
      geom.density = 0.0
      if not geom.name:
        # A link may carry several visual meshes; keep the names unique.
        geom.name = f"{body.name}_visual{idx}" if idx else f"{body.name}_visual"

    if body.name.endswith("ankle_roll_link"):
      side = body.name.split("_")[0]
      foot = body.add_geom()
      foot.name = f"{side}_foot1_collision"
      foot.type = mujoco.mjtGeom.mjGEOM_BOX
      foot.pos = FOOT_CENTER
      foot.size = FOOT_HALF
      foot.condim = 3
      foot.priority = 1
      foot.friction = [FOOT_FRICTION, FOOT_FRICTION * 0.005, FOOT_FRICTION * 0.0001]
      foot.group = 3
      foot.density = 0.0
      # Terrain-only: collide with the ground (contype 1) but not with other
      # robot geoms. See the note on self-collision at the top of this file.
      foot.contype = 0
      foot.conaffinity = 1

      site = body.add_site()
      site.name = f"{side}_foot"
      site.pos = (FOOT_CENTER[0], FOOT_CENTER[1], FOOT_CENTER[2] - FOOT_HALF[2])
      site.size = [0.01, 0.01, 0.01]
      site.group = 5
      continue

    if body.name.endswith(NO_COLLISION) or body.name not in extents:
      continue

    fromto, radius = collision_capsule(*extents[body.name])
    col = body.add_geom()
    col.name = f"{body.name}_collision"
    col.type = mujoco.mjtGeom.mjGEOM_CAPSULE
    col.fromto = fromto
    col.size = [radius, 0.0, 0.0]
    col.condim = 1
    col.group = 3
    col.density = 0.0
    col.contype = 0
    col.conaffinity = 1

  # --- IMU site + sensors ---------------------------------------------------
  imu = root.add_site()
  imu.name = "imu_in_pelvis"
  imu.pos = IMU_POS
  imu.size = [0.01, 0.01, 0.01]
  imu.group = 5

  for sensor_name, sensor_type, objtype, objname in (
    ("imu_ang_vel", mujoco.mjtSensor.mjSENS_GYRO, mujoco.mjtObj.mjOBJ_SITE, imu.name),
    ("imu_lin_vel", mujoco.mjtSensor.mjSENS_VELOCIMETER, mujoco.mjtObj.mjOBJ_SITE, imu.name),
    ("imu_lin_acc", mujoco.mjtSensor.mjSENS_ACCELEROMETER, mujoco.mjtObj.mjOBJ_SITE, imu.name),
    ("root_angmom", mujoco.mjtSensor.mjSENS_SUBTREEANGMOM, mujoco.mjtObj.mjOBJ_BODY, ROOT_BODY),
  ):
    s = spec.add_sensor()
    s.name = sensor_name
    s.type = sensor_type
    s.objtype = objtype
    s.objname = objname

  model = spec.compile()  # fail loudly here rather than at training time
  OUT.write_text(spec.to_xml())

  print(f"wrote {OUT.relative_to(ROOT)}")
  print(f"  bodies {model.nbody}  dofs {model.nv}  geoms {model.ngeom} "
        f"sites {model.nsite}  sensors {model.nsensor}")
  print(f"  mass {float(sum(model.body_mass)):.2f} kg")
  for s in range(model.nsensor):
    print(f"  sensor: {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, s)}")


if __name__ == "__main__":
  main()
