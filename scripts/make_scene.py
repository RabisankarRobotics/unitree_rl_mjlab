"""Build a standalone MuJoCo scene around the tahiti_c1 robot.

The training pipeline never needs this: mjlab builds its scene from the task
config. This file is for the DEPLOYMENT path -- scripts/mujoco_deploy.py and any
C++ runtime -- which loads a plain MJCF and needs a floor, lights and a set of
<position> actuators driving the joints.

Gains, default pose and effort limits are read out of the generated
deploy_config.yaml, so the scene always matches the trained policy.

Run from the repo root:

    python scripts/make_scene.py                       # newest run
    python scripts/make_scene.py --config <path.yaml>

Re-run it after retraining; do not hand-edit the generated XML.
"""

from __future__ import annotations

import argparse
import pathlib

import mujoco
import yaml

ROOT = pathlib.Path(__file__).resolve().parent.parent
XMLS = ROOT / "src/assets/robots/tahiti_c1/xmls"
ROBOT_XML = XMLS / "tahiti_c1.xml"
OUT = XMLS / "scene_tahiti_c1.xml"


def newest_config() -> pathlib.Path:
  runs = sorted(
    (ROOT / "logs/rsl_rl/tahiti_c1_velocity").glob("*/deploy_config.yaml"),
    key=lambda p: p.stat().st_mtime,
  )
  if not runs:
    raise FileNotFoundError(
      "No deploy_config.yaml found. Run scripts/export_deploy_config.py first."
    )
  return runs[-1]


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--config", default=None, help="deploy_config.yaml; default = newest")
  ap.add_argument("-o", "--output", default=str(OUT))
  args = ap.parse_args()

  cfg_path = pathlib.Path(args.config) if args.config else newest_config()
  cfg = yaml.safe_load(cfg_path.read_text())
  joints = cfg["joints"]

  assets = {p.name: p.read_bytes() for p in (XMLS / "meshes").glob("*.STL")}
  spec = mujoco.MjSpec.from_file(str(ROBOT_XML), assets=assets)
  spec.modelname = "scene_tahiti_c1"

  # Physics timestep must match what the policy was trained with.
  spec.option.timestep = float(cfg["control"]["physics_dt"])

  # --- ground plane + lighting -------------------------------------------
  spec.add_texture(
    name="skybox",
    type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
    builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
    rgb1=[0.3, 0.5, 0.7],
    rgb2=[0.0, 0.0, 0.0],
    width=512,
    height=3072,
  )
  spec.add_texture(
    name="groundplane",
    type=mujoco.mjtTexture.mjTEXTURE_2D,
    builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
    mark=mujoco.mjtMark.mjMARK_EDGE,
    rgb1=[0.2, 0.3, 0.4],
    rgb2=[0.1, 0.2, 0.3],
    markrgb=[0.8, 0.8, 0.8],
    width=300,
    height=300,
  )
  mat = spec.add_material(name="groundplane", texuniform=True, texrepeat=[5, 5])
  mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "groundplane"

  floor = spec.worldbody.add_geom()
  floor.name = "floor"
  floor.type = mujoco.mjtGeom.mjGEOM_PLANE
  floor.size = [0.0, 0.0, 0.05]
  floor.material = "groundplane"
  floor.condim = 3
  # Foot friction as trained (c1_constants FULL_COLLISION_WITHOUT_SELF).
  floor.friction = [0.6, 0.005, 0.0001]

  light = spec.worldbody.add_light()
  light.pos = [0.0, 0.0, 3.5]
  light.dir = [0.0, 0.0, -1.0]
  light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL

  # --- actuators ----------------------------------------------------------
  # <position> actuators reproduce the training control law exactly:
  #   tau = clamp(kp*(ctrl - q) - kd*qd, +/- effort_limit)
  # ctrllimited stays False so the policy may command targets beyond the
  # kinematic range, matching mjlab's create_position_actuator.
  by_name = {j["name"]: j for j in joints}
  for jspec in spec.joints:
    j = by_name.get(jspec.name)
    if j is None:
      continue
    a = spec.add_actuator()
    a.name = jspec.name
    a.target = jspec.name
    a.trntype = mujoco.mjtTrn.mjTRN_JOINT
    a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    a.gainprm[0] = j["kp"]
    a.biasprm[1] = -j["kp"]
    a.biasprm[2] = -j["kd"]
    a.ctrllimited = False
    a.inheritrange = 0.0
    a.forcelimited = True
    a.forcerange[:] = [-j["effort_limit"], j["effort_limit"]]
    jspec.armature = j["armature"]
    jspec.frictionloss = j["frictionloss"]

  # --- home keyframe ------------------------------------------------------
  # qpos = [x y z, quat(4), joints...]; z from the trained init height.
  qpos = [0.0, 0.0, 0.92, 1.0, 0.0, 0.0, 0.0]
  order = {j.name: i for i, j in enumerate(spec.joints) if j.name}
  hinge = [j["default_pos"] for j in sorted(joints, key=lambda j: order.get(j["name"], 0))]
  qpos += hinge
  kf = spec.add_key()
  kf.name = "home"
  kf.qpos = qpos
  kf.ctrl = hinge

  model = spec.compile()  # fail here rather than at runtime
  out = pathlib.Path(args.output)
  out.write_text(spec.to_xml())

  print(f"wrote {out.relative_to(ROOT)}   (gains from {cfg_path.name})")
  print(
    f"  bodies {model.nbody}  dofs {model.nv}  actuators {model.nu} "
    f"sensors {model.nsensor}  timestep {model.opt.timestep}"
  )
  print(f"  mass {float(sum(model.body_mass)):.2f} kg   keyframe 'home' at z=0.92")


if __name__ == "__main__":
  main()
