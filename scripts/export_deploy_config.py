"""Emit a self-contained deployment config for a trained policy.

Reads the trained ONNX and the task config, and writes a single YAML holding
everything a runtime controller needs: joint order, default pose, PD gains,
action scales, effort limits and the exact observation layout.

Nothing in the output is hand-typed -- every value is read back from the
compiled model or the ONNX metadata, so the file cannot drift from the policy
it describes. Regenerate it after every training run.

Usage:
  python scripts/export_deploy_config.py --task Tahiti-C1-Flat
  python scripts/export_deploy_config.py --task Tahiti-C1-Flat --run <log_dir> -o out.yaml
"""

from __future__ import annotations

import argparse
import os
import pathlib
from datetime import datetime, timezone

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import onnx  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402

import src.tasks  # noqa: E402,F401  (registers the tasks)
from mjlab.envs import ManagerBasedRlEnv  # noqa: E402
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg  # noqa: E402


def _f(x) -> float:
  return round(float(x), 6)


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--task", default="Tahiti-C1-Flat")
  ap.add_argument("--run", default=None, help="log dir; default = newest")
  ap.add_argument("-o", "--output", default=None)
  ap.add_argument("--device", default="cuda:0" if torch.cuda.is_available() else "cpu")
  args = ap.parse_args()

  agent_cfg = load_rl_cfg(args.task)
  log_root = pathlib.Path("logs/rsl_rl") / agent_cfg.experiment_name
  run_dir = (
    pathlib.Path(args.run)
    if args.run
    else max(log_root.glob("*/"), key=lambda p: p.stat().st_mtime)
  )
  onnx_path = run_dir / "policy.onnx"
  if not onnx_path.exists():
    raise FileNotFoundError(f"No policy.onnx in {run_dir}")

  # --- policy graph -------------------------------------------------------
  model = onnx.load(str(onnx_path))
  meta = {p.key: p.value for p in model.metadata_props}
  in_dim = model.graph.input[0].type.tensor_type.shape.dim[1].dim_value
  out_dim = model.graph.output[0].type.tensor_type.shape.dim[1].dim_value
  # A leading Sub+Div means the observation normalizer is baked into the graph.
  ops = [n.op_type for n in model.graph.node]
  norm_baked = ops[:2] == ["Sub", "Div"]

  # --- environment (source of truth for everything else) ------------------
  env_cfg = load_env_cfg(args.task)
  env_cfg.scene.num_envs = 2
  env = ManagerBasedRlEnv(cfg=env_cfg, device=args.device)
  env.reset()
  robot = env.scene["robot"]
  mj = env.sim.mj_model
  action = env.action_manager.get_term("joint_pos")

  joint_names = list(robot.joint_names)
  scale = action._scale[0].cpu().tolist()
  offset = action._offset[0].cpu().tolist()

  # PD gains / effort limits, read back out of the compiled MuJoCo model in the
  # same order as joint_names (this is what get_base_metadata does for the ONNX).
  name_to_act = {a.target.split("/")[-1]: a.id for a in robot.spec.actuators}
  joints = []
  for i, jn in enumerate(joint_names):
    aid = name_to_act[jn]
    jid = mujoco.mj_name2id(mj, mujoco.mjtObj.mjOBJ_JOINT, f"robot/{jn}")
    lo, hi = (float(v) for v in mj.jnt_range[jid])
    dof = mj.jnt_dofadr[jid]
    joints.append(
      {
        "index": i,
        "name": jn,
        "default_pos": _f(offset[i]),
        "kp": _f(mj.actuator_gainprm[aid, 0]),
        "kd": _f(-mj.actuator_biasprm[aid, 2]),
        "effort_limit": _f(mj.actuator_forcerange[aid, 1]),
        "action_scale": _f(scale[i]),
        "armature": _f(mj.dof_armature[dof]),
        "frictionloss": _f(mj.dof_frictionloss[dof]),
        "lower_limit": _f(lo),
        "upper_limit": _f(hi),
      }
    )

  # --- observation layout -------------------------------------------------
  om = env.observation_manager
  terms = om.active_terms["actor"]
  dims = [int(d[0]) for d in om.group_obs_term_dim["actor"]]
  term_cfgs = env_cfg.observations["actor"].terms
  obs, start = [], 0
  for name, dim in zip(terms, dims):
    tc = term_cfgs[name]
    obs.append(
      {
        "name": name,
        "dim": dim,
        "slice": [start, start + dim],
        "scale": _f(tc.scale) if isinstance(tc.scale, (int, float)) else 1.0,
      }
    )
    start += dim

  twist = env_cfg.commands["twist"]
  phase_period = term_cfgs["phase"].params["period"]

  doc = {
    "_generated": {
      "by": "scripts/export_deploy_config.py",
      "at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
      "task": args.task,
      "run_dir": str(run_dir),
      "warning": "Auto-generated. Regenerate after retraining; do not hand-edit.",
    },
    "policy": {
      "onnx": str(onnx_path),
      "input_name": model.graph.input[0].name,
      "output_name": model.graph.output[0].name,
      "obs_dim": in_dim,
      "action_dim": out_dim,
      # If true the Sub/Div normalizer is INSIDE the graph: feed raw observations.
      "obs_normalization_baked_in": norm_baked,
      "trained_obs_order": meta.get("observation_names", ""),
    },
    "control": {
      "control_dt": _f(env.step_dt),
      "control_hz": _f(1.0 / env.step_dt),
      "physics_dt": _f(env.physics_dt),
      "decimation": int(env_cfg.decimation),
    },
    "joints": joints,
    "observation": {
      "total_dim": sum(dims),
      "note": (
        "Concatenate in this exact order. joint_pos is (q - default_pos); "
        "joint_vel is raw qd. 'actions' is the previous RAW network output, "
        "before action_scale is applied."
      ),
      "terms": obs,
    },
    "action": {
      "formula": "q_target[i] = default_pos[i] + action_scale[i] * action[i]",
      "note": "Network output is unbounded; clamp q_target to joint limits on hardware.",
    },
    "gait": {
      "phase_period_s": _f(phase_period),
      "note": (
        "phase = (t mod period)/period; obs = [sin(2*pi*phase), cos(2*pi*phase)]. "
        "Both entries MUST be zeroed when ||command|| < zero_command_threshold, "
        "or the robot keeps stepping in place when told to stop."
      ),
      "zero_command_threshold": 0.1,
    },
    "command": {
      "note": "Gamepad axes map here. Ranges are what the policy was trained on.",
      "lin_vel_x": [_f(v) for v in twist.ranges.lin_vel_x],
      "lin_vel_y": [_f(v) for v in twist.ranges.lin_vel_y],
      "ang_vel_z": [_f(v) for v in twist.ranges.ang_vel_z],
      "deadzone": 0.1,
    },
  }

  out = pathlib.Path(args.output) if args.output else run_dir / "deploy_config.yaml"
  out.parent.mkdir(parents=True, exist_ok=True)
  with out.open("w") as f:
    yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False, width=100)

  env.close()
  print(f"wrote {out}")
  print(f"  policy      : {onnx_path.name}  ({in_dim} -> {out_dim})")
  print(f"  normalizer  : {'baked into graph' if norm_baked else 'EXTERNAL - apply in controller'}")
  print(f"  joints      : {len(joints)}")
  print(f"  obs terms   : {len(obs)} totalling {sum(dims)}")
  print(f"  control     : {1 / env.step_dt:.0f} Hz")


if __name__ == "__main__":
  main()
