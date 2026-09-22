#!/usr/bin/env python3
"""Generate reference (observation -> action) vectors for the Tahiti C1 policy.

The vectors are produced by running ``policy.onnx`` through onnxruntime, which is
the same runtime ``scripts/mujoco_deploy.py`` used for the sim validation. They
exist so the ``policy.mnn`` running on the robot can be proven numerically
identical to the policy that was actually tested in simulation.

Usage:
    python scripts/make_reference_vectors.py

Writes into ``deploy/robots/tahiti_c1/validation/``:
    reference_vectors.json   full precision, with metadata + model hashes
    reference_vectors.csv    flat obs+action rows, for a C++ harness
    manifest.txt            human-readable summary

Re-run this after any retrain/re-export; the model hashes in the JSON are what
tie a vector set to a specific policy.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import pathlib
import sys

import numpy as np

try:
  import onnxruntime as ort
except ImportError:
  sys.exit(
    "onnxruntime is required.\n"
    "It is not in the `mjlab` env; run this with an interpreter that has it, e.g.\n"
    "  ~/anaconda3/bin/python scripts/make_reference_vectors.py"
  )

ROOT = pathlib.Path(__file__).resolve().parent.parent
ROBOT_DIR = ROOT / "deploy" / "robots" / "tahiti_c1"
ONNX_PATH = ROBOT_DIR / "policy" / "policy.onnx"
MNN_PATH = ROBOT_DIR / "policy" / "policy.mnn"
OUT_DIR = ROBOT_DIR / "validation"

# Observation layout -- 47 floats. Must match policy_mjlab.yaml exactly.
SLICES = {
  "ang_vel": slice(0, 3),
  "proj_gravity": slice(3, 6),
  "vel_cmd": slice(6, 9),
  "gait_phase": slice(9, 11),
  "joint_pos": slice(11, 23),
  "joint_vel": slice(23, 35),
  "prev_actions": slice(35, 47),
}
N_OBS = 47
N_ACT = 12

# Training command ranges, from policy_mjlab.yaml.
CMD_RANGE = {"lin_vel_x": (-0.8, 1.2), "lin_vel_y": (-0.5, 0.5), "ang_vel_z": (-1.0, 1.0)}
ZERO_CMD_THRESH = 0.1  # matches mdp.phase / mujoco_deploy.py


def make_obs(
  ang_vel=(0.0, 0.0, 0.0),
  proj_gravity=(0.0, 0.0, -1.0),
  vel_cmd=(0.0, 0.0, 0.0),
  phase=0.0,
  joint_pos=None,
  joint_vel=None,
  prev_actions=None,
  force_phase=None,
) -> np.ndarray:
  """Assemble one 47-float observation.

  ``phase`` is the normalised gait phase in [0, 1). Following training
  (``mdp.phase``), both sin and cos are zeroed when the command is below
  ``ZERO_CMD_THRESH``. ``force_phase`` bypasses that zeroing so we can emit a
  deliberately-wrong diagnostic case.
  """
  obs = np.zeros(N_OBS, dtype=np.float32)
  obs[SLICES["ang_vel"]] = ang_vel
  obs[SLICES["proj_gravity"]] = proj_gravity
  obs[SLICES["vel_cmd"]] = vel_cmd

  if force_phase is not None:
    sc = force_phase
  elif np.linalg.norm(vel_cmd) < ZERO_CMD_THRESH:
    sc = (0.0, 0.0)
  else:
    sc = (math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase))
  obs[SLICES["gait_phase"]] = sc

  if joint_pos is not None:
    obs[SLICES["joint_pos"]] = joint_pos
  if joint_vel is not None:
    obs[SLICES["joint_vel"]] = joint_vel
  if prev_actions is not None:
    obs[SLICES["prev_actions"]] = prev_actions
  return obs


def build_cases() -> list[dict]:
  rng = np.random.default_rng(20260919)  # fixed -> reproducible
  cases: list[dict] = []

  def add(name, obs, note):
    cases.append({"name": name, "obs": obs, "note": note})

  # --- trivial smoke tests -------------------------------------------------
  add("all_zeros", np.zeros(N_OBS, np.float32),
      "Degenerate input. Easiest possible check that plumbing and byte order agree.")
  add("all_ones", np.ones(N_OBS, np.float32),
      "Degenerate input, non-zero. Catches a normalizer that was dropped in conversion.")

  # --- the single most important case --------------------------------------
  add("stand_home", make_obs(),
      "Exact home pose, upright, zero command, phase zeroed. This is the state at "
      "the instant you enable the policy. Compare this one first.")

  # --- standing, perturbed --------------------------------------------------
  add("stand_tilt_fwd", make_obs(proj_gravity=(0.17, 0.0, -0.985), ang_vel=(0.0, 0.2, 0.0)),
      "Standing, pitched forward ~10 deg. Checks proj_gravity sign convention.")
  add("stand_tilt_left", make_obs(proj_gravity=(0.0, -0.17, -0.985), ang_vel=(0.2, 0.0, 0.0)),
      "Standing, rolled left ~10 deg.")

  # --- forward walk across the gait cycle ----------------------------------
  for p in (0.0, 0.25, 0.5, 0.75):
    add(f"walk_fwd_phase{p:.2f}".replace(".", "p"),
        make_obs(vel_cmd=(0.8, 0.0, 0.0), phase=p,
                 ang_vel=(0.0, 0.05, 0.0),
                 joint_pos=rng.uniform(-0.25, 0.25, N_ACT),
                 joint_vel=rng.uniform(-2.5, 2.5, N_ACT),
                 prev_actions=rng.uniform(-1.0, 1.0, N_ACT)),
        f"Forward 0.8 m/s at gait phase {p:.2f}. Exercises the sin/cos encoding.")

  # --- command corners ------------------------------------------------------
  add("cmd_fwd_max", make_obs(vel_cmd=(CMD_RANGE["lin_vel_x"][1], 0.0, 0.0), phase=0.3,
                              joint_pos=rng.uniform(-0.3, 0.3, N_ACT),
                              joint_vel=rng.uniform(-3.0, 3.0, N_ACT)),
      "Max trained forward command (1.2 m/s).")
  add("cmd_back_max", make_obs(vel_cmd=(CMD_RANGE["lin_vel_x"][0], 0.0, 0.0), phase=0.6,
                               joint_pos=rng.uniform(-0.3, 0.3, N_ACT),
                               joint_vel=rng.uniform(-3.0, 3.0, N_ACT)),
      "Max trained backward command (-0.8 m/s).")
  add("cmd_strafe_left", make_obs(vel_cmd=(0.0, 0.5, 0.0), phase=0.1,
                                  joint_pos=rng.uniform(-0.2, 0.2, N_ACT)),
      "Max lateral left. Note lateral tracking measured ~23% in sim.")
  add("cmd_strafe_right", make_obs(vel_cmd=(0.0, -0.5, 0.0), phase=0.9,
                                   joint_pos=rng.uniform(-0.2, 0.2, N_ACT)),
      "Max lateral right.")
  add("cmd_yaw_left", make_obs(vel_cmd=(0.0, 0.0, 1.0), phase=0.45,
                               ang_vel=(0.0, 0.0, 0.8),
                               joint_pos=rng.uniform(-0.2, 0.2, N_ACT)),
      "Max yaw left.")
  add("cmd_yaw_right", make_obs(vel_cmd=(0.0, 0.0, -1.0), phase=0.55,
                                ang_vel=(0.0, 0.0, -0.8),
                                joint_pos=rng.uniform(-0.2, 0.2, N_ACT)),
      "Max yaw right.")
  add("cmd_combined", make_obs(vel_cmd=(0.6, 0.2, 0.4), phase=0.2,
                               ang_vel=(0.05, 0.1, 0.35),
                               proj_gravity=(0.05, -0.03, -0.998),
                               joint_pos=rng.uniform(-0.25, 0.25, N_ACT),
                               joint_vel=rng.uniform(-2.0, 2.0, N_ACT),
                               prev_actions=rng.uniform(-1.0, 1.0, N_ACT)),
      "Simultaneous forward + lateral + yaw, mid-stride.")

  # --- diagnostics for the known blockers -----------------------------------
  add("DIAG_zero_cmd_phase_running",
      make_obs(vel_cmd=(0.0, 0.0, 0.0), force_phase=(0.707107, 0.707107)),
      "DIAGNOSTIC for blocker 1. Zero command but phase NOT zeroed -- what the "
      "robot currently feeds the net when you release the stick. Its action should "
      "differ noticeably from `stand_home`; that difference is the bug's magnitude.")
  add("DIAG_cmd_just_below_thresh", make_obs(vel_cmd=(0.05, 0.0, 0.0), phase=0.5),
      "Command just under the 0.1 zero threshold -- phase must be zeroed here.")
  add("DIAG_cmd_just_above_thresh", make_obs(vel_cmd=(0.15, 0.0, 0.0), phase=0.5),
      "Command just over the 0.1 zero threshold -- phase must be live here.")

  # --- random plausible states ---------------------------------------------
  for k in range(4):
    add(f"random_{k}",
        make_obs(ang_vel=rng.uniform(-1.5, 1.5, 3),
                 proj_gravity=_unit(rng.normal(0, 1, 3) * np.array([0.2, 0.2, 1.0]) - np.array([0, 0, 1])),
                 vel_cmd=(rng.uniform(*CMD_RANGE["lin_vel_x"]),
                          rng.uniform(*CMD_RANGE["lin_vel_y"]),
                          rng.uniform(*CMD_RANGE["ang_vel_z"])),
                 phase=rng.uniform(0, 1),
                 joint_pos=rng.uniform(-0.35, 0.35, N_ACT),
                 joint_vel=rng.uniform(-4.0, 4.0, N_ACT),
                 prev_actions=rng.uniform(-1.5, 1.5, N_ACT)),
        "Randomised but physically plausible state (seed 20260919).")

  return cases


def _unit(v: np.ndarray) -> np.ndarray:
  n = np.linalg.norm(v)
  return (v / n) if n > 1e-9 else np.array([0.0, 0.0, -1.0])


def sha256(path: pathlib.Path) -> str:
  h = hashlib.sha256()
  h.update(path.read_bytes())
  return h.hexdigest()


def main() -> None:
  if not ONNX_PATH.exists():
    sys.exit(f"missing {ONNX_PATH}")

  sess = ort.InferenceSession(str(ONNX_PATH), providers=["CPUExecutionProvider"])
  in_name = sess.get_inputs()[0].name

  cases = build_cases()
  for c in cases:
    obs = np.asarray(c["obs"], dtype=np.float32)
    assert obs.shape == (N_OBS,), obs.shape
    c["obs"] = obs
    c["action"] = sess.run(None, {in_name: obs[None, :]})[0][0].astype(np.float32)

  OUT_DIR.mkdir(parents=True, exist_ok=True)

  payload = {
    "description": "Reference observation->action pairs for the Tahiti C1 mjlab velocity policy.",
    "generated_by": "scripts/make_reference_vectors.py",
    "onnxruntime_version": ort.__version__,
    "models": {
      "policy.onnx": {"sha256": sha256(ONNX_PATH), "bytes": ONNX_PATH.stat().st_size},
      "policy.mnn": (
        {"sha256": sha256(MNN_PATH), "bytes": MNN_PATH.stat().st_size}
        if MNN_PATH.exists() else None
      ),
    },
    "obs_dim": N_OBS,
    "action_dim": N_ACT,
    "obs_layout": {k: [v.start, v.stop] for k, v in SLICES.items()},
    "notes": [
      "Feed obs RAW. The graph carries its own Sub/Div normalizer.",
      "Actions are the RAW network output, before action_scale and before "
      "adding default_joint_pos.",
      "Tolerance: MNN fp32 should agree to <=1e-4 absolute. Anything above "
      "1e-3 means the conversion or the input plumbing is wrong.",
    ],
    "tolerance": {"abs": 1e-4, "fail_above": 1e-3},
    "cases": [
      {
        "name": c["name"],
        "note": c["note"],
        "obs": [float(x) for x in c["obs"]],
        "expected_action": [float(x) for x in c["action"]],
      }
      for c in cases
    ],
  }

  json_path = OUT_DIR / "reference_vectors.json"
  json_path.write_text(json.dumps(payload, indent=2) + "\n")

  csv_path = OUT_DIR / "reference_vectors.csv"
  with csv_path.open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["name"] + [f"obs{i}" for i in range(N_OBS)]
               + [f"act{i}" for i in range(N_ACT)])
    for c in cases:
      w.writerow([c["name"]] + [f"{x:.8g}" for x in c["obs"]]
                 + [f"{x:.8g}" for x in c["action"]])

  lines = [
    "Tahiti C1 policy reference vectors",
    f"  onnx sha256 : {payload['models']['policy.onnx']['sha256']}",
    f"  mnn  sha256 : {payload['models']['policy.mnn']['sha256'] if payload['models']['policy.mnn'] else 'MISSING'}",
    f"  cases       : {len(cases)}",
    f"  obs dim     : {N_OBS}   action dim: {N_ACT}",
    "",
    f"{'case':34s} {'|action|_max':>12s}  note",
  ]
  for c in cases:
    lines.append(f"{c['name']:34s} {float(np.abs(c['action']).max()):12.5f}  {c['note'][:60]}")
  (OUT_DIR / "manifest.txt").write_text("\n".join(lines) + "\n")

  print("\n".join(lines))
  print(f"\nwrote {json_path}")
  print(f"wrote {csv_path}")
  print(f"wrote {OUT_DIR / 'manifest.txt'}")


if __name__ == "__main__":
  main()
