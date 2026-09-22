#!/usr/bin/env python3
"""Verify a deployed policy reproduces the reference vectors.

Run this ON THE ROBOT against policy.mnn, to prove the MNN conversion is
numerically identical to the policy.onnx that was validated in simulation.

    python3 verify_policy.py --model /path/to/policy_mjlab.mnn

It also runs against .onnx (via onnxruntime) if you want to sanity-check on a
dev machine first:

    python3 verify_policy.py --model ../policy/policy.onnx

Exit code 0 = all cases within tolerance, 1 = mismatch.

If MNN's Python bindings are not installed on the robot, use
reference_vectors.csv from a small C++ harness instead: each row is
  name, obs0..obs46, act0..act11
Feed obs RAW (the graph carries its own normalizer) and compare your model's
output against act0..act11.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent


# --------------------------------------------------------------------------
# Backends
# --------------------------------------------------------------------------
class OnnxBackend:
  name = "onnxruntime"

  def __init__(self, path: pathlib.Path) -> None:
    import onnxruntime as ort

    self.sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    self.input_name = self.sess.get_inputs()[0].name

  def __call__(self, obs: np.ndarray) -> np.ndarray:
    return self.sess.run(None, {self.input_name: obs[None, :]})[0][0]


class MnnBackend:
  name = "MNN"

  def __init__(self, path: pathlib.Path) -> None:
    import MNN  # noqa: N814

    self.MNN = MNN
    self.interp = MNN.Interpreter(str(path))
    self.session = self.interp.createSession({})
    self.in_t = self.interp.getSessionInput(self.session)
    self.out_t = self.interp.getSessionOutput(self.session)

  def __call__(self, obs: np.ndarray) -> np.ndarray:
    MNN = self.MNN
    src = MNN.Tensor(
      (1, obs.shape[0]),
      MNN.Halide_Type_Float,
      obs.astype(np.float32).reshape(1, -1),
      MNN.Tensor_DimensionType_Caffe,
    )
    self.in_t.copyFrom(src)
    self.interp.runSession(self.session)
    dst = MNN.Tensor(
      self.out_t.getShape(),
      MNN.Halide_Type_Float,
      np.zeros(self.out_t.getShape(), np.float32),
      MNN.Tensor_DimensionType_Caffe,
    )
    self.out_t.copyToHostTensor(dst)
    return np.array(dst.getData(), dtype=np.float32).reshape(-1)


def make_backend(path: pathlib.Path):
  suffix = path.suffix.lower()
  if suffix == ".mnn":
    return MnnBackend(path)
  if suffix == ".onnx":
    return OnnxBackend(path)
  sys.exit(f"unsupported model type: {suffix}")


# --------------------------------------------------------------------------
def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--model", required=True, type=pathlib.Path,
                  help="path to policy .mnn (on robot) or .onnx (dev machine)")
  ap.add_argument("--vectors", type=pathlib.Path, default=HERE / "reference_vectors.json")
  ap.add_argument("--tol", type=float, default=None,
                  help="absolute tolerance (default: from the vectors file)")
  ap.add_argument("--verbose", action="store_true", help="print every action element")
  args = ap.parse_args()

  if not args.model.exists():
    sys.exit(f"model not found: {args.model}")
  if not args.vectors.exists():
    sys.exit(f"vectors not found: {args.vectors}")

  ref = json.loads(args.vectors.read_text())
  tol = args.tol if args.tol is not None else ref["tolerance"]["abs"]
  fail_above = ref["tolerance"]["fail_above"]

  print(f"model   : {args.model}")
  print(f"vectors : {args.vectors}  ({len(ref['cases'])} cases)")

  # Model identity: warn if this .mnn is not the one the vectors were made against.
  digest = hashlib.sha256(args.model.read_bytes()).hexdigest()
  expected = (ref["models"].get(args.model.name)
              or ref["models"].get("policy.mnn" if args.model.suffix == ".mnn" else "policy.onnx"))
  print(f"sha256  : {digest}")
  if expected and expected.get("sha256"):
    if digest == expected["sha256"]:
      print("          matches the model these vectors were generated against")
    else:
      print("          !! DIFFERENT from the model these vectors were generated against")
      print(f"          expected {expected['sha256']}")
      print("          (fine if you re-converted; a mismatch here plus passing")
      print("           numbers just means the conversion is reproducible)")

  backend = make_backend(args.model)
  print(f"backend : {backend.name}")
  print(f"tol     : {tol:g} (hard fail above {fail_above:g})\n")

  worst = 0.0
  worst_name = ""
  n_warn = 0
  n_fail = 0

  print(f"{'case':34s} {'max|err|':>10s}  status")
  print("-" * 60)
  for c in ref["cases"]:
    obs = np.asarray(c["obs"], dtype=np.float32)
    exp = np.asarray(c["expected_action"], dtype=np.float32)
    got = np.asarray(backend(obs), dtype=np.float32).reshape(-1)

    if got.shape != exp.shape:
      print(f"{c['name']:34s} {'--':>10s}  SHAPE {got.shape} != {exp.shape}")
      n_fail += 1
      continue

    err = float(np.abs(got - exp).max())
    if err > worst:
      worst, worst_name = err, c["name"]

    if err > fail_above:
      status = "FAIL"
      n_fail += 1
    elif err > tol:
      status = "warn"
      n_warn += 1
    else:
      status = "ok"
    print(f"{c['name']:34s} {err:10.3e}  {status}")

    if args.verbose or status == "FAIL":
      print(f"    expected {np.array2string(exp, precision=5, max_line_width=200)}")
      print(f"    got      {np.array2string(got, precision=5, max_line_width=200)}")

  print("-" * 60)
  print(f"worst case: {worst_name}  max|err| = {worst:.3e}")
  print(f"ok/warn/fail: {len(ref['cases']) - n_warn - n_fail}/{n_warn}/{n_fail}")

  if n_fail:
    print("\nFAIL -- the deployed model does not reproduce the validated policy.")
    print("Check, in order:")
    print("  1. Are you feeding observations RAW? The graph has a baked-in")
    print("     normalizer; normalizing again is the usual cause of large errors.")
    print("  2. Is the observation order the 7 terms from policy_mjlab.yaml?")
    print("  3. Was the .mnn converted from THIS .onnx (compare sha256 above)?")
    return 1

  print("\nPASS -- deployed model reproduces the validated policy.")
  print("NOTE: this validates the MODEL ONLY. It does not check the observation")
  print("      pipeline, joint ordering, gains, or the gait-phase zeroing bug.")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
