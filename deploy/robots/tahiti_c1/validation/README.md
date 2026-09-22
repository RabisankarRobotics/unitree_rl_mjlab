# Tahiti C1 — policy reference vectors

Proof that the `policy.mnn` running on the robot is numerically the same policy
as the `policy.onnx` that was validated in MuJoCo.

## Files

| file | what it is |
|---|---|
| `reference_vectors.json` | 23 (observation → action) pairs, full precision, plus model SHA-256s and tolerances |
| `reference_vectors.csv` | same data flat — `name, obs0..obs46, act0..act11` — for a C++ harness |
| `manifest.txt` | human-readable case list |
| `verify_policy.py` | runs the vectors through a model and reports the error |

Regenerate after any retrain/re-export:

```bash
~/anaconda3/bin/python scripts/make_reference_vectors.py
```

(Uses `onnxruntime`, which is not in the `mjlab` conda env — hence the explicit
interpreter. `scripts/mujoco_deploy.py` uses the same runtime.)

## Running it on the robot

Copy this whole directory over, then:

```bash
python3 verify_policy.py --model /path/to/config/models/policy_mjlab.mnn
```

Exit code 0 = pass. Tolerance is `1e-4` absolute, hard failure above `1e-3`.

No MNN Python bindings on the robot? Use `reference_vectors.csv` from a small
C++ harness: feed `obs0..obs46` **raw** and compare your output against
`act0..act11`.

## Two things that are easy to get wrong

**Feed observations RAW.** The graph carries its own normalizer — the first two
nodes are `Sub` then `Div`, with `obs_normalizer._mean` baked in, and the MNN
conversion preserved them. Normalizing again in the service is the classic
deployment failure and shows up as instant violent twitching.

**`expected_action` is the RAW network output** — before `action_scale` and
before adding `default_joint_pos`. Compare at that stage, not at the joint
target.

## Observation layout (47 floats)

```
[ 0: 3) ang_vel        IMU gyro, body frame, rad/s
[ 3: 6) proj_gravity   gravity direction in body frame, unit
[ 6: 9) vel_cmd        vx, vy, wz
[ 9:11) gait_phase     sin(2*pi*phase), cos(2*pi*phase)  -- BOTH zero when ||vel_cmd|| < 0.1
[11:23) joint_pos      q - default_joint_pos, rad
[23:35) joint_vel      raw qd, rad/s
[35:47) prev_actions   previous RAW network output
```

Joint order is left leg then right leg:
`hip_yaw, hip_pitch, hip_roll, knee, ankle_pitch, ankle_roll` ×2.

## Diagnostic cases

Three cases are deliberately about the known `gait_phase` bug rather than the
conversion:

- `DIAG_zero_cmd_phase_running` — zero command with phase **not** zeroed, i.e.
  what the service currently feeds the network when you release the stick.
- `DIAG_cmd_just_below_thresh` / `DIAG_cmd_just_above_thresh` — either side of
  the 0.1 threshold; phase must be zeroed below, live above.

Comparing `DIAG_zero_cmd_phase_running` against `stand_home` measures the bug:
up to **6.85° of joint-target error at standstill** (worst joint `right_knee`;
mean 3.66° across the twelve). That is the posture error driving the
step-in-place behaviour.

## Scope

This validates **the model only**. It does not check the observation pipeline,
joint ordering, PD gains, IMU convention, or the gait-phase zeroing. Those are
separate checks against the robot's `hardware.yaml`, `policy_service` source and
`robot_controller.cpp`.
