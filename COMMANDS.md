# Command Reference

Working command list for the whole pipeline: **train → play → sim2sim → hardware**.

Every command here was run or read out of the source on 2026-09-20. Flags that do
not exist are not listed. Gotchas are marked ⚠️ and are things that have actually
bitten, not hypotheticals.

**Contents**

1. [Environment](#1-environment)
2. [Sanity checks](#2-sanity-checks)
3. [Training](#3-training)
4. [Monitoring a run](#4-monitoring-a-run)
5. [Play — visualise in mjlab](#5-play--visualise-in-mjlab)
6. [Sim2sim — test the deployment path](#6-sim2sim--test-the-deployment-path)
7. [Export and validate a policy](#7-export-and-validate-a-policy)
8. [Robot / asset pipeline](#8-robot--asset-pipeline)
9. [Motion imitation](#9-motion-imitation)
10. [Unitree C++ deployment (G1 etc.)](#10-unitree-c-deployment-g1-etc)
11. [Robot side — robo_control](#11-robot-side--robo_control)
12. [Sync between laptop and server](#12-sync-between-laptop-and-server)
13. [Task IDs](#13-task-ids)
14. [Gotcha index](#14-gotcha-index)

---

## 1. Environment

```bash
conda activate mjlab
cd ~/unitree_rl_mjlab
```

### Building the env from scratch

```bash
conda create -n mjlab python=3.11 -y
conda activate mjlab
pip install -e .

# ⚠️ REQUIRED -- setup.py's pins are unbounded and pip installs versions that break
pip install "mujoco==3.5.0" "warp-lang==1.12.0" scipy

# only needed for scripts/mujoco_deploy.py (see section 6)
pip install onnxruntime
```

Only needed for the **C++ deployment** build, not for training:

```bash
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```

### Verify versions

```bash
python -c "
import importlib.metadata as md
for p in ['mjlab','mujoco-warp','mujoco','warp-lang','rsl-rl-lib']: print(p, md.version(p))
import torch; print('cuda:', torch.cuda.is_available())"
```

Expected: `mjlab 1.2.0`, `mujoco-warp 3.5.0`, `mujoco 3.5.0`, `warp-lang 1.12.0`,
`rsl-rl-lib 5.0.1`, `cuda: True`.

Ground truth for pins: `cd ~/mjlab && git show v1.2.0:uv.lock`

---

## 2. Sanity checks

```bash
# all registered tasks (31 as of now)
python scripts/list_envs.py

# robot model + meshes + actuators compile?
python -c "
from mjlab.entity import Entity
from src.assets.robots import get_tahiti_c1_robot_cfg
m = Entity(get_tahiti_c1_robot_cfg()).spec.compile()
print('bodies', m.nbody, 'nv', m.nv, 'actuators', m.nu)"

# same for G1 -> expect bodies 31, nv 35, actuators 29
python -c "
from mjlab.entity import Entity
from src.assets.robots.unitree_g1.g1_constants import get_g1_robot_cfg
m = Entity(get_g1_robot_cfg()).spec.compile()
print('bodies', m.nbody, 'nv', m.nv, 'actuators', m.nu)"

# actuator gain audit
python scripts/check_actuators.py --robot tahiti_c1
python scripts/check_actuators.py --robot g1

# open a robot in the viewer (each *_constants.py has a __main__ block)
python src/assets/robots/unitree_g1/g1_constants.py
```

---

## 3. Training

### Minimum viable

```bash
python scripts/train.py Tahiti-C1-Flat \
  --env.scene.num-envs=4096 \
  --agent.logger=tensorboard
```

⚠️ **Both flags matter.** `num_envs` defaults to **1** (`velocity_env_cfg.py:401`), and
`logger` defaults to **wandb** (`mjlab/rl/config.py:105`) which blocks on a login prompt.

### Common variations

```bash
# name the run (shows up in the log dir)
--agent.run-name=c1_flat_v1

# short smoke test
--agent.max-iterations=10

# resume newest checkpoint automatically
--agent.resume=True

# resume a specific run / checkpoint (regexes)
--agent.resume=True --agent.load-run=".*2026-09-19.*" --agent.load-checkpoint="model_3000.pt"

# reproducibility
--agent.seed=42

# checkpoint frequency (default 100)
--agent.save-interval=50

# record videos during training
--video --video-length=200 --video-interval=2000

# multi-GPU
--gpu-ids 0 1
--gpu-ids all

# debug NaNs (slow)
--enable-nan-guard
```

### PPO hyperparameters

```bash
--agent.algorithm.learning-rate=1.0e-3
--agent.algorithm.entropy-coef=0.01
--agent.algorithm.gamma=0.99
--agent.algorithm.lam=0.95
--agent.algorithm.clip-param=0.2
--agent.algorithm.desired-kl=0.01
--agent.algorithm.num-learning-epochs=5
--agent.algorithm.num-mini-batches=4
--agent.num-steps-per-env=24
--agent.max-iterations=10001
```

⚠️ **Scaling `num_envs` does not scale learning.** Gradient updates per iteration is
`num_learning_epochs x num_mini_batches` = **20**, independent of `num_envs`. If you
raise envs 4x, raise `num_mini_batches` 4x to keep the minibatch size and recover the
update count:

```bash
python scripts/train.py Tahiti-C1-Flat \
  --env.scene.num-envs=16384 \
  --agent.algorithm.num-mini-batches=16 \
  --agent.logger=tensorboard
```

### Network architecture

```bash
--agent.actor.hidden-dims 512 256 128
--agent.critic.hidden-dims 512 256 128
--agent.actor.activation=elu
--agent.actor.obs-normalization=True
```

### Env overrides from the CLI

```bash
--env.scene.num-envs=4096
--env.decimation=4
--env.episode-length-s=20.0
--env.sim.mujoco.timestep=0.005
--env.sim.mujoco.gravity "0 0 -9.81"
--env.seed=42
```

Command ranges (what the policy is trained to track):

```bash
--env.commands.twist.ranges.lin-vel-x "-0.8 1.2"
--env.commands.twist.ranges.lin-vel-y "-0.5 0.5"
--env.commands.twist.ranges.ang-vel-z "-1.0 1.0"
--env.commands.twist.resampling-time-range "5.0 10.0"
--env.commands.twist.rel-standing-envs=0.02
```

Action scale and actuator gains are reachable too — useful for a quick gain sweep
without editing `<robot>_constants.py`:

```bash
--env.actions.joint-pos.scale=0.35
--env.scene.entities.robot.articulation.actuators.0.base-cfg.stiffness=228.571
--env.scene.entities.robot.articulation.actuators.0.base-cfg.damping=24.019
```

Full list (3750 lines): `python scripts/train.py <Task> --help`

> tahiti_c1 uses `DelayedActuatorCfg`, so it also exposes action-latency knobs
> (`...actuators.0.delay-min-lag`, `.delay-max-lag`, `.delay-update-period`) that the
> Unitree robots' `BuiltinPositionActuatorCfg` does not have.

### Picking `num_envs`

Measured on an RTX 2000 Ada Laptop (8 GB):

| envs | steps/s | iter time | peak VRAM |
|---|---|---|---|
| 1024 | 26,301 | 0.93 s | — |
| **4096** | **32,181** | 3.05 s | 2,224 MiB |
| 8192 | 27,425 | 7.17 s | 5,738 MiB |

Throughput **peaks then falls** — it is not a VRAM limit. Re-measure on each machine:

```bash
for N in 4096 8192 16384 32768; do
  echo "=== $N ==="
  python scripts/train.py Tahiti-C1-Flat --env.scene.num-envs=$N \
    --agent.logger=tensorboard --agent.max-iterations=5 2>&1 \
    | grep -E "Steps per second|Iteration time" | tail -4
done
```

Take the largest N still climbing; stop when the gain drops below ~15%. Ignore
iteration 0 (kernel compile + warmup).

### Long runs over SSH

```bash
tmux new -s train
conda activate mjlab && cd ~/unitree_rl_mjlab
python scripts/train.py Tahiti-C1-Flat --env.scene.num-envs=4096 --agent.logger=tensorboard
# detach: Ctrl-B then D     reattach: tmux attach -t train
```

**Outputs:** `logs/rsl_rl/<experiment_name>/<timestamp>/`
containing `model_*.pt`, `policy.onnx`, `events.out.tfevents.*`,
`params/{env,agent}.yaml`, and `git/*.diff`.

⚠️ First run on a new machine compiles Warp CUDA kernels — **~35 s of
`Module ... (compiled)` output before anything happens.** Not a hang. Cached in
`~/.cache/warp/1.12.0`.

---

## 4. Monitoring a run

```bash
tensorboard --logdir logs/rsl_rl/tahiti_c1_velocity
# remote: tensorboard --logdir ... --port 6006
#         then from the laptop: ssh -L 6006:localhost:6006 user@server
```

What to watch:

| Metric | Healthy trend |
|---|---|
| `Episode_Termination/fell_over` | falls toward 0 (untrained ≈ 15) |
| `Mean episode length` | climbs toward 1000 (20 s x 50 Hz) |
| `Metrics/twist/error_vel_xy` | flattens, well under 0.1 |
| `Mean reward` | flattens (untrained ≈ -9.7) |

Checkpoints land every `save_interval` iterations, so `Ctrl-C` when the curves flatten
— `max_iterations` is only an upper bound.

```bash
nvidia-smi --query-gpu=memory.used,memory.total,utilization.gpu --format=csv -l 5
```

---

## 5. Play — visualise in mjlab

```bash
python scripts/play.py Tahiti-C1-Flat \
  --checkpoint_file logs/rsl_rl/tahiti_c1_velocity/<timestamp>/model_3000.pt \
  --num_envs 4
```

⚠️ **`--checkpoint_file` is mandatory.** Without it `play.py:94` touches
`cfg.wandb_run_path`, which does not exist on `PlayConfig` → `AttributeError`.

```bash
--viewer native     # desktop MuJoCo window (needs DISPLAY)
--viewer viser      # browser at http://localhost:8080
--viewer auto       # native if DISPLAY set, else viser (default)

--num_envs 1
--device cpu
--no_terminations                       # never reset; good for inspecting motion
--agent zero                            # zero actions, no checkpoint needed
--agent random                          # random actions
--video --video_length 500              # writes to <log_dir>/videos/play/
--video_height 720 --video_width 1280
--camera 0
```

### Driving the robot

⚠️ **The velocity joystick exists only in the viser viewer** (`velocity_command.py:113`
builds it with `server.gui.add_slider`). The native viewer has no command control.

In viser, open the **Twist** folder:

| Control | Note |
|---|---|
| **`Enable` checkbox** | **defaults OFF** — until ticked the sliders do nothing |
| `lin_vel_x` / `lin_vel_y` / `ang_vel_z` | the command |
| `Max ...` sliders | widen the range beyond trained limits |
| `Zero` | snap all to 0 |

The joystick drives **only the currently selected env**; the others keep sampling
random commands.

### Native viewer keys (playback only)

| Key | Action | Key | Action |
|---|---|---|---|
| `Space` | pause / resume | `,` `.` | prev / next env |
| `Enter` | reset | `P` | toggle plots |
| `→` | single step | `R` | toggle debug viz |
| `-` `=` | slower / faster | `A` | show all envs |

### What play mode changes automatically

From `env_cfgs.py:145` — play is **easier** than training:
episode length → effectively infinite; observation noise **off**; `push_robot`
**removed**; curriculum cleared; `randomize_terrain` added on reset.

---

## 6. Sim2sim — test the deployment path

`scripts/mujoco_deploy.py` is the **deployment** path, not the training path. It imports
only the exported ONNX, the generated `deploy_config.yaml` and a plain MJCF scene —
**mjlab is never imported**. So if this walks and the firmware port does not, the bug is
in the port.

⚠️ Needs `onnxruntime`, which is **not** in the `mjlab` env:
`pip install onnxruntime` (or use `~/anaconda3/bin/python`).

```bash
# newest run, Xbox gamepad
python scripts/mujoco_deploy.py

# no gamepad -- hold a fixed command
python scripts/mujoco_deploy.py --keyboard --vx 0.5 --vy 0.0 --wz 0.0

# headless, timed, for scripted regression
python scripts/mujoco_deploy.py --headless --duration 10

# pick a specific policy / scene
python scripts/mujoco_deploy.py --config logs/rsl_rl/tahiti_c1_velocity/<ts>/deploy_config.yaml
python scripts/mujoco_deploy.py --scene src/assets/robots/tahiti_c1/xmls/scene_tahiti_c1.xml
```

### Gamepad (Xbox layout)

| Input | Action |
|---|---|
| Left stick Y / X | forward-back / strafe |
| Right stick X | turn |
| **A** | zero the command (stand) |
| **START** | reset to home pose |
| **BACK** | quit |
| **B** | manual push (when `--push-interval 0`) |

Axis indices `AX_LY, AX_LX, AX_RX = 1, 0, 3`; buttons `A,B,BACK,START = 0,1,6,7`.
No pad found → `RuntimeError`; use `--keyboard` or check `/dev/input/js*`.

### Robustness / push testing

```bash
# 'velocity' reproduces the training push exactly
python scripts/mujoco_deploy.py --push-mode velocity --push-interval 3.0 --push-scale 1.0

# 'force' is a physical shove the policy was never trained on
python scripts/mujoco_deploy.py --push-mode force --push-force 268 --push-duration 0.1

# manual only -- press gamepad B
python scripts/mujoco_deploy.py --push-mode velocity --push-interval 0

--seed 0        # reproducible pushes
```

### Control loop it reproduces

```
obs = [gyro(3), proj_gravity(3), cmd(3), phase(2), q-default(12), qd(12), prev_action(12)]  -> 47, RAW
action   = onnx(obs)                                   -> 12, unbounded
q_target = default_pos + action_scale * action
tau      = clamp(kp*(q_target-q) - kd*qd, +/-effort)
```

---

## 7. Export and validate a policy

```bash
# regenerate deploy_config.yaml from a run
python scripts/export_deploy_config.py --task Tahiti-C1-Flat
python scripts/export_deploy_config.py --task Tahiti-C1-Flat --run logs/rsl_rl/tahiti_c1_velocity/<ts>
python scripts/export_deploy_config.py --task Tahiti-C1-Flat -o /tmp/deploy_config.yaml

# build a standalone MJCF scene for the deployment harness
python scripts/make_scene.py --config logs/rsl_rl/tahiti_c1_velocity/<ts>/deploy_config.yaml
```

### Inspect ONNX metadata

```bash
python -c "
import onnx
m = onnx.load('deploy/robots/tahiti_c1/policy/policy.onnx')
for p in m.metadata_props: print(p.key, '=', p.value[:200])
print('in ', [(i.name,[d.dim_value for d in i.type.tensor_type.shape.dim]) for i in m.graph.input])
print('out', [(o.name,[d.dim_value for d in o.type.tensor_type.shape.dim]) for o in m.graph.output])"
```

### Reference vectors (ONNX ↔ MNN equivalence)

```bash
# regenerate after any retrain (needs onnxruntime)
~/anaconda3/bin/python scripts/make_reference_vectors.py

# check a model against them
python deploy/robots/tahiti_c1/validation/verify_policy.py \
  --model deploy/robots/tahiti_c1/policy/policy.onnx

# on the robot, against the MNN
python3 verify_policy.py --model /path/to/config/models/policy_mjlab.mnn
```

Tolerance `1e-4` absolute, hard fail above `1e-3`. Exit 0 = pass.
Last measured: MNN vs ONNX max error **3.7e-6** across 23 cases.

⚠️ Validates the **model only** — not the observation pipeline, joint ordering, gains,
or the gait-phase zeroing.

---

## 8. Robot / asset pipeline

```bash
# URDF -> MJCF
python scripts/urdf_to_mjcf.py
```
⚠️ **No argparse.** Any argument (including `--help`) is ignored and it just runs,
overwriting `src/assets/robots/tahiti_c1/xmls/tahiti_c1.xml`. Output is deterministic,
so a re-run on unchanged input produces no git diff.

```bash
# actuator / gain audit
python scripts/check_actuators.py --robot tahiti_c1

# terrain preview (opens a viewer; Ctrl-C to exit)
python scripts/visualize_terrain.py
```

Files to touch when adding a robot:

| Create | Path |
|---|---|
| constants | `src/assets/robots/<robot>/<robot>_constants.py` |
| package marker | `src/assets/robots/<robot>/__init__.py` |
| MJCF + meshes | `src/assets/robots/<robot>/xmls/` |
| env config | `src/tasks/velocity/config/<robot>/env_cfgs.py` |
| PPO config | `src/tasks/velocity/config/<robot>/rl_cfg.py` |
| **registration** | `src/tasks/velocity/config/<robot>/__init__.py` ← without this the task does not exist |

| Edit | Path |
|---|---|
| exports | `src/assets/robots/__init__.py` |
| shared rewards / DR / curriculum | `src/tasks/velocity/velocity_env_cfg.py` (affects **every** robot) |
| custom MDP terms | `src/tasks/velocity/mdp/{rewards,observations,terminations,curriculums}.py` |

Gain-tuning method: `doc/actuator_tuning.md`.

---

## 9. Motion imitation

```bash
python scripts/csv_to_npz.py \
  --robot g1 \
  --input-file src/assets/motions/g1/dance1_subject2.csv \
  --output-name dance1_subject2.npz \
  --input-fps 30 --output-fps 50 \
  --render True

python scripts/train.py Unitree-G1-Tracking-No-State-Estimation \
  --motion_file=src/assets/motions/g1/dance1_subject2.npz \
  --env.scene.num-envs=4096 --agent.logger=tensorboard

python scripts/play.py Unitree-G1-Tracking-No-State-Estimation \
  --motion_file=src/assets/motions/g1/dance1_subject2.npz \
  --checkpoint_file logs/rsl_rl/g1_tracking/<ts>/model_xx.pt
```

⚠️ Tracking tasks for **29-dof G1** load the robot from `mjlab.asset_zoo.robots`, not
`src/assets/robots` — editing this repo's `g1.xml` has no effect on them.

---

## 10. Unitree C++ deployment (G1 etc.)

Does **not** apply to tahiti_c1 — see section 11.

```bash
# install the policy (velocity policies have no .onnx.data; only mimic does)
mkdir -p deploy/robots/g1/config/policy/velocity/v1/{exported,params}
cp logs/rsl_rl/g1_velocity/<ts>/policy.onnx deploy/robots/g1/config/policy/velocity/v1/exported/
cp deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml \
   deploy/robots/g1/config/policy/velocity/v1/params/

# build
cd deploy/robots/g1 && mkdir build && cd build && cmake .. && make

# sim first
cd simulate && mkdir build && cd build && cmake .. && make -j8
./unitree_mujoco                       # gamepad required
cd deploy/robots/g1/build && ./g1_ctrl --network=lo

# real robot
./g1_ctrl --network=enp5s0
```

Version selection (`param.h:86`): if `policy_dir` has no `exported/`, it sorts the
subdirectories and takes the **last** one containing `exported/`.
⚠️ Lexicographic — `v0, v1, v10, v2` picks **v2**. Use `v01..v10` past nine.

FSM gamepad transitions (`config.yaml`): Passive→FixStand `LT+↑`;
FixStand→Velocity `RT+A`; **Velocity→Passive `LT+B`** (stop); Velocity→Mimic `RB+A`.

⚠️ **`bad_orientation()` is disabled** (`terminations.h:10` — body commented out,
`return false`). There is **no automatic fall protection**; `LT+B` is the only stop.

---

## 11. Robot side — robo_control

The tahiti_c1 controller is an **Orange Pi** (MAC OUI `C0742B` = Shenzhen Xunlong)
running Ubuntu 24.04 / OpenSSH 9.6p1, at **192.168.123.100**. Laptop side is a static
`192.168.123.123/24` on `eno1`.

```bash
ssh orangepi@192.168.123.100          # or root@; integrator may have changed it
```

`~/.ssh/config`:
```
Host c1
    HostName 192.168.123.100
    User orangepi
```

### Finding the robot if the IP changes

```bash
ip -br addr; ip -br link                 # confirm carrier on eno1
for i in $(seq 1 254); do (ping -c1 -W1 192.168.123.$i >/dev/null 2>&1 \
  && echo "192.168.123.$i") & done 2>/dev/null; wait 2>/dev/null
ip neigh show dev eno1
sudo nmap -sn 192.168.123.0/24           # if installed
```

### On the robot

```bash
uname -m                                 # expect aarch64
find / -name "services.yaml" -o -name "hardware.yaml" 2>/dev/null

source /opt/ros/<distro>/setup.bash
colcon build
colcon build --packages-select policy_service
source install/setup.bash

./scripts/install-deps.sh
./scripts/install-deps.sh --mnn

ros2 launch launch/robo.launch.py
ros2 run control_service control_service --ros-args -p config_path:=config/yaml/hardware.yaml

./scripts/can-start.sh
./scripts/can-stop.sh
./scripts/run-calibration.sh
./scripts/sync-systemd.sh
sudo systemctl status robo
```

### ROS 2 topics

| Topic | Publisher → Subscriber |
|---|---|
| `robo/joint_command` | policy_service → control_service |
| `robo/joint_state` | control_service → policy_service |
| `robo/imu_data` | imu_service → policy_service |
| `robo/joystick` | joystick_service → mode_service |
| `robo/robot_state` | mode_service → policy_service, control_service |

```bash
ros2 topic list
ros2 topic echo /robo/joint_state
ros2 topic hz /robo/joint_command        # expect ~50 Hz
```

### ⚠️ Three open blockers before hardware

1. **`gait_phase` not zeroed at zero command.** Training zeroed sin *and* cos when
   `||vel_cmd|| < 0.1`; `policy_service::buildObservationTerms()` advances phase
   unconditionally → robot steps in place when the stick is released. Measured cost:
   up to **6.85° joint-target error standing** (worst `right_knee`; mean 3.66°).
   Needs a C++ patch. Reference: `scripts/mujoco_deploy.py:215`.
2. **Ankle kp/kd cross the transmission untransformed** — `42.857 / 2.279` are
   joint-space gains applied in motor space to the X6-60 pair. Retune on a stand.
3. **Never normalize observations in the service** — the graph carries its own
   `Sub`/`Div` normalizer. Double normalization = instant violent twitching.

---

## 12. Sync between laptop and server

```bash
# code goes through git
git add -A && git commit -m "..." && git push
# on the server
git pull

# logs/ is gitignored -- copy checkpoints down manually
scp -r user@server:~/unitree_rl_mjlab/logs/rsl_rl/tahiti_c1_velocity/<ts> \
       ./logs/rsl_rl/tahiti_c1_velocity/

# a policy you want to keep is committed under deploy/ (not ignored)
cp logs/rsl_rl/tahiti_c1_velocity/<ts>/policy.onnx deploy/robots/tahiti_c1/policy/
```

Note `robo_control/` is its own git repo nested inside this one; it is untracked here
and should be pulled/pushed separately.

---

## 13. Task IDs

```bash
python scripts/list_envs.py
```

**This repo (velocity — `<name>-Flat` / `<name>-Rough`):**
`Tahiti-C1`, `Unitree-A2`, `Unitree-As2`, `Unitree-G1`, `Unitree-G1-23Dof`,
`Unitree-Go2`, `Unitree-H1_2`, `Unitree-H2`, `Unitree-R1`

**This repo (tracking):**
`Unitree-G1-Tracking`, `Unitree-G1-Tracking-No-State-Estimation`,
`Unitree-G1-23Dof-Tracking`, `Unitree-G1-23Dof-Tracking-No-State-Estimation`

**From mjlab itself:** `Mjlab-Velocity-{Flat,Rough}-Unitree-{G1,Go1}`,
`Mjlab-Tracking-Flat-Unitree-G1[-No-State-Estimation]`, `Mjlab-Lift-Cube-Yam[-Depth|-Rgb]`

Start on `-Flat`; `-Rough` adds the height-scan observation and terrain curriculum and
is much harder from scratch.

---

## 14. Gotcha index

| # | Gotcha | Where |
|---|---|---|
| 1 | `pip install -e .` pulls `mujoco`/`warp-lang` too new → import AttributeErrors | §1 |
| 2 | `scipy` is imported by mjlab's terrain code but not declared as a dependency | §1 |
| 3 | `num_envs` defaults to **1** | §3 |
| 4 | `logger` defaults to **wandb** and blocks on login | §3 |
| 5 | Gradient updates per iteration are fixed at 20 regardless of `num_envs` | §3 |
| 6 | Throughput peaks then falls as `num_envs` grows — not a VRAM limit | §3 |
| 7 | First Warp compile takes ~35 s and looks like a hang | §3 |
| 8 | `play.py` AttributeErrors without `--checkpoint_file` | §5 |
| 9 | The velocity joystick exists only in the **viser** viewer | §5 |
| 10 | The viser `Enable` checkbox defaults **OFF** | §5 |
| 11 | Play mode disables noise and pushes — easier than training | §5 |
| 12 | `mujoco_deploy.py` needs `onnxruntime`, absent from the `mjlab` env | §6 |
| 13 | `urdf_to_mjcf.py` has no argparse — any arg still runs the conversion | §8 |
| 14 | 29-dof G1 **tracking** uses `mjlab.asset_zoo`, not this repo's assets | §9 |
| 15 | G1 `deploy.yaml` is hand-written; nothing generates it | §10 |
| 16 | Policy version dirs sort lexicographically — `v10` loses to `v2` | §10 |
| 17 | `bad_orientation()` is disabled — no automatic fall protection | §10 |
| 18 | tahiti_c1 does **not** use the Unitree C++ FSM; it uses MNN + ROS 2 | §11 |
| 19 | tahiti_c1 gait phase is not zeroed at zero command | §11 |
| 20 | tahiti_c1 ankle gains cross the transmission untransformed | §11 |
