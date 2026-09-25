"""Run a trained policy on the MuJoCo robot, driven by an Xbox gamepad.

This is the deployment path, not the training path. It deliberately uses only
what a real controller has: the exported ONNX, the generated deploy_config.yaml
and a plain MJCF scene. mjlab is never imported. So if this walks and your
firmware port does not, the bug is in the port, not the policy.

    python scripts/mujoco_deploy.py                       # newest run, gamepad
    python scripts/mujoco_deploy.py --keyboard            # no gamepad
    python scripts/mujoco_deploy.py --headless --duration 10

Gamepad (Xbox layout)
    left stick  Y / X    forward-back / strafe
    right stick X        turn
    A                    zero the command (stand)
    START                reset the robot to its home pose
    BACK                 quit

Control loop, matching training exactly:
    obs = [gyro(3), proj_gravity(3), cmd(3), phase(2),
           q-default(12), qd(12), prev_action(12)]          -> 47 per frame, RAW
           x history_length, flattened TERM-MAJOR           -> 235 at history 5
    action   = onnx(obs)                                     -> 12, unbounded
    q_target = default_pos + action_scale * action
    tau      = clamp(kp*(q_target-q) - kd*qd, +/-effort)     (done by MuJoCo)
"""

from __future__ import annotations

import argparse
import collections
import math
import pathlib
import time

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCENE = ROOT / "src/assets/robots/tahiti_c1/xmls/scene_tahiti_c1.xml"


# ---------------------------------------------------------------------------
# Command sources
# ---------------------------------------------------------------------------


class GamepadCommand:
  """Xbox pad via pygame. Axis indices follow the common Linux SDL mapping."""

  AX_LY, AX_LX, AX_RX = 1, 0, 3
  BTN_A, BTN_B, BTN_BACK, BTN_START = 0, 1, 6, 7

  def __init__(self, ranges: dict, deadzone: float) -> None:
    import pygame

    self.pygame = pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
      raise RuntimeError("No gamepad found. Use --keyboard, or check /dev/input/js*.")
    self.js = pygame.joystick.Joystick(0)
    self.js.init()
    self.ranges = ranges
    self.deadzone = deadzone
    self.quit = False
    self.reset = False
    self.push = False
    print(f"[gamepad] {self.js.get_name()}  axes={self.js.get_numaxes()}")

  def _axis(self, i: int) -> float:
    v = self.js.get_axis(i) if i < self.js.get_numaxes() else 0.0
    return 0.0 if abs(v) < self.deadzone else v

  @staticmethod
  def _map(v: float, lo: float, hi: float) -> float:
    # Map [-1,1] onto an asymmetric range without a jump at zero.
    return v * (hi if v >= 0 else -lo)

  def __call__(self) -> np.ndarray:
    self.pygame.event.pump()
    for e in self.pygame.event.get():
      if e.type == self.pygame.JOYBUTTONDOWN:
        if e.button == self.BTN_BACK:
          self.quit = True
        elif e.button == self.BTN_START:
          self.reset = True
        elif e.button == self.BTN_B:
          self.push = True
    if self.js.get_button(self.BTN_A):
      return np.zeros(3, dtype=np.float32)
    r = self.ranges
    return np.array(
      [
        self._map(-self._axis(self.AX_LY), *r["lin_vel_x"]),  # stick up = +x
        self._map(-self._axis(self.AX_LX), *r["lin_vel_y"]),  # stick left = +y
        self._map(-self._axis(self.AX_RX), *r["ang_vel_z"]),  # stick left = +yaw
      ],
      dtype=np.float32,
    )

  def close(self) -> None:
    self.pygame.quit()


class ScriptedCommand:
  """Fallback with no gamepad: hold a fixed command."""

  def __init__(self, vx: float, vy: float, wz: float) -> None:
    self.cmd = np.array([vx, vy, wz], dtype=np.float32)
    self.quit = False
    self.reset = False
    self.push = False
    print(f"[scripted] command = {self.cmd}")

  def __call__(self) -> np.ndarray:
    return self.cmd

  def close(self) -> None:
    pass


# ---------------------------------------------------------------------------
# Disturbances
# ---------------------------------------------------------------------------

# What the `push_robot` event actually did during training
# (src/tasks/velocity/config/tahiti_c1/env_cfgs.py). Note these are VELOCITIES,
# not forces: mjlab's push_by_setting_velocity does
#     vel_w += uniform(range);  write_root_link_velocity_to_sim(vel_w)
# i.e. an instantaneous change of the base velocity, sampled in WORLD frame,
# every 5-6 s. No external force or torque was ever applied during training.
TRAIN_PUSH_LIN = np.array([[-0.5, 0.5], [-0.5, 0.5], [-0.4, 0.4]])  # x, y, z  [m/s]
TRAIN_PUSH_ANG = np.array(
  [[-0.52, 0.52], [-0.52, 0.52], [-0.78, 0.78]]  # roll, pitch, yaw  [rad/s]
)
TRAIN_PUSH_INTERVAL_S = (5.0, 6.0)


def push_velocity(model, data, rng, scale: float = 1.0) -> np.ndarray:
  """Reproduce the training push exactly: a velocity impulse on the free joint.

  MuJoCo free-joint convention: qvel[0:3] is linear velocity in the WORLD frame,
  qvel[3:6] is angular velocity in the BODY frame. The training ranges are
  sampled in world frame for both, so the angular part is rotated in.
  """
  dv_w = rng.uniform(TRAIN_PUSH_LIN[:, 0], TRAIN_PUSH_LIN[:, 1]) * scale
  dw_w = rng.uniform(TRAIN_PUSH_ANG[:, 0], TRAIN_PUSH_ANG[:, 1]) * scale
  rot = np.zeros(9)
  mujoco.mju_quat2Mat(rot, data.qpos[3:7])
  data.qvel[0:3] += dv_w
  data.qvel[3:6] += rot.reshape(3, 3).T @ dw_w
  return dv_w


def push_force(model, data, body_id: int, force_w: np.ndarray) -> None:
  """Apply a Cartesian force to a body, held until cleared.

  More physical than the velocity impulse (a real shove acts over time), but
  NOT what the policy was trained on. An impulse of F*dt on mass m produces
  dv = F*dt/m, so F = m*dv/dt gives a force roughly equivalent to a push.
  """
  data.xfrc_applied[body_id, :3] = force_w
  data.xfrc_applied[body_id, 3:] = 0.0


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------


class PolicyController:
  """Everything a real runtime must do, and nothing it cannot."""

  def __init__(self, cfg: dict, onnx_path: pathlib.Path) -> None:
    self.cfg = cfg
    joints = cfg["joints"]
    self.joint_names = [j["name"] for j in joints]
    self.default_pos = np.array([j["default_pos"] for j in joints], dtype=np.float32)
    self.action_scale = np.array([j["action_scale"] for j in joints], dtype=np.float32)
    self.lower = np.array([j["lower_limit"] for j in joints], dtype=np.float32)
    self.upper = np.array([j["upper_limit"] for j in joints], dtype=np.float32)

    self.obs_dim = int(cfg["policy"]["obs_dim"])
    obs_cfg = cfg.get("observation", {})
    self.history = int(obs_cfg.get("history_length", 1))
    self.single_dim = int(obs_cfg.get("single_step_dim", self.obs_dim))
    self.n = len(joints)
    self.control_dt = float(cfg["control"]["control_dt"])
    self.phase_period = float(cfg["gait"]["phase_period_s"])
    self.zero_cmd_thresh = float(cfg["gait"]["zero_command_threshold"])

    self.session = ort.InferenceSession(
      str(onnx_path), providers=["CPUExecutionProvider"]
    )
    self.input_name = self.session.get_inputs()[0].name
    # The exported graph carries its own Sub/Div normalizer, so observations go
    # in raw. Normalizing here as well would be the classic deployment bug.
    assert cfg["policy"]["obs_normalization_baked_in"], (
      "This policy expects EXTERNAL observation normalization, which this "
      "controller does not implement."
    )
    self.reset()

  def reset(self) -> None:
    self.prev_action = np.zeros(self.n, dtype=np.float32)
    self.t = 0.0
    # One ring buffer per term, matching robo_control's ObservationHistory.
    # Seeded lazily on the first frame with `history` copies of it, exactly as
    # the SDK does, so the two agree from the very first control cycle.
    self._hist: list[collections.deque] | None = None

  def _push_history(self, terms: list[np.ndarray]) -> np.ndarray:
    """Append one frame and return the flattened TERM-MAJOR vector.

    Layout: [A_t0..A_tH-1, B_t0..B_tH-1, ...], oldest -> newest within a term.
    This is what mjlab produces with flatten_history_dim=True and what
    policy_service::ObservationHistory::flatten() expects, so no reordering is
    needed anywhere between training and the robot.
    """
    if self._hist is None:
      self._hist = [
        collections.deque([t.copy() for _ in range(self.history)], maxlen=self.history)
        for t in terms
      ]
    else:
      for buf, t in zip(self._hist, terms):
        buf.append(t)
    return np.concatenate([np.concatenate(b) for b in self._hist]).astype(np.float32)

  def build_obs(
    self, gyro: np.ndarray, proj_g: np.ndarray, cmd: np.ndarray,
    q: np.ndarray, qd: np.ndarray,
  ) -> np.ndarray:
    phase = (self.t % self.phase_period) / self.phase_period
    sc = np.array(
      [math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)], dtype=np.float32
    )
    # Zeroed while standing, exactly as mdp.phase does during training.
    if np.linalg.norm(cmd) < self.zero_cmd_thresh:
      sc[:] = 0.0
    terms = [gyro, proj_g, cmd, sc, q - self.default_pos, qd, self.prev_action]
    frame = np.concatenate(terms).astype(np.float32)
    assert frame.shape[0] == self.single_dim, (
      f"frame {frame.shape[0]} != single_step_dim {self.single_dim}"
    )
    obs = self._push_history(terms) if self.history > 1 else frame
    assert obs.shape[0] == self.obs_dim, f"obs {obs.shape[0]} != {self.obs_dim}"
    return obs

  def step(self, obs: np.ndarray) -> np.ndarray:
    action = self.session.run(None, {self.input_name: obs[None, :]})[0][0]
    self.prev_action = action.astype(np.float32)
    q_target = self.default_pos + self.action_scale * self.prev_action
    self.t += self.control_dt
    return np.clip(q_target, self.lower, self.upper)


# ---------------------------------------------------------------------------


def newest_config() -> pathlib.Path:
  runs = sorted(
    (ROOT / "logs/rsl_rl/tahiti_c1_velocity").glob("*/deploy_config.yaml"),
    key=lambda p: p.stat().st_mtime,
  )
  if not runs:
    raise FileNotFoundError("No deploy_config.yaml; run scripts/export_deploy_config.py")
  return runs[-1]


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--config", default=None)
  ap.add_argument("--scene", default=str(SCENE))
  ap.add_argument("--keyboard", action="store_true", help="no gamepad; fixed command")
  ap.add_argument("--vx", type=float, default=0.5)
  ap.add_argument("--vy", type=float, default=0.0)
  ap.add_argument("--wz", type=float, default=0.0)
  ap.add_argument("--headless", action="store_true")
  ap.add_argument("--duration", type=float, default=0.0, help="seconds; 0 = forever")
  ap.add_argument(
    "--push-mode",
    choices=["off", "velocity", "force"],
    default="off",
    help="'velocity' reproduces the training push exactly; 'force' is a "
    "physical shove the policy was never trained on",
  )
  ap.add_argument(
    "--push-interval", type=float, default=0.0,
    help="seconds between automatic pushes; 0 = manual only (gamepad B)",
  )
  ap.add_argument("--push-scale", type=float, default=1.0,
                  help="multiplier on the trained push magnitude")
  ap.add_argument("--push-force", type=float, default=268.0,
                  help="force mode: newtons (m*dv/dt = 53.5*0.5/0.1)")
  ap.add_argument("--push-duration", type=float, default=0.1,
                  help="force mode: seconds to hold the force")
  ap.add_argument("--seed", type=int, default=0)
  args = ap.parse_args()

  cfg_path = pathlib.Path(args.config) if args.config else newest_config()
  cfg = yaml.safe_load(cfg_path.read_text())
  onnx_path = ROOT / cfg["policy"]["onnx"]
  print(f"config : {cfg_path}")
  print(f"policy : {onnx_path.name}")

  ctrl = PolicyController(cfg, onnx_path)

  model = mujoco.MjModel.from_xml_path(args.scene)
  data = mujoco.MjData(model)
  decimation = int(cfg["control"]["decimation"])

  # Map policy joint order -> MuJoCo addresses. Never assume they match.
  qadr, vadr, ctrl_ids = [], [], []
  for name in ctrl.joint_names:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if jid < 0 or aid < 0:
      raise KeyError(f"'{name}' missing from {args.scene}")
    qadr.append(model.jnt_qposadr[jid])
    vadr.append(model.jnt_dofadr[jid])
    ctrl_ids.append(aid)
  qadr, vadr, ctrl_ids = map(np.array, (qadr, vadr, ctrl_ids))

  sid = {
    n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, n)
    for n in ("imu_ang_vel",)
  }
  gyro_adr = model.sensor_adr[sid["imu_ang_vel"]]

  def reset() -> None:
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    ctrl.reset()

  reset()

  if args.keyboard:
    pad = ScriptedCommand(args.vx, args.vy, args.wz)
  else:
    try:
      pad = GamepadCommand(cfg["command"], float(cfg["command"]["deadzone"]))
    except Exception as e:  # noqa: BLE001
      print(f"[warn] {e}\n[warn] falling back to scripted command")
      pad = ScriptedCommand(args.vx, args.vy, args.wz)

  viewer = None
  if not args.headless:
    # NB: `from mujoco import viewer as ...`, not `import mujoco.viewer` -- the
    # latter would rebind `mujoco` as a local and shadow the module-level import.
    from mujoco import viewer as mj_viewer

    viewer = mj_viewer.launch_passive(model, data)

  rng = np.random.default_rng(args.seed)
  torso_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
  next_push = (
    rng.uniform(*TRAIN_PUSH_INTERVAL_S) if args.push_interval < 0 else args.push_interval
  )
  force_until = -1.0
  n_pushed = 0

  print("running. BACK=quit  START=reset  A=stand  B=push")
  if args.push_mode != "off":
    how = "every %.1fs" % args.push_interval if args.push_interval else "manual (B)"
    print(f"push   : mode={args.push_mode}  {how}  scale={args.push_scale}")
  wall = time.time()
  try:
    while True:
      if pad.quit or (args.duration and data.time > args.duration):
        break
      if pad.reset:
        reset()
        pad.reset = False

      # --- disturbance ----------------------------------------------------
      due = args.push_interval and data.time >= next_push
      if args.push_mode != "off" and (pad.push or due):
        if args.push_mode == "velocity":
          dv = push_velocity(model, data, rng, args.push_scale)
          print(f"  [push] t={data.time:6.2f}s  dv_world=[{dv[0]:+.2f} {dv[1]:+.2f} {dv[2]:+.2f}] m/s")
        else:
          ang = rng.uniform(0, 2 * math.pi)
          f = args.push_force * args.push_scale * np.array(
            [math.cos(ang), math.sin(ang), 0.0]
          )
          push_force(model, data, torso_id, f)
          force_until = data.time + args.push_duration
          print(f"  [push] t={data.time:6.2f}s  F=[{f[0]:+6.1f} {f[1]:+6.1f}] N for {args.push_duration}s")
        n_pushed += 1
        pad.push = False
        if due:
          next_push = data.time + args.push_interval
      if force_until > 0 and data.time > force_until:
        data.xfrc_applied[torso_id] = 0.0
        force_until = -1.0

      cmd = pad()
      gyro = data.sensordata[gyro_adr : gyro_adr + 3].astype(np.float32)
      # Gravity in body frame = R^T * (0,0,-1). An IMU gives this directly.
      quat = data.qpos[3:7]
      rot = np.zeros(9)
      mujoco.mju_quat2Mat(rot, quat)
      proj_g = (rot.reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])).astype(np.float32)

      q = data.qpos[qadr].astype(np.float32)
      qd = data.qvel[vadr].astype(np.float32)

      q_target = ctrl.step(ctrl.build_obs(gyro, proj_g, cmd, q, qd))
      data.ctrl[ctrl_ids] = q_target

      for _ in range(decimation):
        mujoco.mj_step(model, data)

      if viewer is not None:
        if not viewer.is_running():
          break
        viewer.sync()
        # Keep sim time near wall-clock so the gait looks right.
        lag = data.time - (time.time() - wall)
        if lag > 0:
          time.sleep(lag)

      if args.headless and int(data.time * 50) % 100 == 0:
        print(
          f"  t={data.time:6.2f}s  z={data.qpos[2]:.3f}  "
          f"cmd=[{cmd[0]:+.2f} {cmd[1]:+.2f} {cmd[2]:+.2f}]"
        )
  except KeyboardInterrupt:
    print("\ninterrupted")
  finally:
    if viewer is not None:
      viewer.close()
    pad.close()

  fell = data.qpos[2] < 0.6
  print(
    f"stopped at t={data.time:.2f}s  base height {data.qpos[2]:.3f} m  "
    f"pushes={n_pushed}  {'FELL' if fell else 'still standing'}"
  )


if __name__ == "__main__":
  main()
