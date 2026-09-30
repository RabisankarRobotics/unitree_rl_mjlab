"""Unitree G1 perceptive stair walking: stairs + flat, with a terrain height scan.

SELF-CONTAINED. This file does NOT call ``make_velocity_env_cfg()`` and does not
build on the G1 rough/flat configs. Every sensor, observation, action, command,
event, reward, termination and curriculum term is written out here with G1's
values already filled in. What you read is what runs.

Only term *functions* are imported: the shared library in
``src.tasks.velocity.mdp`` and three stair-specific ones in ``stairs_mdp.py``.

This is the TEACHER stage of perceptive locomotion: the policy gets a near-
perfect height scan by ray-casting against the simulated terrain. On the real
robot the same 17 x 11 grid must come from a LiDAR elevation map (see
section 2), and realistic map errors get added in a later stage.

Differences from Unitree-G1-Rough, and why
------------------------------------------
  terrain        flat + stairs up/down only (no slopes/waves/noise); steps
                 3-20 cm high, treads 25/30/35 cm
  height scan    hits terrain only; Rough's also hits G1's own leg meshes
  foot height    measured from the step under the foot, not from world z = 0
  new reward     feet_stumble: toe kicking a stair riser
  commands       slower (max 1.0 m/s): stairs, not running
  scan noise     +/-2 cm instead of +/-10 cm (teacher; realism comes later)

Layout
------
  1. Constants        <- start here when tuning
  2. Sensors
  3. Observations
  4. Actions
  5. Commands
  6. Events (resets + domain randomisation)
  7. Rewards
  8. Terminations
  9. Curriculum
 10. Terrain
 11. Assembly (scene / sim / viewer)
 12. Play overrides

Run
---
  preview (no training):  python scripts/play.py Unitree-G1-Stairs --agent zero
  train:                  python scripts/train.py Unitree-G1-Stairs --env.scene.num-envs 4096
"""

import math

import mjlab.terrains as terrain_gen
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.action_manager import ActionTermCfg
from mjlab.managers.command_manager import CommandTermCfg
from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.metrics_manager import MetricsTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.scene import SceneCfg
from mjlab.sensor import (
  ContactMatch,
  ContactSensorCfg,
  GridPatternCfg,
  ObjRef,
  RayCastSensorCfg,
)
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.terrains import TerrainEntityCfg
from mjlab.terrains.terrain_generator import TerrainGeneratorCfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise
from mjlab.viewer import ViewerConfig

import src.tasks.velocity.mdp as mdp
from src.assets.robots import G1_ACTION_SCALE, get_g1_robot_cfg
from src.tasks.velocity.mdp import UniformVelocityCommandCfg

from . import stairs_mdp

# ============================================================================
# 1. CONSTANTS -- the things you are most likely to change
# ============================================================================

# --- robot names (from src/assets/robots/unitree_g1/xmls/g1.xml) -----------
PELVIS = "pelvis"  # height scan is attached here
TORSO = "torso_link"  # orientation / angular-velocity rewards act on this
FOOT_SITES = ("left_foot", "right_foot")  # on the sole, under the ankle
FOOT_BODIES = ("left_ankle_roll_link", "right_ankle_roll_link")
FOOT_GEOMS = tuple(
  f"{side}_foot{i}_collision" for side in ("left", "right") for i in range(1, 8)
)

# --- stairs ------------------------------------------------------------------
# A policy handles stairs INSIDE its training range and is unreliable outside
# it. Target stairs (measured): 16 cm rise, 30 cm tread. Train past them so they
# sit inside the range, not at its edge.
#
# Step height: each staircase gets ONE height, drawn from this range by its
# row: row r gets a random value in [r/10, (r+1)/10] of the range. So row 0 is
# 3.0-4.7 cm and row 9 is 18.3-20 cm. 16 cm falls in row 7-8. G1 may not
# manage 20 cm; if the curriculum stalls at 16-18 cm that is still enough.
STEP_HEIGHT_RANGE = (0.03, 0.20)
# Tread (step depth) [m]. One staircase type per value, for both up and down,
# so the policy cannot overfit one stride length. G1 foot is ~0.2 m long.
STEP_TREADS = (0.25, 0.30, 0.35)

# --- height scan -------------------------------------------------------------
# 1.6 m (forward/back) x 1.0 m (left/right) at 10 cm spacing = 17 x 11 = 187
# rays, centred on the pelvis. At 1 m/s this sees ~0.8 s ahead.
SCAN_SIZE = (1.6, 1.0)
SCAN_RESOLUTION = 0.1
SCAN_MAX_DISTANCE = 5.0
# Uniform noise on every scan value, in metres. Kept small for the teacher.
# Stage 3 (sim2real) widens this and adds holes / delay / offsets.
SCAN_NOISE = 0.02

# --- gait ----------------------------------------------------------------------
GAIT_PERIOD = 0.6  # [s] one full left+right cycle
GAIT_STANCE_THRESHOLD = 0.56  # fraction of the cycle a foot is on the ground
# Target swing height ABOVE THE STEP UNDER THE FOOT (the foot ray switches to
# the next step once the foot is over it). Slightly above flat ground's 0.10 m.
# It does not need to exceed the tallest riser: lifting the toe over a riser
# is driven by the foot_stumble penalty, not by this target.
FOOT_CLEARANCE = 0.12

# --- commands [m/s, rad/s] ---------------------------------------------------
# Slower than Rough (-1..2 m/s): stairs are walked, not run.
CMD_LIN_VEL_X = (-0.6, 1.0)
CMD_LIN_VEL_Y = (-0.4, 0.4)
CMD_ANG_VEL_Z = (-1.0, 1.0)
CMD_STANDING_THRESHOLD = 0.1  # below this |command|, the robot should stand

# --- episode / control -------------------------------------------------------
FELL_OVER_ANGLE = math.radians(70.0)
SIM_TIMESTEP = 0.005  # physics 200 Hz
DECIMATION = 4  # policy 50 Hz
EPISODE_LENGTH_S = 20.0

# --- posture reward spreads (larger = more freedom for that joint) ----------
POSE_STD_STANDING = {".*": 0.05}
POSE_STD_WALKING = {
  # Legs: knee / hip pitch loose -- stairs need big knee and hip flexion.
  r".*hip_pitch.*": 0.5,
  r".*hip_roll.*": 0.15,
  r".*hip_yaw.*": 0.15,
  r".*knee.*": 0.5,
  r".*ankle_pitch.*": 0.15,
  r".*ankle_roll.*": 0.1,
  # Waist: tight, keeps the torso upright.
  r".*waist_yaw.*": 0.15,
  r".*waist_roll.*": 0.1,
  r".*waist_pitch.*": 0.1,
  # Arms: small natural swing.
  r".*shoulder_pitch.*": 0.15,
  r".*shoulder_roll.*": 0.1,
  r".*shoulder_yaw.*": 0.1,
  r".*elbow.*": 0.1,
  r".*wrist.*": 0.1,
}
POSE_STD_RUNNING = {
  r".*hip_pitch.*": 0.5,
  r".*hip_roll.*": 0.25,
  r".*hip_yaw.*": 0.25,
  r".*knee.*": 0.5,
  r".*ankle_pitch.*": 0.25,
  r".*ankle_roll.*": 0.1,
  r".*waist_yaw.*": 0.25,
  r".*waist_roll.*": 0.1,
  r".*waist_pitch.*": 0.1,
  r".*shoulder_pitch.*": 0.25,
  r".*shoulder_roll.*": 0.1,
  r".*shoulder_yaw.*": 0.1,
  r".*elbow.*": 0.1,
  r".*wrist.*": 0.1,
}


def unitree_g1_stairs_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """G1 velocity tracking on flat ground and stairs, with a height scan."""

  # ==========================================================================
  # 2. SENSORS
  # ==========================================================================

  # --- the "vision": terrain height grid around the pelvis -----------------
  # Each ray is shot straight down. The observation (section 3) is
  #   pelvis_z - ground_z   for each of the 187 grid points.
  # On the real robot you reproduce the SAME numbers by sampling a LiDAR
  # elevation map at the same 187 points (same order, same yaw alignment).
  terrain_scan = RayCastSensorCfg(
    name="terrain_scan",
    frame=ObjRef(type="body", name=PELVIS, entity="robot"),
    # Grid follows the robot's position and heading but NOT its pitch/roll,
    # so rays always point straight down even when the torso leans.
    ray_alignment="yaw",
    pattern=GridPatternCfg(size=SCAN_SIZE, resolution=SCAN_RESOLUTION),
    max_distance=SCAN_MAX_DISTANCE,
    exclude_parent_body=True,
    # Terrain only. G1's visual meshes are geom group 2 and collision capsules
    # group 3; mjlab's default (0, 1, 2) would let rays hit the robot's own
    # legs, reporting a thigh instead of the step behind it.
    include_geom_groups=(0,),
    debug_vis=True,
  )

  # --- one downward ray under each foot -------------------------------------
  # Gives the height of the step under each foot, so foot clearance can be
  # measured from that step instead of from world z = 0 (see stairs_mdp.py).
  # Attached to the ankle body (3.5 cm above the sole) so the ray starts above
  # the ground even when the foot sinks slightly into it. size=(0, 0) -> 1 ray.
  foot_scans = tuple(
    RayCastSensorCfg(
      name=f"{side}_foot_scan",
      frame=ObjRef(type="body", name=body, entity="robot"),
      ray_alignment="yaw",
      pattern=GridPatternCfg(size=(0.0, 0.0), resolution=0.1),
      max_distance=2.0,
      exclude_parent_body=True,
      include_geom_groups=(0,),
    )
    for side, body in zip(("left", "right"), FOOT_BODIES, strict=True)
  )
  FOOT_SCAN_NAMES = tuple(s.name for s in foot_scans)

  # --- foot <-> terrain contact ---------------------------------------------
  # Drives gait / slip / landing / stumble rewards and the contact critic obs.
  # reduce="netforce" sums all contacts per foot and reports the force in the
  # WORLD frame, which feet_stumble relies on.
  feet_ground_contact = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(
      mode="subtree", pattern=r"^(left_ankle_roll_link|right_ankle_roll_link)$", entity="robot"
    ),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )

  # --- self collision (e.g. knee hitting knee, hand hitting thigh) ----------
  self_collision = ContactSensorCfg(
    name="self_collision",
    primary=ContactMatch(mode="subtree", pattern=PELVIS, entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern=PELVIS, entity="robot"),
    fields=("found", "force"),
    reduce="none",
    num_slots=1,
    history_length=4,
  )

  # ==========================================================================
  # 3. OBSERVATIONS
  # ==========================================================================
  # actor  = what the deployed policy sees. Everything here must be available
  #          on the real robot. 98 proprio + 187 scan = 285 dims.
  # critic = actor + privileged simulator state. Training only.

  actor_terms = {
    # 3 -- IMU gyro.
    "base_ang_vel": ObservationTermCfg(
      func=mdp.builtin_sensor,
      params={"sensor_name": "robot/imu_ang_vel"},
      noise=Unoise(n_min=-0.2, n_max=0.2),
    ),
    # 3 -- gravity direction in the body frame (from the IMU orientation).
    "projected_gravity": ObservationTermCfg(
      func=mdp.projected_gravity,
      noise=Unoise(n_min=-0.05, n_max=0.05),
    ),
    # 3 -- commanded (vx, vy, wz).
    "command": ObservationTermCfg(
      func=mdp.generated_commands,
      params={"command_name": "twist"},
    ),
    # 2 -- sin/cos gait clock; zero when the command is ~0.
    "phase": ObservationTermCfg(
      func=mdp.phase,
      params={"period": GAIT_PERIOD, "command_name": "twist"},
    ),
    # 29 -- joint positions relative to the default pose.
    "joint_pos": ObservationTermCfg(
      func=mdp.joint_pos_rel,
      noise=Unoise(n_min=-0.01, n_max=0.01),
    ),
    # 29 -- joint velocities.
    "joint_vel": ObservationTermCfg(
      func=mdp.joint_vel_rel,
      noise=Unoise(n_min=-1.5, n_max=1.5),
    ),
    # 29 -- previous action.
    "actions": ObservationTermCfg(func=mdp.last_action),
    # 187 -- THE PERCEPTION INPUT. pelvis_z - ground_z per grid point, in m,
    # noised, then scaled by 1/5 so values are ~0.15 on flat ground.
    # Order: x (forward) varies fastest, then y. Deployment must match it.
    "height_scan": ObservationTermCfg(
      func=envs_mdp.height_scan,
      params={"sensor_name": terrain_scan.name},
      noise=Unoise(n_min=-SCAN_NOISE, n_max=SCAN_NOISE),
      scale=1 / SCAN_MAX_DISTANCE,
    ),
  }

  critic_terms = {
    **actor_terms,
    # 187 -- noise-free scan.
    "height_scan": ObservationTermCfg(
      func=envs_mdp.height_scan,
      params={"sensor_name": terrain_scan.name},
      scale=1 / SCAN_MAX_DISTANCE,
    ),
    # 3 -- true base linear velocity. Not measurable on hardware.
    "base_lin_vel": ObservationTermCfg(
      func=mdp.builtin_sensor,
      params={"sensor_name": "robot/imu_lin_vel"},
      noise=Unoise(n_min=-0.5, n_max=0.5),
    ),
    # 2 -- each foot's height above the step under it.
    "foot_height": ObservationTermCfg(
      func=stairs_mdp.terrain_relative_foot_height,
      params={
        "sensor_names": FOOT_SCAN_NAMES,
        "asset_cfg": SceneEntityCfg("robot", site_names=FOOT_SITES),
      },
    ),
    # 2 -- time since each foot left the ground.
    "foot_air_time": ObservationTermCfg(
      func=mdp.foot_air_time,
      params={"sensor_name": feet_ground_contact.name},
    ),
    # 2 -- binary contact flags.
    "foot_contact": ObservationTermCfg(
      func=mdp.foot_contact,
      params={"sensor_name": feet_ground_contact.name},
    ),
    # 6 -- log-compressed contact forces.
    "foot_contact_forces": ObservationTermCfg(
      func=mdp.foot_contact_forces,
      params={"sensor_name": feet_ground_contact.name},
    ),
  }

  observations = {
    "actor": ObservationGroupCfg(
      terms=actor_terms,
      concatenate_terms=True,
      enable_corruption=True,  # apply the noise above (off in play mode)
      history_length=1,
    ),
    "critic": ObservationGroupCfg(
      terms=critic_terms,
      concatenate_terms=True,
      enable_corruption=False,
      history_length=1,
    ),
  }

  # Logged only, never fed to the policy.
  metrics = {
    "mean_action_acc": MetricsTermCfg(func=mdp.mean_action_acc),
  }

  # ==========================================================================
  # 4. ACTIONS
  # ==========================================================================
  # 29 joint position targets: q_target = q_default + scale * action.
  # Per-joint scale from g1_constants.py (0.25 * effort_limit / stiffness).

  actions: dict[str, ActionTermCfg] = {
    "joint_pos": JointPositionActionCfg(
      entity_name="robot",
      actuator_names=(".*",),
      scale=G1_ACTION_SCALE,
      use_default_offset=True,
    )
  }

  # ==========================================================================
  # 5. COMMANDS
  # ==========================================================================

  commands: dict[str, CommandTermCfg] = {
    "twist": UniformVelocityCommandCfg(
      entity_name="robot",
      resampling_time_range=(3.0, 8.0),
      rel_standing_envs=0.05,  # 5% of envs are told to stand still
      # Heading mode: a target heading is sampled and wz is computed to turn
      # towards it. Pyramid stairs rise on all four sides, so every heading
      # meets stairs.
      heading_command=True,
      heading_control_stiffness=0.5,
      debug_vis=True,
      ranges=UniformVelocityCommandCfg.Ranges(
        lin_vel_x=CMD_LIN_VEL_X,
        lin_vel_y=CMD_LIN_VEL_Y,
        ang_vel_z=CMD_ANG_VEL_Z,
        heading=(-math.pi, math.pi),
      ),
    )
  }
  commands["twist"].viz.z_offset = 1.15  # draw the arrow above the head

  # ==========================================================================
  # 6. EVENTS (resets + domain randomisation)
  # ==========================================================================
  # startup = once per env at build time, reset = every episode start,
  # interval = periodically during an episode.

  events = {
    # Random start position (+/-0.5 m) and heading on the env's terrain tile.
    "reset_base": EventTermCfg(
      func=mdp.reset_root_state_uniform,
      mode="reset",
      params={
        "pose_range": {
          "x": (-0.5, 0.5),
          "y": (-0.5, 0.5),
          "z": (0.0, 0.0),
          "yaw": (-3.14, 3.14),
        },
        "velocity_range": {},
      },
    ),
    # Start from the default pose (zero ranges).
    "reset_robot_joints": EventTermCfg(
      func=mdp.reset_joints_by_offset,
      mode="reset",
      params={
        "position_range": (-0.0, 0.0),
        "velocity_range": (-0.0, 0.0),
        "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
      },
    ),
    # Shove every 5-6 s. Removed in play mode.
    "push_robot": EventTermCfg(
      func=mdp.push_by_setting_velocity,
      mode="interval",
      interval_range_s=(5.0, 6.0),
      params={
        "velocity_range": {
          "x": (-0.5, 0.5),
          "y": (-0.5, 0.5),
          "z": (-0.4, 0.4),
          "roll": (-0.52, 0.52),
          "pitch": (-0.52, 0.52),
          "yaw": (-0.78, 0.78),
        },
      },
    ),
    # Foot friction per env; all foot geoms share the same draw.
    "foot_friction": EventTermCfg(
      mode="startup",
      func=dr.geom_friction,
      params={
        "asset_cfg": SceneEntityCfg("robot", geom_names=FOOT_GEOMS),
        "operation": "abs",
        "ranges": (0.3, 1.6),
        "shared_random": True,
      },
    ),
    # Joint encoder offset, +/-0.86 deg.
    "encoder_bias": EventTermCfg(
      mode="startup",
      func=dr.encoder_bias,
      params={
        "asset_cfg": SceneEntityCfg("robot"),
        "bias_range": (-0.015, 0.015),
      },
    ),
    # Torso centre-of-mass offset, +/-5 cm per axis.
    "base_com": EventTermCfg(
      mode="startup",
      func=dr.body_com_offset,
      params={
        "asset_cfg": SceneEntityCfg("robot", body_names=(TORSO,)),
        "operation": "add",
        "ranges": {
          0: (-0.05, 0.05),
          1: (-0.05, 0.05),
          2: (-0.05, 0.05),
        },
      },
    ),
  }

  # ==========================================================================
  # 7. REWARDS
  # ==========================================================================
  # Effective reward = weight * func(...). Positive weight = encourage.
  # In tensorboard (Episode_Reward/*) no penalty should dwarf
  # track_linear_velocity; if one does, it is dominating and probably wrong.

  rewards = {
    # --- task: follow the command -----------------------------------------
    "track_linear_velocity": RewardTermCfg(
      func=mdp.track_linear_velocity,
      weight=1.0,
      params={"command_name": "twist", "std": math.sqrt(0.25)},
    ),
    "track_angular_velocity": RewardTermCfg(
      func=mdp.track_angular_velocity,
      weight=1.0,
      params={"command_name": "twist", "std": math.sqrt(0.5)},
    ),
    # --- posture ------------------------------------------------------------
    "body_orientation_l2": RewardTermCfg(
      func=mdp.body_orientation_l2,
      weight=-1.0,
      params={"asset_cfg": SceneEntityCfg("robot", body_names=(TORSO,))},
    ),
    "pose": RewardTermCfg(
      func=mdp.variable_posture,
      weight=1.0,
      params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
        "command_name": "twist",
        "std_standing": POSE_STD_STANDING,
        "std_walking": POSE_STD_WALKING,
        "std_running": POSE_STD_RUNNING,
        "walking_threshold": 0.1,
        "running_threshold": 1.5,
      },
    ),
    "body_ang_vel": RewardTermCfg(
      func=mdp.body_angular_velocity_penalty,
      weight=-0.05,
      params={"asset_cfg": SceneEntityCfg("robot", body_names=(TORSO,))},
    ),
    "angular_momentum": RewardTermCfg(
      func=mdp.angular_momentum_penalty,
      weight=-0.025,
      params={"sensor_name": "robot/root_angmom"},
    ),
    # --- safety / smoothness ----------------------------------------------
    "is_terminated": RewardTermCfg(func=mdp.is_terminated, weight=-200.0),
    "joint_acc_l2": RewardTermCfg(func=mdp.joint_acc_l2, weight=-2.5e-7),
    "joint_pos_limits": RewardTermCfg(func=mdp.joint_pos_limits, weight=-10.0),
    "action_rate_l2": RewardTermCfg(func=mdp.action_rate_l2, weight=-0.05),
    "self_collisions": RewardTermCfg(
      func=mdp.self_collision_cost,
      weight=-1.0,
      params={"sensor_name": self_collision.name, "force_threshold": 10.0},
    ),
    # --- gait shaping ------------------------------------------------------
    # Contact should follow the gait clock; offset [0, 0.5] = alternating feet.
    "foot_gait": RewardTermCfg(
      func=mdp.feet_gait,
      weight=0.5,
      params={
        "period": GAIT_PERIOD,
        "offset": [0.0, 0.5],
        "threshold": GAIT_STANCE_THRESHOLD,
        "command_threshold": CMD_STANDING_THRESHOLD,
        "command_name": "twist",
        "sensor_name": feet_ground_contact.name,
      },
    ),
    # STAIRS: swing height measured above the step under the foot.
    # Replaces Rough's feet_clearance, which uses world height.
    "foot_clearance": RewardTermCfg(
      func=stairs_mdp.feet_clearance_terrain,
      weight=-1.0,
      params={
        "target_height": FOOT_CLEARANCE,
        "sensor_names": FOOT_SCAN_NAMES,
        "command_name": "twist",
        "command_threshold": CMD_STANDING_THRESHOLD,
        "asset_cfg": SceneEntityCfg("robot", site_names=FOOT_SITES),
      },
    ),
    # STAIRS: penalise the toe kicking a stair riser. This is what the policy
    # must learn to avoid by lifting the foot early -- using the height scan.
    "foot_stumble": RewardTermCfg(
      func=stairs_mdp.feet_stumble,
      weight=-1.0,
      params={"sensor_name": feet_ground_contact.name, "ratio": 4.0},
    ),
    # Stance foot should not slide.
    "foot_slip": RewardTermCfg(
      func=mdp.feet_slip,
      weight=-0.25,
      params={
        "sensor_name": feet_ground_contact.name,
        "command_name": "twist",
        "command_threshold": CMD_STANDING_THRESHOLD,
        "asset_cfg": SceneEntityCfg("robot", site_names=FOOT_SITES),
      },
    ),
    # Impact force at touchdown; important when stepping DOWN stairs.
    "soft_landing": RewardTermCfg(
      func=mdp.soft_landing,
      weight=-1e-3,
      params={
        "sensor_name": feet_ground_contact.name,
        "command_name": "twist",
        "command_threshold": CMD_STANDING_THRESHOLD,
      },
    ),
    # Joints should stay still when told to stand.
    "stand_still": RewardTermCfg(
      func=mdp.stand_still,
      weight=-1.0,
      params={
        "command_name": "twist",
        "command_threshold": CMD_STANDING_THRESHOLD,
        "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
      },
    ),
  }

  # ==========================================================================
  # 8. TERMINATIONS
  # ==========================================================================
  # time_out=True marks truncation (value is bootstrapped), not failure.

  terminations = {
    "time_out": TerminationTermCfg(func=mdp.time_out, time_out=True),
    "fell_over": TerminationTermCfg(
      func=mdp.bad_orientation,
      params={"limit_angle": FELL_OVER_ANGLE},
    ),
  }

  # ==========================================================================
  # 9. CURRICULUM
  # ==========================================================================

  curriculum = {
    # At each reset: an env that walked > 4 m (half a tile) moves to the next
    # harder row (taller steps); one that walked < half its commanded
    # distance moves down a row. Logged as Curriculum/terrain_levels.
    "terrain_levels": CurriculumTermCfg(
      func=mdp.terrain_levels_vel,
      params={"command_name": "twist"},
    ),
    # Start with gentle commands, widen after 5000 iterations (x 24 steps).
    "command_vel": CurriculumTermCfg(
      func=mdp.commands_vel,
      params={
        "command_name": "twist",
        "velocity_stages": [
          {
            "step": 0,
            "lin_vel_x": (-0.4, 0.6),
            "lin_vel_y": (-0.3, 0.3),
            "ang_vel_z": (-0.8, 0.8),
          },
          {
            "step": 5000 * 24,
            "lin_vel_x": CMD_LIN_VEL_X,
            "lin_vel_y": CMD_LIN_VEL_Y,
            "ang_vel_z": CMD_ANG_VEL_Z,
          },
        ],
      },
    ),
  }

  # ==========================================================================
  # 10. TERRAIN
  # ==========================================================================
  # A 10 x 22 grid of 8 m x 8 m tiles.
  #   rows    = difficulty (curriculum): step height 3 cm (row 0) -> 20 cm (row 9).
  #   columns = terrain type. `proportion` is normalised over all entries, so
  #             with 22 columns, 4 : 3 : 3 ... gives exactly 4 flat columns and
  #             3 columns per staircase type (3 treads x up/down = 6 types).
  # Each env starts at the centre of one tile.
  #   stairs_down_* (pyramid)          : starts on top, stairs lead DOWN.
  #   stairs_up_*   (inverted pyramid) : starts in a pit, stairs lead UP.
  # Tile layout: 1 m flat border, steps, 3 m flat platform in the middle.
  # Steps each way = (8 - 2*1 - 3) / 2 / tread -> 6 at 25 cm, 5 at 30, 4 at 35.

  sub_terrains: dict = {"flat": terrain_gen.BoxFlatTerrainCfg(proportion=4.0)}
  for tread in STEP_TREADS:
    cm = round(tread * 100)
    sub_terrains[f"stairs_down_{cm}cm"] = terrain_gen.BoxPyramidStairsTerrainCfg(
      proportion=3.0,
      step_height_range=STEP_HEIGHT_RANGE,
      step_width=tread,
      platform_width=3.0,
      border_width=1.0,
    )
    sub_terrains[f"stairs_up_{cm}cm"] = terrain_gen.BoxInvertedPyramidStairsTerrainCfg(
      proportion=3.0,
      step_height_range=STEP_HEIGHT_RANGE,
      step_width=tread,
      platform_width=3.0,
      border_width=1.0,
    )

  stairs_terrain = TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,  # flat margin around the whole grid
    num_rows=10,
    num_cols=22,
    curriculum=True,  # row index = difficulty (instead of random per tile)
    sub_terrains=sub_terrains,
    add_lights=True,
  )

  # ==========================================================================
  # 11. ASSEMBLY
  # ==========================================================================

  cfg = ManagerBasedRlEnvCfg(
    scene=SceneCfg(
      entities={"robot": get_g1_robot_cfg()},
      terrain=TerrainEntityCfg(
        terrain_type="generator",
        terrain_generator=stairs_terrain,
        # New envs start on rows 0-3 (3-8 cm steps) and earn their way up.
        max_init_terrain_level=3,
      ),
      sensors=(terrain_scan, *foot_scans, feet_ground_contact, self_collision),
      num_envs=1,  # overridden by --env.scene.num-envs
      extent=2.0,
    ),
    observations=observations,
    actions=actions,
    commands=commands,
    events=events,
    rewards=rewards,
    terminations=terminations,
    curriculum=curriculum,
    metrics=metrics,
    viewer=ViewerConfig(
      origin_type=ViewerConfig.OriginType.ASSET_BODY,
      entity_name="robot",
      body_name=TORSO,
      distance=3.0,
      elevation=-5.0,
      azimuth=90.0,
    ),
    sim=SimulationCfg(
      # Contact buffers sized for box stairs (many more contacts than a plane).
      nconmax=48,
      njmax=1500,
      contact_sensor_maxmatch=500,
      mujoco=MujocoCfg(
        timestep=SIM_TIMESTEP,
        iterations=10,
        ls_iterations=20,
        ccd_iterations=500,
      ),
    ),
    decimation=DECIMATION,
    episode_length_s=EPISODE_LENGTH_S,
  )

  if play:
    _apply_play_overrides(cfg)
  return cfg


# ============================================================================
# 12. PLAY OVERRIDES (scripts/play.py)
# ============================================================================


def _apply_play_overrides(cfg: ManagerBasedRlEnvCfg) -> None:
  # Never time out, so one episode can be watched indefinitely.
  cfg.episode_length_s = int(1e9)
  # True behaviour: no observation noise, no shoving, no curriculum.
  cfg.observations["actor"].enable_corruption = False
  cfg.events.pop("push_robot", None)
  cfg.curriculum = {}
  # Scatter robots across tiles instead of stacking them on one.
  cfg.events["randomize_terrain"] = EventTermCfg(
    func=envs_mdp.randomize_terrain,
    mode="reset",
    params={},
  )
  # Smaller grid; tile types and heights drawn at random from the full range.
  assert cfg.scene.terrain is not None
  gen = cfg.scene.terrain.terrain_generator
  assert gen is not None
  gen.curriculum = False
  gen.num_rows = 5
  gen.num_cols = 5
  gen.border_width = 10.0
