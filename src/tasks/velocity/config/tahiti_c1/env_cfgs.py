"""Tahiti C1 velocity environment configurations.

SELF-CONTAINED. Unlike the other robots in this repo, this file does NOT call
``make_velocity_env_cfg()``. Every observation, action, command, event, reward,
termination and curriculum term is written out here, with this robot's values
already substituted. Nothing is inherited, so what you read is what runs.

Only the term *functions* are imported (``src.tasks.velocity.mdp``); those are a
library of reward/observation implementations, not configuration.

Layout
------
  1. Robot-specific constants   <- start here when tuning
  2. Sensors
  3. Observations
  4. Actions
  5. Commands
  6. Events (domain randomisation)
  7. Rewards
  8. Terminations
  9. Curriculum
 10. Assembly (scene / sim / viewer)
 11. Flat variant + play overrides

Tuning guide
------------
  robot sags, crouches, weak                  -> gains, in c1_constants.py
  jitter, buzzing, huge action_rate penalty   -> gains, in c1_constants.py
  never leaves the ground / shuffles          -> FOOT_CLEARANCE up, foot_clearance weight up
  stomps, slaps the ground                    -> soft_landing weight more negative
  drags feet, trips on rough terrain          -> FOOT_CLEARANCE up
  feet slide during stance                    -> foot_slip weight more negative
  gait is not alternating / hops              -> foot_gait weight up, check GAIT_PERIOD
  torso wobbles, arms-out look                -> body_ang_vel / angular_momentum more negative
  falls constantly early in training           -> lower CMD_* ranges, lengthen curriculum stage 0
  tracks velocity poorly                      -> track_* weights up, or pose weight down
  won't stand still on zero command           -> stand_still weight more negative
  terminates too early on rough terrain       -> FELL_OVER_ANGLE up

See doc/actuator_tuning.md for the gain derivation and
``python scripts/check_actuators.py`` to re-verify gains after a model change.
"""

import math
from dataclasses import replace

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
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.terrains import TerrainEntityCfg
from mjlab.terrains.config import ROUGH_TERRAINS_CFG
from mjlab.utils.noise import UniformNoiseCfg as Unoise
from mjlab.viewer import ViewerConfig

import src.tasks.velocity.mdp as mdp
from src.assets.robots import C1_ACTION_SCALE, get_tahiti_c1_robot_cfg

# ============================================================================
# 1. ROBOT-SPECIFIC CONSTANTS  -- the things you are most likely to change
# ============================================================================

BASE_BODY = "base_link"
FOOT_SITES = ("left_foot", "right_foot")
FOOT_GEOMS = ("left_foot1_collision", "right_foot1_collision")
ANKLE_BODIES = r"^(left_ankle_roll_link|right_ankle_roll_link)$"

# Gait period [s]. Default for the other robots is 0.6. C1's loaded hip bandwidth
# is only 2.3-3.2 Hz (heavy X12-320 rotors), so 0.6 s (1.67 Hz) leaves 1.35x
# margin; 0.8 s gives ~1.9x. Must match between `phase` obs and `foot_gait` reward.
GAIT_PERIOD = 0.8

# Stance fraction of the gait cycle. 0.56 = slightly more stance than swing.
GAIT_STANCE_THRESHOLD = 0.56

# Swing-foot clearance target: ABSOLUTE world height of the foot site (the sole).
# G1 uses 0.10 m on ~0.6 m legs; C1's legs are 0.8 m, so this scales up.
FOOT_CLEARANCE = 0.12

# Commanded velocity ranges [m/s, rad/s]. Lower than G1: 53.5 kg on 0.8 s strides.
CMD_LIN_VEL_X = (-0.8, 1.2)
CMD_LIN_VEL_Y = (-0.5, 0.5)
CMD_ANG_VEL_Z = (-1.0, 1.0)

# Below this command magnitude the robot is asked to stand still.
CMD_STANDING_THRESHOLD = 0.1

# Terminate when the torso tilts past this.
FELL_OVER_ANGLE = math.radians(70.0)

# Observation history, frames per term. 1 = no history.
#
# The actor never observes base linear velocity (only the critic does), yet it
# is asked to track a velocity command -- so with no history it has no direct
# way to sense how fast it is travelling and must infer speed from joint state
# and the gait clock alone. History is how a policy recovers unobserved state,
# and it also gives it something to work with against the 0-30 ms actuator
# delay. The policy that previously ran on this hardware used 5.
#
# Costs: actor obs 47 -> 235, critic 62 -> 310. The deployment interface changes
# with it, so after training you must regenerate the deploy config, reconvert
# the MNN, and set `observations.history_length: 5` in policy_mjlab.yaml.
#
# robo_control computes its input size from the actual term list times this
# value, so 47 x 5 = 235 is handled correctly on the robot side.
OBS_HISTORY_LENGTH = 5

# Control rate: 0.005 s physics x 4 decimation = 50 Hz policy.
# NOTE: c1_constants.DELAY_MIN_LAG/DELAY_MAX_LAG count physics steps, so they
# depend on SIM_TIMESTEP. Change this and the actuator delay changes with it.
SIM_TIMESTEP = 0.005
DECIMATION = 4
EPISODE_LENGTH_S = 20.0

# The assembled hardware weighs this much more than the URDF (batteries, compute,
# wiring, covers). Added to base_link as a point mass at its COM -- which is the
# one case dr.body_mass is documented as valid for, since it leaves the inertia
# tensor alone. If you later learn WHERE the extra mass sits, put it in the URDF
# instead and shrink this range to the leftover uncertainty: domain randomisation
# should cover what you do not know, not a systematic offset you do know.
PAYLOAD_MASS_KG = (4.0, 6.0)

# --- startup domain randomisation, all multiplicative ------------------------
# Insurance against exactly the class of error that caused the first hardware
# failure: the deployed PD gains not matching what the policy trained on.
# pd_gains additionally covers the ankle linkage, whose effective joint gain is
# J^T diag(kp) J -- about 4.7x at the home pose, swinging 4.6-9.6 over a stride.
DR_KP_RANGE = (0.8, 1.2)  # +/-20%
DR_KD_RANGE = (0.8, 1.2)  # +/-20%
DR_FRICTION_RANGE = (0.8, 1.2)  # +/-20%
DR_ARMATURE_RANGE = (0.8, 1.2)  # +/-20%

# --- episode start-state scatter ---------------------------------------------
# Deliberately moderate. Measured (200 iters, 2048 envs): full scatter of
# +/-0.5 base velocity with +/-0.1 rad joint offsets cost early learning
# (ep_len 294 vs 339 for a softer setting), and stacking it with body_impulse
# stopped learning entirely. These sit between "off" and that full setting.
# Joint offsets are clamped to the soft limits (soft_joint_pos_limit_factor),
# so a scattered start cannot spawn against a hard stop.
RESET_BASE_VEL_RANGE = {
  "x": (-0.3, 0.3),
  "y": (-0.3, 0.3),
  "z": (-0.15, 0.15),
  "roll": (-0.3, 0.3),
  "pitch": (-0.3, 0.3),
  "yaw": (-0.3, 0.3),
}
RESET_JOINT_POS_RANGE = (-0.05, 0.05)  # rad, ~2.9 deg per joint
RESET_JOINT_VEL_RANGE = (-0.25, 0.25)  # rad/s

# Posture reward spreads, per joint regex. Wider std = more freedom.
# Knees/hip_pitch loosest (leg bending during stride); hip roll/yaw tighter
# (lateral sway); ankle roll tightest (balance); ankle pitch looser (clearance).
POSE_STD_STANDING = {".*": 0.05}
POSE_STD_WALKING = {
  r".*hip_pitch.*": 0.5,
  # hip_roll and hip_yaw loosened 0.15 -> 0.30, ankle_roll 0.10 -> 0.15.
  #
  # These are the joints that turning and side-stepping actually need, and at
  # std 0.15 the pose reward was paying the policy to keep them still. Measured
  # with the 5800-iteration checkpoint: turning at 0.8 rad/s used only 0.196 rad
  # of hip_yaw and strafing at 0.4 m/s only 0.100 rad of hip_roll -- the robot
  # was pivoting on planted feet instead of stepping round.
  #
  # The reward is exp(-(q/std)^2), so at std 0.15 a 0.30 rad step-turn scores
  # 0.018 for that joint -- essentially the whole term lost. At std 0.30 the
  # same excursion scores 0.37, which is affordable.
  #
  # (These values were inherited from G1's lower body; C1's hip_yaw range is
  # only +/-0.44 rad, so the same std is proportionally much tighter here.)
  r".*hip_roll.*": 0.30,
  r".*hip_yaw.*": 0.30,
  r".*knee.*": 0.5,
  r".*ankle_pitch.*": 0.15,
  r".*ankle_roll.*": 0.15,
}
# Running values ~1.5-2x walking, to accommodate a larger motion range.
POSE_STD_RUNNING = {
  r".*hip_pitch.*": 0.5,
  r".*hip_roll.*": 0.35,
  r".*hip_yaw.*": 0.35,
  r".*knee.*": 0.5,
  r".*ankle_pitch.*": 0.25,
  r".*ankle_roll.*": 0.15,
}


def tahiti_c1_rough_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Tahiti C1 velocity tracking on generated rough terrain."""

  # ==========================================================================
  # 2. SENSORS
  # ==========================================================================

  # Height scan grid in front of / around the robot. Rough terrain only; the
  # flat variant deletes this sensor and its observation terms.
  terrain_scan = RayCastSensorCfg(
    name="terrain_scan",
    frame=ObjRef(type="body", name=BASE_BODY, entity="robot"),
    ray_alignment="yaw",
    pattern=GridPatternCfg(size=(1.6, 1.0), resolution=0.1),
    max_distance=5.0,
    exclude_parent_body=True,
    debug_vis=True,
    viz=RayCastSensorCfg.VizCfg(show_normals=True),
  )

  # Foot-ground contact. Drives foot_gait / foot_slip / soft_landing rewards and
  # the foot_contact / foot_air_time critic observations.
  feet_ground_contact = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(mode="subtree", pattern=ANKLE_BODIES, entity="robot"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )

  # NOTE: no self-collision sensor. The MJCF disables self-collision entirely
  # (contype=0 on every geom) because its capsules are visual-mesh bounding-box
  # fits and adjacent joint housings overlap. See c1_constants.py.

  # ==========================================================================
  # 3. OBSERVATIONS
  # ==========================================================================
  # actor = what the deployed policy sees (47 dims on flat, 47+176 on rough).
  # critic = actor + privileged state, simulation only. Add anything you like
  # to the critic; adding to the actor means the real robot must provide it.

  actor_terms = {
    # 3 -- body angular velocity from the IMU gyro.
    "base_ang_vel": ObservationTermCfg(
      func=mdp.builtin_sensor,
      params={"sensor_name": "robot/imu_ang_vel"},
      # Raised 0.2 -> 0.3 for sim2real: a BNO08x on a walking biped sees
      # structural vibration on top of its datasheet noise floor.
      noise=Unoise(n_min=-0.3, n_max=0.3),
    ),
    # 3 -- gravity direction in body frame; tells the policy which way is up.
    "projected_gravity": ObservationTermCfg(
      func=mdp.projected_gravity,
      # 0.05 -> 0.08: this is a fused orientation ESTIMATE, not a raw reading,
      # so it carries filter lag and slow drift the sim does not model.
      noise=Unoise(n_min=-0.05, n_max=0.05),
    ),
    # 3 -- commanded (vx, vy, wz).
    "command": ObservationTermCfg(
      func=mdp.generated_commands,
      params={"command_name": "twist"},
    ),
    # 2 -- sin/cos gait clock, zeroed when the command is ~0.
    "phase": ObservationTermCfg(
      func=mdp.phase,
      params={"period": GAIT_PERIOD, "command_name": "twist"},
    ),
    # 12 -- joint positions relative to the default pose.
    "joint_pos": ObservationTermCfg(
      func=mdp.joint_pos_rel,
      # 0.01 -> 0.03. The 17-bit encoders are far better than this, but the
      # error that matters is calibration: hardware.yaml zero_offsets plus
      # <=15 arcmin (0.0044 rad) of gearbox backlash, which drifts with use.
      noise=Unoise(n_min=-0.03, n_max=0.03),
    ),
    # 12 -- joint velocities.
    "joint_vel": ObservationTermCfg(
      func=mdp.joint_vel_rel,
      # 1.5 -> 2.0. Joint velocity is finite-differenced from encoder counts on
      # hardware, which is considerably noisier than MuJoCo's exact qvel.
      noise=Unoise(n_min=-2.0, n_max=2.0),
    ),
    # 12 -- previous action.
    "actions": ObservationTermCfg(func=mdp.last_action),
    # 176 -- terrain height grid. Deleted by the flat variant.
    "height_scan": ObservationTermCfg(
      func=envs_mdp.height_scan,
      params={"sensor_name": "terrain_scan"},
      noise=Unoise(n_min=-0.1, n_max=0.1),
      scale=1 / terrain_scan.max_distance,
    ),
  }

  critic_terms = {
    **actor_terms,
    # 3 -- true base linear velocity (not observable on hardware).
    "base_lin_vel": ObservationTermCfg(
      func=mdp.builtin_sensor,
      params={"sensor_name": "robot/imu_lin_vel"},
      noise=Unoise(n_min=-0.5, n_max=0.5),
    ),
    # 176 -- noise-free height scan. Deleted by the flat variant.
    "height_scan": ObservationTermCfg(
      func=envs_mdp.height_scan,
      params={"sensor_name": "terrain_scan"},
      scale=1 / terrain_scan.max_distance,
    ),
    # 2 -- world z of each foot site.
    "foot_height": ObservationTermCfg(
      func=mdp.foot_height,
      params={"asset_cfg": SceneEntityCfg("robot", site_names=FOOT_SITES)},
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

  # History of OBS_HISTORY_LENGTH frames per term, flattened TERM-MAJOR:
  #   [A_t0..A_tH-1, B_t0..B_tH-1, ...]  (oldest -> newest within each term)
  # This matches policy_service::ObservationHistory::flatten() in robo_control
  # exactly, so the layout transfers to hardware without reordering. Verified
  # against mjlab's ObservationTermCfg.flatten_history_dim documentation.
  observations = {
    "actor": ObservationGroupCfg(
      terms=actor_terms,
      concatenate_terms=True,
      enable_corruption=True,  # apply the noise above; disabled in play mode
      history_length=OBS_HISTORY_LENGTH,
    ),
    "critic": ObservationGroupCfg(
      terms=critic_terms,
      concatenate_terms=True,
      enable_corruption=False,
      history_length=OBS_HISTORY_LENGTH,
    ),
  }

  # Logged only, never fed to the policy.
  metrics = {
    "mean_action_acc": MetricsTermCfg(func=mdp.mean_action_acc),
  }

  # ==========================================================================
  # 4. ACTIONS
  # ==========================================================================
  # 12 joint position targets: q_target = q_default + scale * action.
  # scale is per-joint, derived in c1_constants.py (0.35 rad, ankle_roll 0.14).

  actions: dict[str, ActionTermCfg] = {
    "joint_pos": JointPositionActionCfg(
      entity_name="robot",
      actuator_names=(".*",),
      scale=C1_ACTION_SCALE,
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
      rel_standing_envs=0.05,  # 5% of envs get a zero command
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
  commands["twist"].viz.z_offset = 1.05  # draw the arrow above the torso

  # ==========================================================================
  # 6. EVENTS (resets + domain randomisation)
  # ==========================================================================
  # mode="startup" runs once per env at build time (hardware variation),
  # mode="reset"   runs on every episode reset,
  # mode="interval" runs periodically during the episode.

  events = {
    # Scatter start position and heading.
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
        # Non-zero start velocity. Previously {} -- every episode began exactly
        # at rest, so the policy only ever saw states it had itself created.
        "velocity_range": RESET_BASE_VEL_RANGE,
      },
    ),
    # Scatter the starting posture. Directly relevant to the hardware
    # transition: at LB+Y the robot is in whatever pose READY left it in, with
    # real calibration offsets, not the sim's exact default.
    "reset_robot_joints": EventTermCfg(
      func=mdp.reset_joints_by_offset,
      mode="reset",
      params={
        "position_range": RESET_JOINT_POS_RANGE,
        "velocity_range": RESET_JOINT_VEL_RANGE,
        "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
      },
    ),
    # Random shove every 5-6 s. Teaches push recovery. Removed in play mode.
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
    # Ground friction varies per env; both feet share the same draw.
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
    # Per-joint encoder offset, +/- 0.86 deg. Models calibration error.
    "encoder_bias": EventTermCfg(
      mode="startup",
      func=dr.encoder_bias,
      params={
        "asset_cfg": SceneEntityCfg("robot"),
        "bias_range": (-0.015, 0.015),
      },
    ),
    # Hardware is 4-5 kg heavier than the URDF. Added to the torso as a point
    # mass at its COM. Paired with `base_com` below, which shifts where that
    # mass effectively sits.
    "payload_mass": EventTermCfg(
      mode="startup",
      func=dr.body_mass,
      params={
        "asset_cfg": SceneEntityCfg("robot", body_names=(BASE_BODY,)),
        "operation": "add",
        "ranges": PAYLOAD_MASS_KG,
      },
    ),
    # Torso centre-of-mass offset, +/- 5 cm on each axis. Models payload and
    # CAD error. Widen if the real robot carries a battery/compute you have not
    # modelled.
    # --- added for sim2real ------------------------------------------------
    # These three are startup-mode, so they model per-robot hardware variation
    # rather than disturbing the policy mid-episode. Measured (200 iters, 2048
    # envs): removing ALL startup DR changed learning not at all (ep_len 38 vs
    # 37), so they cost nothing in trainability. The per-step disturbances
    # (body_impulse, reset scatter) are deliberately NOT added -- those were
    # measured to stop the policy learning to walk at all.
    #
    # Scales each actuator group's own value, so X12 and X6 keep their relative
    # sizing and only the spread is shared.
    "pd_gains": EventTermCfg(
      mode="startup",
      func=dr.pd_gains,
      params={
        "asset_cfg": SceneEntityCfg("robot"),
        "kp_range": DR_KP_RANGE,
        "kd_range": DR_KD_RANGE,
        "operation": "scale",
      },
    ),
    # Dry friction (dof_frictionloss), set from the datasheet backdrive torque.
    # A single point estimate that drifts with temperature and wear.
    "joint_friction": EventTermCfg(
      mode="startup",
      func=dr.joint_friction,
      params={
        "asset_cfg": SceneEntityCfg("robot"),
        "operation": "scale",
        "ranges": DR_FRICTION_RANGE,
      },
    ),
    # Reflected inertia. Also carries the ankle linkage uncertainty: the sim
    # models the ankle as two serial joints, while the real four-bar reflects
    # ~4.7x more inertia into pitch and ~1.7x into roll.
    "joint_armature": EventTermCfg(
      mode="startup",
      func=dr.joint_armature,
      params={
        "asset_cfg": SceneEntityCfg("robot"),
        "operation": "scale",
        "ranges": DR_ARMATURE_RANGE,
      },
    ),
    "base_com": EventTermCfg(
      mode="startup",
      func=dr.body_com_offset,
      params={
        "asset_cfg": SceneEntityCfg("robot", body_names=(BASE_BODY,)),
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
  # Effective reward is `weight * func(...)`. Positive = encourage.
  # Watch these per-term in tensorboard under Episode_Reward/*: any term whose
  # magnitude dwarfs track_linear_velocity is dominating and probably wrong.

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
    # --- posture: stay upright and near the home pose ----------------------
    "body_orientation_l2": RewardTermCfg(
      func=mdp.body_orientation_l2,
      weight=-1.0,
      params={"asset_cfg": SceneEntityCfg("robot", body_names=(BASE_BODY,))},
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
    # 75% of C1's mass is in the legs (a human is ~32%), so swing-leg inertia is
    # the dominant trunk disturbance. Both weights are 2x the other robots'.
    "body_ang_vel": RewardTermCfg(
      func=mdp.body_angular_velocity_penalty,
      weight=-0.1,
      params={"asset_cfg": SceneEntityCfg("robot", body_names=(BASE_BODY,))},
    ),
    "angular_momentum": RewardTermCfg(
      func=mdp.angular_momentum_penalty,
      weight=-0.05,
      params={"sensor_name": "robot/root_angmom"},
    ),
    # --- safety / smoothness ----------------------------------------------
    "is_terminated": RewardTermCfg(func=mdp.is_terminated, weight=-200.0),
    "joint_acc_l2": RewardTermCfg(func=mdp.joint_acc_l2, weight=-2.5e-7),
    "joint_pos_limits": RewardTermCfg(func=mdp.joint_pos_limits, weight=-10.0),
    "action_rate_l2": RewardTermCfg(func=mdp.action_rate_l2, weight=-0.05),
    # --- gait shaping ------------------------------------------------------
    # Rewards contact matching the clock. offset [0.0, 0.5] = the two feet are
    # half a cycle apart, i.e. alternating steps.
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
    # Penalises |foot_z - target| weighted by foot speed: only bites while the
    # foot is actually moving, so it shapes swing height, not stance.
    "foot_clearance": RewardTermCfg(
      func=mdp.feet_clearance,
      weight=-1.0,
      params={
        "target_height": FOOT_CLEARANCE,
        "command_name": "twist",
        "command_threshold": CMD_STANDING_THRESHOLD,
        "asset_cfg": SceneEntityCfg("robot", site_names=FOOT_SITES),
      },
    ),
    # -0.75, up from -0.25. At the old weight this contributed only -0.0149
    # against a measured 0.70 m/s of foot slip during forward walking, so
    # sliding was barely discouraged. Raised alongside the looser hip stds:
    # loosening pose makes stepping affordable, this makes sliding expensive.
    "foot_slip": RewardTermCfg(
      func=mdp.feet_slip,
      weight=-0.75,
      params={
        "sensor_name": feet_ground_contact.name,
        "command_name": "twist",
        "command_threshold": CMD_STANDING_THRESHOLD,
        "asset_cfg": SceneEntityCfg("robot", site_names=FOOT_SITES),
      },
    ),
    # Penalises impact force at touchdown.
    "soft_landing": RewardTermCfg(
      func=mdp.soft_landing,
      weight=-1e-3,
      params={
        "sensor_name": feet_ground_contact.name,
        "command_name": "twist",
        "command_threshold": CMD_STANDING_THRESHOLD,
      },
    ),
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
  # time_out=True marks truncation (bootstrap the value), not failure.

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
    # Promote envs that walked far enough to harder terrain tiles; demote the
    # ones that did not. Removed by the flat variant.
    "terrain_levels": CurriculumTermCfg(
      func=mdp.terrain_levels_vel,
      params={"command_name": "twist"},
    ),
    # Widen the command ranges once the policy can walk at all.
    # `step` counts environment steps: 5000 iterations x 24 steps_per_env.
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
  # 10. ASSEMBLY
  # ==========================================================================

  cfg = ManagerBasedRlEnvCfg(
    scene=SceneCfg(
      entities={"robot": get_tahiti_c1_robot_cfg()},
      terrain=TerrainEntityCfg(
        terrain_type="generator",
        terrain_generator=replace(ROUGH_TERRAINS_CFG, curriculum=True),
        max_init_terrain_level=5,
      ),
      sensors=(terrain_scan, feet_ground_contact),
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
      body_name=BASE_BODY,
      distance=3.0,
      elevation=-5.0,
      azimuth=90.0,
    ),
    sim=SimulationCfg(
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
    _apply_play_overrides(cfg, rough=True)
  return cfg


# ============================================================================
# 11. FLAT VARIANT + PLAY OVERRIDES
# ============================================================================


def tahiti_c1_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Tahiti C1 velocity tracking on flat ground.

  Identical to the rough config except for the five changes below. Train this
  one first: rough terrain additionally needs the height-scan observation and
  the terrain curriculum working, and you do not want to debug both at once.
  """
  cfg = tahiti_c1_rough_env_cfg(play=False)

  # (1) Flat plane instead of a generated terrain.
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  # (2) No terrain to scan, so drop the raycast sensor...
  cfg.scene.sensors = tuple(
    s for s in (cfg.scene.sensors or ()) if s.name != "terrain_scan"
  )
  # (3) ...and the observations that read it. Actor drops 47+176 -> 47.
  del cfg.observations["actor"].terms["height_scan"]
  del cfg.observations["critic"].terms["height_scan"]

  # (4) No terrain levels to progress through.
  cfg.curriculum.pop("terrain_levels", None)

  # (5) Far fewer contacts without terrain geometry.
  cfg.sim.njmax = 300
  cfg.sim.nconmax = None
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.mujoco.ccd_iterations = 50

  if play:
    _apply_play_overrides(cfg, rough=False)
  return cfg


def _apply_play_overrides(cfg: ManagerBasedRlEnvCfg, rough: bool) -> None:
  """Turn a training config into a viewing config (scripts/play.py)."""
  # Never time out, so you can watch one episode indefinitely.
  cfg.episode_length_s = int(1e9)
  # Show the policy's true behaviour: no observation noise, no shoving.
  cfg.observations["actor"].enable_corruption = False
  cfg.events.pop("push_robot", None)
  cfg.curriculum = {}
  # Spread the robots over the terrain tiles instead of stacking them.
  cfg.events["randomize_terrain"] = EventTermCfg(
    func=envs_mdp.randomize_terrain,
    mode="reset",
    params={},
  )

  if rough:
    if cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None:
      cfg.scene.terrain.terrain_generator.curriculum = False
      cfg.scene.terrain.terrain_generator.num_cols = 5
      cfg.scene.terrain.terrain_generator.num_rows = 5
      cfg.scene.terrain.terrain_generator.border_width = 10.0
  else:
    # Gentler commands make the gait easier to inspect.
    twist_cmd = cfg.commands["twist"]
    assert isinstance(twist_cmd, UniformVelocityCommandCfg)
    twist_cmd.ranges.lin_vel_x = (-0.4, 0.8)
    twist_cmd.ranges.lin_vel_y = (-0.3, 0.3)
    twist_cmd.ranges.ang_vel_z = (-0.5, 0.5)
