"""Tahiti C1 constants -- HAND-TUNED variant.

Sibling of c1_constants.py. Same robot, same MJCF; the difference is where the
actuator numbers come from:

  c1_constants.py        derived from the MyActuator datasheets
                         (doc/actuator_tuning.md: armature -> omega/zeta -> Kp/Kd)

  c1_tuned_constants.py  transcribed from the IsaacLab DelayedPDActuatorCfg that
      <-- this file      was already validated on this hardware, and whose gains
                         match the working robo_control/config/yaml/policy.yaml

Why this exists: the derived config failed sim2real -- the robot vibrated and
fell on entering POLICY mode. Comparing what actually reached the motors against
the previously-working config:

    sent to motors     OLD (worked)      DERIVED      ratio
    hip/knee kp              250.0        228.571     0.91x
    hip/knee kd               25.0         24.019     0.96x
    ANKLE kp                 200.0         42.857     0.21x   <-- 5x softer
    ANKLE kd                  20.0          2.279     0.11x   <-- 9x less damped

The legs were within 10% of proven values; the ankles were severely
under-damped, which is a textbook vibration source on a biped. This file adopts
the proven numbers wholesale rather than deriving them.

NOTE: gains are part of the plant the policy learns to control, so switching to
this file REQUIRES retraining. Changing only the robot-side YAML would hand the
policy a different robot than it trained on.
"""

from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg, DelayedActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.actuator import ElectricActuator
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg
from src import SRC_PATH

##
# MJCF and assets.
##

C1_XML: Path = SRC_PATH / "assets" / "robots" / "tahiti_c1" / "xmls" / "tahiti_c1.xml"
assert C1_XML.exists()

##
# Actuator parameters, transcribed from the IsaacLab asset config.
#
# Every value below is a measured/tuned quantity, NOT a derived one. Notable
# differences from the datasheet-derived config:
#
#   effort_limit  170 / 40 Nm  = 2x the RATED torque (85 / 20), still well under
#                                the 320 / 60 peak.
#
#                                Originally set to rated, which FAILED: measured
#                                with the 8200-iteration checkpoint, ankle pitch
#                                sat at its 20 Nm limit 23% of the time and the
#                                policy collapsed to standing still. Rated is the
#                                CONTINUOUS thermal limit; push-off and balance
#                                recovery are sub-second transients where the
#                                motor really does deliver far more. Training at
#                                rated forbids the burst torque that balance
#                                needs.
#
#                                NOTE: check the driver-side torque clamp in
#                                robo_control matches. If the driver limits at
#                                rated, the policy will request torque the
#                                hardware will not deliver.
#   armature      0.326938     = 0.633x the datasheet-derived 0.516 (X12)
#                 0.019603     = 0.772x the datasheet-derived 0.025386 (X6)
#                                Not a constant ratio, so these look like system
#                                -identified values rather than J_rotor * N^2.
#                                They supersede the rotor-vs-output-side
#                                question the derived config had to guess at.
#   friction                   = dry / Coulomb friction  -> joint frictionloss
#   viscous_friction           = velocity-proportional    -> joint damping
#                                (mjlab's actuator cfg has no field for this, so
#                                it is applied to the joints in get_spec below)
##

# --- X12-320: hip yaw/pitch/roll + knee (8 joints) ---
X12_STIFFNESS = 250.0
X12_DAMPING = 25.0
X12_EFFORT_LIMIT = 170.0  # 2x rated (85); peak is 320
X12_VELOCITY_LIMIT = 10.0
X12_ARMATURE = 0.326938
X12_FRICTION = 1.694307  # dry
X12_VISCOUS_FRICTION = 0.350134

# --- X6-60: ankle pitch + roll (4 joints) ---
X6_STIFFNESS = 200.0
X6_DAMPING = 20.0
X6_EFFORT_LIMIT = 40.0  # 2x rated (20); peak is 60
X6_VELOCITY_LIMIT = 16.0
X6_ARMATURE = 0.019603
X6_FRICTION = 0.321816  # dry
X6_VISCOUS_FRICTION = 0.165640

X12_JOINTS = (
  ".*_hip_yaw_joint",
  ".*_hip_pitch_joint",
  ".*_hip_roll_joint",
  ".*_knee_joint",
)
X6_JOINTS = (".*_ankle_pitch_joint", ".*_ankle_roll_joint")

# Viscous friction per joint-name substring, applied in get_spec().
_VISCOUS_BY_SUBSTR = {
  "hip_yaw": X12_VISCOUS_FRICTION,
  "hip_pitch": X12_VISCOUS_FRICTION,
  "hip_roll": X12_VISCOUS_FRICTION,
  "knee": X12_VISCOUS_FRICTION,
  "ankle_pitch": X6_VISCOUS_FRICTION,
  "ankle_roll": X6_VISCOUS_FRICTION,
}


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, C1_XML.parent / "meshes", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  """Load the MJCF and stamp in viscous friction as passive joint damping.

  MuJoCo's joint `damping` is passive velocity-proportional resistance, which is
  exactly IsaacLab's `viscous_friction`. It is distinct from the actuator's `kd`
  (a control gain) and from `frictionloss` (load-independent dry friction), so
  all three coexist.
  """
  spec = mujoco.MjSpec.from_file(str(C1_XML))
  spec.assets = get_assets(spec.meshdir)
  for joint in spec.joints:
    if joint.type != mujoco.mjtJoint.mjJNT_HINGE:
      continue
    for substr, viscous in _VISCOUS_BY_SUBSTR.items():
      if substr in joint.name:
        joint.damping = viscous
        break
  return spec


##
# Actuator config.
#
# Delay wrapper carries over unchanged from the IsaacLab config: min_delay=0,
# max_delay=6. Both simulators count PHYSICS steps and both run 200 Hz physics,
# so the values transfer 1:1 -- verified empirically (a fixed lag of 6 holds a
# step command back 6 physics steps before it reaches mjModel.ctrl).
##

DELAY_MIN_LAG = 0
DELAY_MAX_LAG = 6

ACTUATOR_X12 = ElectricActuator(
  reflected_inertia=X12_ARMATURE,
  velocity_limit=X12_VELOCITY_LIMIT,
  effort_limit=X12_EFFORT_LIMIT,
)
ACTUATOR_X6 = ElectricActuator(
  reflected_inertia=X6_ARMATURE,
  velocity_limit=X6_VELOCITY_LIMIT,
  effort_limit=X6_EFFORT_LIMIT,
)

_PD_X12 = BuiltinPositionActuatorCfg(
  target_names_expr=X12_JOINTS,
  stiffness=X12_STIFFNESS,
  damping=X12_DAMPING,
  effort_limit=X12_EFFORT_LIMIT,
  armature=X12_ARMATURE,
  frictionloss=X12_FRICTION,
)
_PD_X6 = BuiltinPositionActuatorCfg(
  target_names_expr=X6_JOINTS,
  stiffness=X6_STIFFNESS,
  damping=X6_DAMPING,
  effort_limit=X6_EFFORT_LIMIT,
  armature=X6_ARMATURE,
  frictionloss=X6_FRICTION,
)

C1_TUNED_ACTUATOR_X12 = DelayedActuatorCfg(
  base_cfg=_PD_X12,
  delay_target="position",
  delay_min_lag=DELAY_MIN_LAG,
  delay_max_lag=DELAY_MAX_LAG,
)
C1_TUNED_ACTUATOR_X6 = DelayedActuatorCfg(
  base_cfg=_PD_X6,
  delay_target="position",
  delay_min_lag=DELAY_MIN_LAG,
  delay_max_lag=DELAY_MAX_LAG,
)

##
# Keyframe config.
##

# Matches robo_control/config/yaml/mode_service.yaml `ready_joint_pos` exactly,
# so the robot does not jump at the READY -> POLICY transition. A mismatch here
# was one of the suspected contributors to the failed bring-up: the derived
# config used -0.225 / 0.45 / 0.225, which is ~0.15 rad away from what the
# controller holds in READY, giving a torque spike the instant policy starts.
#
# Also cheaper on torque than the deeper crouch, which matters now that
# effort_limit is the 85 Nm rated figure rather than the 320 Nm peak
# (single-support, % of rated):
#
#                    knee          hip_pitch      ankle_pitch
#   -0.225/0.45     35.8 Nm  42%    15.4 Nm 18%    11.8 Nm 59%
#   -0.1/0.3 (this) 30.6 Nm  36%     5.3 Nm  6%    11.8 Nm 59%
#
# Trade-off accepted: 17 deg of knee bend instead of 26 deg, so less margin from
# the knee singularity and less impact absorption. The torque numbers say the
# straighter pose is not stressing anything, and it is the pose the previously
# working policy used on this hardware.
#
# z is measured, not assumed: lowest foot-sole geom sits 0.9316 m below the base
# at this pose (scripts/check_actuators.py style probe on tahiti_c1.xml).
HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.9316),
  joint_pos={
    ".*_hip_pitch_joint": -0.1,
    ".*_knee_joint": 0.3,
    ".*_ankle_pitch_joint": 0.2,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

_FOOT_REGEX = r"^(left|right)_foot[1-7]_collision$"

# Self-collision stays OFF: the MJCF capsules are visual-mesh bounding-box fits
# and hip_yaw interpenetrates hip_roll by ~28 mm at the home pose.
FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
  geom_names_expr=(".*_collision",),
  contype=0,
  conaffinity=1,
  condim={_FOOT_REGEX: 3, ".*_collision": 1},
  priority={_FOOT_REGEX: 1},
  friction={_FOOT_REGEX: (0.6,)},
)

FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(_FOOT_REGEX,),
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
)

##
# Final config.
##

# 0.85, matching IsaacLab (the derived config used 0.9). Tighter soft limits
# keep the policy further from the hard stops.
SOFT_JOINT_POS_LIMIT_FACTOR = 0.85

C1_TUNED_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(C1_TUNED_ACTUATOR_X12, C1_TUNED_ACTUATOR_X6),
  soft_joint_pos_limit_factor=SOFT_JOINT_POS_LIMIT_FACTOR,
)


def get_tahiti_c1_tuned_robot_cfg() -> EntityCfg:
  """Get a fresh hand-tuned Tahiti C1 robot configuration instance."""
  return EntityCfg(
    init_state=HOME_KEYFRAME,
    collisions=(FULL_COLLISION_WITHOUT_SELF,),
    spec_fn=get_spec,
    articulation=C1_TUNED_ARTICULATION,
  )


##
# Action scale.
#
# NOT derived here. 0.25 * effort / stiffness would give 0.085 rad (legs) and
# 0.025 rad (ankles) -- far below the 0.15-0.5 healthy band, because the rated
# effort limits are small relative to the hand-tuned gains. Instead these are
# the proven values from robo_control/config/yaml/policy.yaml, which the
# previously-working policy used on this hardware.
##

C1_TUNED_ACTION_SCALE: dict[str, float] = {}
for _n in X12_JOINTS:
  C1_TUNED_ACTION_SCALE[_n] = 0.25
for _n in X6_JOINTS:
  C1_TUNED_ACTION_SCALE[_n] = 0.10


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_tahiti_c1_tuned_robot_cfg())

  viewer.launch(robot.spec.compile())
