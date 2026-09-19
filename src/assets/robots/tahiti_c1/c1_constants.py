"""Tahiti C1 constants.

12 DOF biped, 53.52 kg, no upper body. Actuators are MyActuator servo modules:
X12-320 on hip yaw/pitch/roll and knee, X6-60 on ankle pitch/roll.

Every gain below is derived, not hand-tuned. See doc/actuator_tuning.md for the
method, and run ``python scripts/check_actuators.py`` to re-verify after any change
to the model, the home pose, or the control rate.
"""

import math
from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg, DelayedActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.actuator import ElectricActuator, reflected_inertia, rpm_to_rad
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg
from src import SRC_PATH

##
# MJCF and assets.
##

# Generated from tahiti_c1.urdf by scripts/urdf_to_mjcf.py. Do not hand-edit:
# re-run that script when the URDF changes.
C1_XML: Path = SRC_PATH / "assets" / "robots" / "tahiti_c1" / "xmls" / "tahiti_c1.xml"
assert C1_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, C1_XML.parent / "meshes", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(C1_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Actuator config.
##

# Datasheet values (MyActuator). "Moment of inertia" is read as ROTOR-side, i.e.
# referred to the motor shaft before the gearbox -- see doc/actuator_tuning.md §3
# for the three tests that settle this. Reading it as output-side would make every
# armature below 400x too small.
ROTOR_INERTIA_X12 = 12.9e-4  # kg.m^2 (12.9 kg.cm^2)
GEAR_X12 = 20.0
ROTOR_INERTIA_X6 = 0.66e-4  # kg.m^2 (0.66 kg.cm^2)
GEAR_X6 = 19.612

ARMATURE_X12 = reflected_inertia(ROTOR_INERTIA_X12, GEAR_X12)  # 0.51600
ARMATURE_X6 = reflected_inertia(ROTOR_INERTIA_X6, GEAR_X6)  # 0.02539

ACTUATOR_X12 = ElectricActuator(
  reflected_inertia=ARMATURE_X12,
  velocity_limit=rpm_to_rad(125.0),  # 13.09 rad/s no-load
  effort_limit=320.0,  # peak; rated is 85 Nm (see §6 on the thermal budget)
)
ACTUATOR_X6 = ElectricActuator(
  reflected_inertia=ARMATURE_X6,
  velocity_limit=rpm_to_rad(176.0),  # 18.43 rad/s no-load
  effort_limit=60.0,  # peak; rated is 20 Nm
)

# Backdrive torque from the datasheet, used as dry friction.
FRICTIONLOSS_X12 = 3.8
FRICTIONLOSS_X6 = 1.6

##
# Gains.
#
# Kp is set by the torque/range target rather than by a natural frequency:
#
#     Kp = effort_limit / THETA_SAT
#
# where THETA_SAT is the tracking error at which the motor saturates. 1.4 rad
# matches G1's leg joints and yields a 0.35 rad action scale on every joint.
#
#     Kd = 2 * ZETA * sqrt(Kp * I_ref)
#
# I_ref = armature + I_load is the TOTAL inertia at the joint, using the median
# over each actuator group (spread is only 2.0x / 1.3x, so one pair per group is
# enough). The medians below are measured at the home pose by check_actuators.py.
#
# ZETA is 1.0, not G1's 2.0: here the armature dominates the load, so the typed
# damping ratio survives to the joint instead of being halved by it. Raising it
# would also breach the Kd*v_noload check -- 24.02 * 13.09 = 314 of a 320 Nm limit.
##

THETA_SAT = 1.4  # rad
ZETA = 1.0

I_REF_X12 = 0.6310  # kg.m^2, median over hip yaw/pitch/roll + knee
I_REF_X6 = 0.0303  # kg.m^2, median over ankle pitch/roll

STIFFNESS_X12 = ACTUATOR_X12.effort_limit / THETA_SAT  # 228.6
DAMPING_X12 = 2.0 * ZETA * math.sqrt(STIFFNESS_X12 * I_REF_X12)  # 24.02

STIFFNESS_X6 = ACTUATOR_X6.effort_limit / THETA_SAT  # 42.9
DAMPING_X6 = 2.0 * ZETA * math.sqrt(STIFFNESS_X6 * I_REF_X6)  # 2.28

_C1_PD_X12 = BuiltinPositionActuatorCfg(
  target_names_expr=(
    ".*_hip_yaw_joint",
    ".*_hip_pitch_joint",
    ".*_hip_roll_joint",
    ".*_knee_joint",
  ),
  stiffness=STIFFNESS_X12,
  damping=DAMPING_X12,
  effort_limit=ACTUATOR_X12.effort_limit,
  armature=ACTUATOR_X12.reflected_inertia,
  frictionloss=FRICTIONLOSS_X12,
)
_C1_PD_X6 = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_ankle_pitch_joint", ".*_ankle_roll_joint"),
  stiffness=STIFFNESS_X6,
  damping=DAMPING_X6,
  effort_limit=ACTUATOR_X6.effort_limit,
  armature=ACTUATOR_X6.reflected_inertia,
  frictionloss=FRICTIONLOSS_X6,
)

##
# Actuator delay (sim2real).
#
# The real chain -- policy -> master -> bus -> slave driver -> current loop --
# does not apply a command the instant it is computed. Training without that lag
# produces a policy that relies on instantaneous response and oscillates on
# hardware. DelayedActuatorCfg buffers the position target to reproduce it.
#
# Lag is quantised to PHYSICS timesteps, not control steps:
#     1 lag unit = SIM_TIMESTEP = 5 ms   (one control step = 4 units = 20 ms)
#
# >>> SET THIS FROM YOUR MEASUREMENT <<<
# Put the round-trip command latency you measured master->slave here, in ms.
# The range is sampled per environment so the policy cannot exploit one exact
# value; keep a spread even if your measurement is tight.
##

# Measured on hardware (master -> slave chain), carried over from the IsaacLab
# DelayedPDActuatorCfg setup. The units are identical in both simulators:
# min_delay / max_delay there and delay_min_lag / delay_max_lag here both count
# PHYSICS steps, and both run 200 Hz physics, so the values transfer 1:1.
#
# Verified empirically in this repo, not just read off the docstring: with
# decimation=4 and a fixed lag of 6, a step change in the action first reaches
# mjModel.ctrl at control step 2 (8 physics steps), i.e. it was held back 6
# physics steps. With lag 0 it lands at control step 1.
#
#   physics_dt = 0.005 s  ->  1 lag = 5 ms
#   lag 0..6              ->  0..30 ms   (0 .. 1.5 control steps at 50 Hz)
#
# The lag is resampled per environment, so the policy has to tolerate the whole
# band instead of learning to compensate one fixed latency.
DELAY_MIN_LAG = 0
DELAY_MAX_LAG = 6

_MS_PER_LAG = 1000.0 * 0.005  # SIM_TIMESTEP; keep in sync with the env cfg
DELAY_MIN_MS = DELAY_MIN_LAG * _MS_PER_LAG  # 0 ms
DELAY_MAX_MS = DELAY_MAX_LAG * _MS_PER_LAG  # 30 ms

C1_ACTUATOR_X12 = DelayedActuatorCfg(
  base_cfg=_C1_PD_X12,
  delay_target="position",
  delay_min_lag=DELAY_MIN_LAG,
  delay_max_lag=DELAY_MAX_LAG,
)
C1_ACTUATOR_X6 = DelayedActuatorCfg(
  base_cfg=_C1_PD_X6,
  delay_target="position",
  delay_min_lag=DELAY_MIN_LAG,
  delay_max_lag=DELAY_MAX_LAG,
)

##
# Keyframe config.
##

# Shallow crouch. A deeper one costs knee torque fast: at 0.6 rad the knee already
# draws 60% of its *rated* (not peak) torque just standing. z comes from the foot
# sole sitting 0.920 m below the base at this pose.
HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.92),
  joint_pos={
    ".*_hip_pitch_joint": -0.225,
    ".*_knee_joint": 0.45,
    ".*_ankle_pitch_joint": 0.225,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

_FOOT_REGEX = r"^(left|right)_foot[1-7]_collision$"

# Self-collision is OFF (contype=0 on every geom, so robot geoms never pair with
# each other). The body capsules in the MJCF are fitted to visual-mesh bounding
# boxes and adjacent joint housings overlap -- hip_yaw interpenetrates hip_roll by
# ~28 mm at the home pose, and they are not a parent-child pair so MuJoCo does not
# exclude them automatically. Tighten those capsules to real CAD envelopes before
# turning self-collision on.
FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
  geom_names_expr=(".*_collision",),
  contype=0,
  conaffinity=1,
  condim={_FOOT_REGEX: 3, ".*_collision": 1},
  priority={_FOOT_REGEX: 1},
  friction={_FOOT_REGEX: (0.6,)},
)

# Feet only: everything above the ankle passes through the terrain.
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

C1_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(C1_ACTUATOR_X12, C1_ACTUATOR_X6),
  soft_joint_pos_limit_factor=0.9,
)


def get_tahiti_c1_robot_cfg() -> EntityCfg:
  """Get a fresh Tahiti C1 robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=HOME_KEYFRAME,
    collisions=(FULL_COLLISION_WITHOUT_SELF,),
    spec_fn=get_spec,
    articulation=C1_ARTICULATION,
  )


# scale = 0.25 * effort_limit / stiffness, so a +/-1 action commands 25% of peak
# torque on every joint regardless of its gains (doc §7). Comes out at 0.35 rad
# for both motor groups.
C1_ACTION_SCALE: dict[str, float] = {}
for _a in C1_ARTICULATION.actuators:
  # Unwrap the delay wrapper: the scale depends on the PD gains underneath it.
  _pd = _a.base_cfg if isinstance(_a, DelayedActuatorCfg) else _a
  assert isinstance(_pd, BuiltinPositionActuatorCfg)
  assert _pd.effort_limit is not None
  for _n in _pd.target_names_expr:
    C1_ACTION_SCALE[_n] = 0.25 * _pd.effort_limit / _pd.stiffness

# Ankle roll only travels +/-0.17 rad, so the torque-derived 0.35 would let the
# policy spend its whole action range outside the hard stops. Cap at 0.4 x range.
C1_ACTION_SCALE[".*_ankle_roll_joint"] = 0.14


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_tahiti_c1_robot_cfg())

  viewer.launch(robot.spec.compile())
