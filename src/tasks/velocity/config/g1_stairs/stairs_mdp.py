"""Stair-specific observation and reward terms for the G1 stairs task.

Kept next to the task (not in src/tasks/velocity/mdp) so the shared library,
and every other robot's task, stays untouched.

Why these exist
---------------
The shared `feet_clearance` reward and `foot_height` observation use the foot's
ABSOLUTE world height. On flat ground that equals height above the ground, but
on stairs it does not: a robot standing on the 5th step (z = 0.6 m) would be
penalised on every swing for being "too high", which teaches it to avoid
climbing. The terms below measure foot height from the terrain directly under
each foot, using one downward ray per foot.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from mjlab.entity import Entity
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactSensor, RayCastSensor

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


def terrain_relative_foot_height(
  env: ManagerBasedRlEnv,
  sensor_names: tuple[str, ...],
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """Height of each foot site above the terrain directly below it. [B, N]

  `sensor_names[i]` is a single-ray RayCastSensor under the i-th site in
  `asset_cfg.site_names` (same order). A ray that misses (e.g. over the edge of
  the terrain) falls back to the site's absolute height.
  """
  asset: Entity = env.scene[asset_cfg.name]
  foot_z = asset.data.site_pos_w[:, asset_cfg.site_ids, 2]  # [B, N]
  ground_z = torch.zeros_like(foot_z)
  for i, name in enumerate(sensor_names):
    sensor: RayCastSensor = env.scene[name]
    hit = sensor.data.distances[:, 0] >= 0.0
    ground_z[:, i] = torch.where(hit, sensor.data.hit_pos_w[:, 0, 2], 0.0)
  return foot_z - ground_z


def feet_clearance_terrain(
  env: ManagerBasedRlEnv,
  target_height: float,
  sensor_names: tuple[str, ...],
  command_name: str,
  command_threshold: float = 0.1,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """Swing-height cost: |height_above_terrain - target| * foot_speed, summed.

  Same formula as the shared `feet_clearance`, but the height is measured from
  the step under the foot. Weighting by horizontal foot speed means it only
  acts on swinging feet, not on the stance foot. Zero when the command is ~0.
  """
  asset: Entity = env.scene[asset_cfg.name]
  foot_z = terrain_relative_foot_height(env, sensor_names, asset_cfg)  # [B, N]
  foot_vel_xy = asset.data.site_lin_vel_w[:, asset_cfg.site_ids, :2]  # [B, N, 2]
  vel_norm = torch.norm(foot_vel_xy, dim=-1)  # [B, N]
  cost = torch.sum(torch.abs(foot_z - target_height) * vel_norm, dim=1)  # [B]

  command = env.command_manager.get_command(command_name)
  assert command is not None
  total_command = torch.norm(command[:, :2], dim=1) + torch.abs(command[:, 2])
  return cost * (total_command > command_threshold).float()


def feet_stumble(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  ratio: float = 4.0,
) -> torch.Tensor:
  """1.0 when any foot is pushed mostly sideways, i.e. it hit a stair riser.

  A foot resting on a tread gets a mostly vertical contact force. A toe that
  kicks the vertical face of a step gets a mostly horizontal one. Flags the
  latter when |F_xy| > ratio * |F_z|. Needs the sensor's force in world frame,
  which ContactSensor provides when reduce="netforce".
  """
  sensor: ContactSensor = env.scene[sensor_name]
  force = sensor.data.force
  assert force is not None  # [B, N, 3], world frame
  horizontal = torch.norm(force[..., :2], dim=-1)  # [B, N]
  vertical = torch.abs(force[..., 2])  # [B, N]
  return (horizontal > ratio * vertical).any(dim=1).float()
