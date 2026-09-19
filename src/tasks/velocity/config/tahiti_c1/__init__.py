from mjlab.tasks.registry import register_mjlab_task
from src.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import (
  tahiti_c1_flat_env_cfg,
  tahiti_c1_rough_env_cfg,
)
from .rl_cfg import tahiti_c1_ppo_runner_cfg

register_mjlab_task(
  task_id="Tahiti-C1-Rough",
  env_cfg=tahiti_c1_rough_env_cfg(),
  play_env_cfg=tahiti_c1_rough_env_cfg(play=True),
  rl_cfg=tahiti_c1_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Tahiti-C1-Flat",
  env_cfg=tahiti_c1_flat_env_cfg(),
  play_env_cfg=tahiti_c1_flat_env_cfg(play=True),
  rl_cfg=tahiti_c1_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
