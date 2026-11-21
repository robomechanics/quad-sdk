"""Batched quasi-static support-force allocation for raised-leg preparation.

The force controller is confined to the observed preparation substate. It uses
normal Go2 position actuators; it never writes physical root or joint states.
"""
from dataclasses import dataclass
import torch


@dataclass
class PreparationConfig:
    body_height: float = .28
    shift_seconds: float = 2.0
    lift_seconds: float = 1.5
    hold_seconds: float = 1.0
    timeout_seconds: float = 10.0
    lean_roll: float = .06
    lean_pitch: float = -.04
    unload_force: float = 8.0
    unload_seconds: float = .1
    max_body_speed: float = .06
    max_angular_speed: float = .25
    max_foot_speed: float = .06


def support_forces(feet, com, force, torque, weights):
    """Regularized weighted force allocation; nonnegative normal/friction bounds."""
    r = feet-com[:, None]
    n = r.shape[0]
    skew = torch.zeros(n, 4, 3, 3, device=r.device, dtype=r.dtype)
    skew[..., 0, 1], skew[..., 0, 2] = -r[..., 2], r[..., 1]
    skew[..., 1, 0], skew[..., 1, 2] = r[..., 2], -r[..., 0]
    skew[..., 2, 0], skew[..., 2, 1] = -r[..., 1], r[..., 0]
    linear = torch.eye(3, device=r.device).expand(n, 4, 3, 3)
    a = torch.cat((linear, skew), dim=2).permute(0, 2, 1, 3).reshape(n, 6, 12)
    w = weights.repeat_interleave(3, dim=-1)
    aw = a*w[:, None]
    wrench = torch.cat((force, torque), -1)
    forces = aw.transpose(1, 2) @ torch.linalg.solve(
        aw @ a.transpose(1, 2)+1e-5*torch.eye(6, device=r.device)[None], wrench[..., None])
    forces = forces.reshape(n, 4, 3)
    forces[..., 2].clamp_(0, 150)
    horizontal = forces[..., :2].norm(dim=-1, keepdim=True).clamp_min(1e-6)
    forces[..., :2] *= (.5*forces[..., 2, None]/horizontal).clamp(max=1)
    return forces
