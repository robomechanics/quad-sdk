"""Frozen contact-free CPG adapter. Requires the packaged Conga root on sys.path.

Support values are schedule flags, never physical contact estimates. Keep this
adapter numerically identical to the frozen experiment (see the bundled report).
"""
from dataclasses import replace
import numpy as np
import torch
from deploy.cpg.runtime import DanceRuntime

class ContactFreeRuntime(DanceRuntime):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        c = self.controller
        update = c.dance.update
        def timed_update(dt, ready, contacts, success, failure, velocity):
            due = c.dance.time >= c.dance.target_time - 1e-6
            moving = (c.dance.kind == c.dance.STEP) | (c.dance.kind == c.dance.KICK)
            return update(dt, ready, torch.ones_like(contacts), due & moving,
                          torch.zeros_like(failure), velocity)
        c.dance.update = timed_update

    def observe(self, frame, velocity, diagnostic_truth=None):
        if diagnostic_truth is not None:
            raise ValueError('Truth feedback is forbidden')
        d = self.controller.dance
        support = np.ones(4, dtype=bool)
        kind = d.kind.item()
        if kind in (d.STEP, d.KICK, d.HOLD):
            airborne = kind in (d.KICK, d.HOLD) or d.time.item() < d.target_time.item() - 1e-6
            if airborne:
                support[d.selected[0].numpy()] = False
        return super().observe(replace(frame, foot_force=support.astype(float) * 20.), velocity)
