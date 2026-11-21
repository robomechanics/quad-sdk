"""Full Go2 walking/dance runtime; input contains no simulator root state.

Joint arrays use SDK order FR, FL, RR, RL, each hip/thigh/calf. Foot-force arrays
use FL, FR, RL, RR. Quaternions are world-from-body, wxyz; gyro is body-frame.
The flat-ground estimator is deliberately separate from simulator truth audits.
"""
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace, ModuleType
import importlib
import hashlib
import sys
import xml.etree.ElementTree as ET

import numpy as np
import onnxruntime as ort
import torch
import yaml

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'source'


def load_control_modules():
    # The training package auto-registers Isaac tasks on import. Give the shared
    # pure modules their own namespace rather than importing Isaac or mocking it.
    for name, path in [('_go2_control', SOURCE),
                       ('_go2_control.cpg', SOURCE / 'cpg'),
                       ('_go2_control.skills', SOURCE / 'skills')]:
        if name not in sys.modules:
            module = ModuleType(name)
            module.__path__ = [str(path)]
            sys.modules[name] = module
    return (importlib.import_module('_go2_control.cpg.core'),
            importlib.import_module('_go2_control.cpg.controller'))


core, control = load_control_modules()
SDK_NAMES = [f'{leg}_{joint}_joint' for leg in ('FR', 'FL', 'RR', 'RL')
             for joint in ('hip', 'thigh', 'calf')]
FOOT_NAMES = ('FL', 'FR', 'RL', 'RR')


def load_selection(path):
    """Resolve relocatable bundle paths and verify the actor before loading it."""
    import json
    path=Path(path).resolve()
    selection=json.loads(path.read_text())
    for key in ('deploy_metadata','onnx_export','jit_export','settings'):
        if key in selection:
            item=Path(selection[key])
            selection[key]=str(item if item.is_absolute() else path.parent/item)
    if hashlib.sha256(Path(selection['onnx_export']).read_bytes()).hexdigest()!=selection['onnx_sha256']:
        raise ValueError('Policy checksum mismatch')
    return selection


@dataclass
class SensorFrame:
    time: float
    q: np.ndarray
    qd: np.ndarray
    quaternion: np.ndarray
    gyro: np.ndarray
    foot_force: np.ndarray

    def validate(self):
        for name, size in [('q', 12), ('qd', 12), ('quaternion', 4),
                           ('gyro', 3), ('foot_force', 4)]:
            value = np.asarray(getattr(self, name))
            if value.shape != (size,) or not np.isfinite(value).all():
                raise ValueError(f'Invalid sensor {name}')
        if not np.isfinite(self.time) or abs(np.linalg.norm(self.quaternion) - 1) > .02:
            raise ValueError('Invalid timestamp or IMU quaternion')
        q=np.asarray(self.q).reshape(4,3);qd=np.asarray(self.qd).reshape(4,3)
        if (q<np.array([-1.20,-1.73,-2.88])).any() or (q>np.array([1.20,3.65,-.68])).any() or (np.abs(qd)>np.array([35.,35.,18.])).any():
            raise ValueError('Encoder position/velocity outside physical plausibility bounds')
        if (self.foot_force < 0).any():
            raise ValueError('Foot force must be nonnegative')


def rotation(quaternion):
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class FlatGroundEstimator:
    """Support-foot kinematic height/velocity; no root pose or ground truth input."""
    def __init__(self):
        self.height = .322
        self.contact = np.zeros(4, dtype=bool)

    def update(self, frame):
        # Match the controller's force hysteresis. Keep the last support height
        # during brief no-contact samples; flight is outside this estimator's scope.
        self.contact = np.where(self.contact, frame.foot_force > 2, frame.foot_force >= 5)
        indices = [SDK_NAMES.index(f'{leg}_{j}_joint') for leg in FOOT_NAMES
                   for j in ('hip', 'thigh', 'calf')]
        q = torch.tensor(frame.q[indices], dtype=torch.float32).reshape(1, 4, 3)
        qd = torch.tensor(frame.qd[indices], dtype=torch.float32).reshape(1, 4, 3)
        feet = core.fk(q)[0].numpy()
        relative_velocity = core.foot_velocity(q, qd)[0].numpy() + np.cross(frame.gyro, feet)
        rot = rotation(frame.quaternion)
        world_relative = feet @ rot.T
        if self.contact.any():
            self.height = float(np.median(.022 - world_relative[self.contact, 2]))
            velocity = -relative_velocity[self.contact].mean(0)
        else:
            velocity = np.zeros(3)
        positions = world_relative.copy()
        positions[:, 2] += self.height
        speeds = (relative_velocity + velocity) @ rot.T
        return positions, speeds, rot.T @ np.array([0., 0., -1.]), feet


class RobotFacade:
    def __init__(self, names, defaults, limits, stiffness):
        self.names = names
        self.data = SimpleNamespace(
            default_joint_pos=torch.tensor(defaults, dtype=torch.float32)[None],
            joint_pos=torch.tensor(defaults, dtype=torch.float32)[None],
            joint_vel=torch.zeros(1, 12),
            soft_joint_pos_limits=torch.tensor(limits, dtype=torch.float32)[None],
            default_joint_stiffness=torch.tensor(stiffness, dtype=torch.float32)[None],
            body_pos_w=torch.zeros(1, 4, 3), body_lin_vel_w=torch.zeros(1, 4, 3),
            projected_gravity_b=torch.tensor([[0., 0., -1.]]), root_ang_vel_b=torch.zeros(1, 3))
        # Match training's calibrated controller mass, not simulator-only mass.
        self.root_physx_view = SimpleNamespace(get_masses=lambda: torch.tensor([[16.087]]))

    def find_bodies(self, names, preserve_order=True):
        return [FOOT_NAMES.index(n.removesuffix('_foot')) for n in names], names

    def find_joints(self, names, preserve_order=True):
        return [self.names.index(n) for n in names], names


class DanceRuntime:
    def __init__(self, metadata, policy, first_front=0):
        metadata = yaml.safe_load(Path(metadata).read_text())
        self.dt = float(metadata['step_dt'])
        if self.dt != .02:
            raise ValueError('This controller is qualified at 50 Hz only')
        self.map = np.array(metadata['joint_ids_map'])
        if sorted(self.map.tolist()) != list(range(12)):
            raise ValueError('Invalid SDK joint permutation')
        self.names = [SDK_NAMES[i] for i in self.map]
        self.defaults = np.array(metadata['default_joint_pos'], dtype=np.float32)
        # Recover exact geometric defaults rather than the exporter's 3dp rounding.
        angle = np.arccos(.30 / (.213 * 2))
        self.defaults = np.array([0 if '_hip_' in n else angle if '_thigh_' in n else -2*angle
                                  for n in self.names], dtype=np.float32)
        urdf = ET.parse(ROOT / 'assets/go2.urdf')
        joints = {j.get('name'): j for j in urdf.findall('joint')}
        bounds = np.array([[float(joints[n].find('limit').get(key)) for key in ('lower', 'upper')]
                           for n in self.names])
        center = bounds.mean(1, keepdims=True)
        self.limits = center + .9*(bounds-center)
        self.kp = np.array(metadata['stiffness'])
        self.kd = np.array(metadata['damping'])
        self.robot = RobotFacade(self.names, self.defaults, self.limits, self.kp)
        self.sensor = SimpleNamespace(data=SimpleNamespace(net_forces_w=torch.zeros(1, 4, 3)),
                                      find_bodies=self.robot.find_bodies)
        self.env = SimpleNamespace(num_envs=1, device='cpu', step_dt=self.dt, common_step_counter=0,
                                   scene={'robot': self.robot, 'contact_forces': self.sensor})
        # Controller also reads flat ground origins through attribute access.
        class Scene(dict):
            env_origins = torch.zeros(1, 3)
        self.env.scene = Scene(self.env.scene)
        gait = core.GaitConfig(**metadata['cpg_residual']['gait'])
        if gait.mode not in ('walk','dance'):
            raise ValueError('Select walk or dance metadata for this runtime')
        controller_type=control.CPGDanceController if gait.mode=='dance' else control.CPGController
        self.controller = controller_type(SimpleNamespace(gait=gait), self.env)
        self.controller.set_velocity([0., 0., 0.])
        self.controller.reset()
        if first_front not in (0, 1):
            raise ValueError('first_front must be FL=0 or FR=1')
        if first_front and gait.mode=='dance':
            d = self.controller.dance
            d.time += 4*gait.beat
            d.target_time += 4*gait.beat
            d.start_time += 4*gait.beat
            d.initial_front[:] = first_front
            self.controller.time = d.time.clone()
            self.controller.target_time = d.target_time.clone()
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.policy = ort.InferenceSession(str(policy), options, providers=['CPUExecutionProvider'])
        self.input_name = self.policy.get_inputs()[0].name
        self.observation_dim=int(metadata['cpg_residual'].get('observation_dim',75))
        if self.observation_dim not in (75,81) or self.policy.get_inputs()[0].shape[-1]!=self.observation_dim or self.policy.get_outputs()[0].shape[-1]!=12:
            raise ValueError('Expected declared 75/81-observation, 12-action CPG actor')
        self.estimator = FlatGroundEstimator()
        self.last_action = np.zeros((1, 12), dtype=np.float32)
        self.last_time = None
        self.fault = None
        self.stand_requested=False
        self.standing=False
        self.stand_wait=0.
        self.stand_stable=0.
        self.last_target=None

    def request_stand(self):
        """Deliberate stop only: zero travel, finish a landing, then hold four feet.

        This is not triggered by neutral/stale joystick input and does not rearm.
        Resume requires a fresh runtime with measured startup settling.
        """
        self.stand_requested=True

    def observe(self, frame, velocity, diagnostic_truth=None):
        if self.fault:
            raise RuntimeError(f'Controller fault latched: {self.fault}')
        try:
            frame.validate()
            velocity = np.asarray(velocity, dtype=np.float32)
            if velocity.shape != (3,) or not np.isfinite(velocity).all():
                raise ValueError('Invalid operator velocity')
            if self.last_time is not None and abs(frame.time-self.last_time-self.dt) > .002:
                raise ValueError('Stale, skipped, or reordered sensor frame')
            positions, speeds, gravity, feet = self.estimator.update(frame)
            if gravity[2] > -np.cos(.65):
                raise ValueError('Excessive body tilt')
            self.last_time = frame.time
            data = self.robot.data
            data.joint_pos[:] = torch.from_numpy(frame.q[self.map].astype(np.float32))[None]
            data.joint_vel[:] = torch.from_numpy(frame.qd[self.map].astype(np.float32))[None]
            data.root_ang_vel_b[:] = torch.tensor(frame.gyro, dtype=torch.float32)[None]
            data.projected_gravity_b[:] = torch.tensor(gravity, dtype=torch.float32)[None]
            # Truth injection is an explicitly labelled diagnostic mode only.
            if diagnostic_truth is not None:
                positions, speeds = diagnostic_truth
            data.body_pos_w[:] = torch.tensor(positions, dtype=torch.float32)[None]
            data.body_lin_vel_w[:] = torch.tensor(speeds, dtype=torch.float32)[None]
            self.sensor.data.net_forces_w.zero_()
            self.sensor.data.net_forces_w[0, :, 2] = torch.tensor(frame.foot_force, dtype=torch.float32)
            c = self.controller
            if self.stand_requested:velocity=np.zeros(3,dtype=np.float32)
            c.set_velocity(velocity)
            self.env.common_step_counter += 1
            if not self.standing:c._update_command()
            if self.stand_requested and not self.standing:
                self.stand_wait+=self.dt
                support_kind=not hasattr(c,'dance') or c.dance.kind.item()==c.dance.SUPPORT
                stable=(c.ready.item() and support_kind and c.events.confirmed.all().item()
                        and c.velocity.norm().item()<.005 and np.max(np.abs(frame.qd))<.5
                        and np.linalg.norm(frame.gyro)<.3 and self.last_target is not None)
                self.stand_stable=self.stand_stable+self.dt if stable else 0.
                if self.stand_stable>=.08:self.standing=True
                if self.stand_wait>8 and not self.standing:raise ValueError('Requested stand did not reach stable four-foot support')
            if not c.ready.item() and c.age.item() > 5:
                raise ValueError('Measured startup settling timed out')
            observation = np.concatenate((np.clip(frame.gyro*.2, -100, 100),
                                           np.clip(gravity, -100, 100),
                                           np.clip(frame.q[self.map]-self.defaults, -100, 100),
                                           np.clip(frame.qd[self.map]*.05, -100, 100),
                                           np.clip(self.last_action[0], -100, 100),
                                           c.command[0].numpy())).astype(np.float32)[None]
            if self.observation_dim==81:
                observation=np.concatenate((observation,c.measured_features.numpy()),axis=-1).astype(np.float32)
            if not np.isfinite(observation).all():
                raise ValueError('Nonfinite observation')
            return observation
        except (ValueError, RuntimeError) as error:
            self.fault = str(error)
            raise

    def act(self, observation):
        if self.fault:
            raise RuntimeError(f'Controller fault latched: {self.fault}')
        if self.standing:return self.last_target.copy()
        action = self.policy.run(None, {self.input_name: observation})[0]
        if not np.isfinite(action).all():
            self.fault = 'Nonfinite policy output'
            raise ValueError(self.fault)
        self.last_action = action.astype(np.float32)
        c = self.controller
        target = c.q_ref.numpy() + c.cfg.gait.residual_scale*np.tanh(action)*c.ready.numpy()[:, None]
        target = np.clip(target[0], self.limits[:, 0], self.limits[:, 1])
        sdk = np.empty(12)
        sdk[self.map] = target
        self.last_target=sdk.copy()
        return sdk
