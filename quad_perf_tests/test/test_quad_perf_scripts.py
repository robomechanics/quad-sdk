import importlib
import os
import signal
import sys
import types

import pytest
from quad_msgs.msg import GRFArray, LegCommandArray, RobotPlan, RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

import episode_monitor_node
import run_iterations
import time_sync_node
import wait_for_robot_node


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(msg)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    import math

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return types.SimpleNamespace(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


def make_robot_state(roll=0.0, pitch=0.0, z=0.3, vz=0.0):
    msg = RobotState()
    q = make_quaternion(roll=roll, pitch=pitch)
    msg.body.pose.orientation.w = q.w
    msg.body.pose.orientation.x = q.x
    msg.body.pose.orientation.y = q.y
    msg.body.pose.orientation.z = q.z
    msg.body.pose.position.z = z
    msg.body.twist.linear.z = vz
    return msg


def fake_wait_node(required_msgs=2):
    node = types.SimpleNamespace(
        joints_received=False,
        settled_count=0,
        published=False,
        required_msgs=required_msgs,
        max_tilt=0.5,
        max_vz=0.5,
        ready_pub=FakePublisher(),
        logger=FakeLogger(),
    )
    node.get_logger = lambda: node.logger
    return node


def fake_episode_node():
    node = types.SimpleNamespace(
        shutdown_pending=False,
        settle_time=0.1,
        timers=[],
        logger=FakeLogger(),
    )
    node.get_logger = lambda: node.logger
    node.create_timer = lambda period, cb: node.timers.append((period, cb))
    node.request_shutdown = (
        lambda reason: episode_monitor_node.EpisodeMonitorNode.request_shutdown(
            node, reason))
    node.trigger_shutdown = (
        lambda: episode_monitor_node.EpisodeMonitorNode.trigger_shutdown(node))
    return node


def test_wait_for_robot_quaternion_conversion_and_ready_gate():
    q = make_quaternion(roll=0.25, pitch=-0.15)
    roll, pitch = wait_for_robot_node.quaternion_to_rp(q)
    assert roll == pytest.approx(0.25)
    assert pitch == pytest.approx(-0.15)

    node = fake_wait_node(required_msgs=2)
    wait_for_robot_node.WaitForRobotNode.state_cb(node, make_robot_state())
    assert node.settled_count == 0

    joints = JointState()
    joints.position = [0.0]
    wait_for_robot_node.WaitForRobotNode.joints_cb(node, joints)
    assert node.joints_received is True

    wait_for_robot_node.WaitForRobotNode.state_cb(node, make_robot_state())
    assert node.settled_count == 1
    with pytest.raises(SystemExit):
        wait_for_robot_node.WaitForRobotNode.state_cb(node, make_robot_state())
    assert node.published is True
    assert node.ready_pub.messages[-1].data is True


def test_wait_for_robot_resets_count_when_tilted_or_falling():
    node = fake_wait_node(required_msgs=2)
    node.joints_received = True

    wait_for_robot_node.WaitForRobotNode.state_cb(node, make_robot_state())
    assert node.settled_count == 1

    wait_for_robot_node.WaitForRobotNode.state_cb(
        node, make_robot_state(roll=1.0))
    assert node.settled_count == 0

    wait_for_robot_node.WaitForRobotNode.state_cb(
        node, make_robot_state(vz=2.0))
    assert node.settled_count == 0


def test_episode_monitor_triggers_each_shutdown_condition_once():
    q = make_quaternion(roll=0.2, pitch=0.1)
    roll, pitch = episode_monitor_node.quaternion_to_rpy(q)
    assert roll == pytest.approx(0.2)
    assert pitch == pytest.approx(0.1)

    node = fake_episode_node()
    episode_monitor_node.EpisodeMonitorNode.goal_reached_cb(node, Bool(data=True))
    assert node.shutdown_pending is True
    assert len(node.timers) == 1

    episode_monitor_node.EpisodeMonitorNode.planner_failed_cb(
        node, Bool(data=True))
    assert len(node.timers) == 1

    node = fake_episode_node()
    node.max_tilt = 0.5
    episode_monitor_node.EpisodeMonitorNode.state_cb(
        node, make_robot_state(roll=0.8))
    assert node.shutdown_pending is True
    assert len(node.timers) == 1

    with pytest.raises(SystemExit):
        episode_monitor_node.EpisodeMonitorNode.trigger_shutdown(node)


def test_time_sync_callback_republishes_bundle_and_counts():
    node = types.SimpleNamespace(
        pub_state=FakePublisher(),
        pub_cmd=FakePublisher(),
        pub_grfs=FakePublisher(),
        pub_plan=FakePublisher(),
        pub_est=FakePublisher(),
        sync_count=0,
        logger=FakeLogger(),
    )
    node.get_logger = lambda: node.logger

    state = RobotState()
    cmd = LegCommandArray()
    grfs = GRFArray()
    plan = RobotPlan()
    est = RobotState()

    time_sync_node.TimeSyncNode.synced_callback(node, state, cmd, grfs, plan, est)

    assert node.pub_state.messages == [state]
    assert node.pub_cmd.messages == [cmd]
    assert node.pub_grfs.messages == [grfs]
    assert node.pub_plan.messages == [plan]
    assert node.pub_est.messages == [est]
    assert node.sync_count == 1


def test_run_iterations_cleanup_and_episode_paths_are_mocked(monkeypatch):
    run_calls = []

    def fake_run(cmd, **kwargs):
        run_calls.append((cmd, kwargs))
        if cmd[:3] == ['ros2', 'node', 'list']:
            return types.SimpleNamespace(returncode=0, stdout='')
        return types.SimpleNamespace(returncode=1, stdout='')

    removed = []
    monkeypatch.setattr(run_iterations.subprocess, 'run', fake_run)
    monkeypatch.setattr(run_iterations.time, 'sleep', lambda _: None)
    monkeypatch.setattr('glob.glob', lambda pattern: [f'{pattern}-stale'])
    monkeypatch.setattr('shutil.rmtree',
                        lambda path, ignore_errors=True: removed.append(path))

    assert run_iterations.wait_for_clean_state() is True
    assert any(call[0][:2] == ['ros2', 'daemon'] for call in run_calls)
    assert removed

    popen_cmds = []

    class FakeProc:
        pid = 1234
        returncode = 0

        def wait(self, timeout=None):
            return 0

        def poll(self):
            return 0

    monkeypatch.setattr(run_iterations.subprocess, 'Popen',
                        lambda cmd, preexec_fn=None: popen_cmds.append(cmd) or
                        FakeProc())
    monkeypatch.setattr(run_iterations.os, 'killpg',
                        lambda pgid, sig: (_ for _ in ()).throw(
                            AssertionError('killpg should not run on success')))

    args = types.SimpleNamespace(
        robot_type='go2',
        world='flat.sdf',
        plan_delay=5.0,
        output_dir='/tmp/bags',
        timeout=10.0,
    )
    assert run_iterations.run_episode('1/1', 1, args) is True
    assert popen_cmds[0][:4] == ['ros2', 'launch', 'quad_perf_tests',
                                 'run_iteration.py']
    assert 'robot_type:=go2' in popen_cmds[0]
    assert 'world:=flat.sdf' in popen_cmds[0]


def test_run_episode_timeout_sends_sigint(monkeypatch):
    class FakeTimeoutProc:
        pid = 1234
        returncode = None

        def wait(self, timeout=None):
            raise run_iterations.subprocess.TimeoutExpired('cmd', timeout)

    kill_calls = []
    monkeypatch.setattr(run_iterations.subprocess, 'Popen',
                        lambda cmd, preexec_fn=None: FakeTimeoutProc())
    monkeypatch.setattr(run_iterations.os, 'getpgid', lambda pid: pid)
    monkeypatch.setattr(run_iterations.os, 'killpg',
                        lambda pgid, sig: kill_calls.append((pgid, sig)))
    monkeypatch.setattr(run_iterations, 'nuke_all_ros_and_gz', lambda: None)
    monkeypatch.setattr(run_iterations, 'wait_for_clean_state', lambda: True)
    monkeypatch.setattr(run_iterations.subprocess, 'run',
                        lambda *args, **kwargs: types.SimpleNamespace(returncode=1))
    monkeypatch.setattr(run_iterations.time, 'sleep', lambda _: None)

    args = types.SimpleNamespace(
        robot_type='go2',
        world='flat.sdf',
        plan_delay=5.0,
        output_dir='/tmp/bags',
        timeout=1.0,
    )
    assert run_iterations.run_episode('1/1', 1, args) is False
    assert kill_calls == [(1234, signal.SIGINT)]


def test_launch_files_generate_descriptions():
    run_iteration = importlib.import_module('run_iteration')
    iteration_logging = importlib.import_module('iteration_logging')

    run_desc = run_iteration.generate_launch_description()
    logging_desc = iteration_logging.generate_launch_description()

    assert len(run_desc.entities) > 0
    assert len(logging_desc.entities) > 0
