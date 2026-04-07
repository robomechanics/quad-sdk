#!/usr/bin/env python3
"""
Multi-episode training data collection runner.

Launches the full headless simulation pipeline N times.  Each episode
ends automatically when the robot reaches the goal (via the episode
monitor node), or after a safety timeout.

Usage:
  # 10 episodes, default robot
  python3 run_collection.py --num_episodes 10

  # With a safety timeout of 5 minutes per episode
  python3 run_collection.py --num_episodes 10 --timeout 300

  # Custom robot, world, and output
  python3 run_collection.py --num_episodes 5 --robot_type spirit \
      --world rough.sdf --output_dir /data/bags
"""

import argparse
import subprocess
import signal
import sys
import time
import os

# Track the active subprocess so we can clean up on Ctrl+C
_active_proc = None


def nuke_all_ros_and_gz():
    """Kill every ROS/Gazebo process and clean up shared state.
    Called between episodes and on exit."""
    # Gazebo and its spawned plugins
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'ruby.*gz'], capture_output=True)
    # All ros2 nodes that match our robot namespace or common node names
    subprocess.run(['pkill', '-9', '-f', 'parameter_bridge'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'robot_driver_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'local_planner_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'global_body_planner_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'body_force_estimator_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'contact_state_publisher'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'robot_state_publisher'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'rviz_interface_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'rviz2'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'mesh_to_grid_map'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'grid_map'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'time_sync_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'episode_monitor_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'wait_for_robot_node'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'rosbag2'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'static_transform_publisher'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'relay'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'spawner'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'controller_manager'], capture_output=True)

    # Clean up Gazebo transport shared memory and lock files
    # Without this, the next gz sim instance can hang or fail to
    # activate controllers because stale transport state interferes
    import glob as globmod
    import shutil
    for path in globmod.glob('/tmp/gz-transport-*'):
        shutil.rmtree(path, ignore_errors=True)
    for path in globmod.glob('/tmp/gz-msgs-*'):
        shutil.rmtree(path, ignore_errors=True)


def reset_ros2_daemon():
    """Restart the ROS2 daemon to flush stale DDS discovery entries.
    Gazebo plugins (controller_manager, gz_ros_control, etc.) run inside
    the gz sim process.  When gz sim is SIGKILL'd these nodes are dead,
    but the DDS daemon still advertises them.  Restarting the daemon
    clears the cache."""
    subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True)
    time.sleep(1)
    subprocess.run(['ros2', 'daemon', 'start'], capture_output=True)
    time.sleep(2)


def wait_for_clean_state(max_retries=3):
    """Kill everything, reset the daemon, verify clean."""
    nuke_all_ros_and_gz()
    time.sleep(3)
    reset_ros2_daemon()

    for attempt in range(1, max_retries + 1):
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True, text=True, timeout=5)
            remaining = result.stdout.strip()
        except subprocess.TimeoutExpired:
            remaining = ''

        if not remaining:
            print(f'[Cleanup] All nodes terminated.')
            return True

        print(f'[Cleanup] Attempt {attempt}/{max_retries} — '
              f'still running:\n{remaining}')
        nuke_all_ros_and_gz()
        reset_ros2_daemon()

    print('[Cleanup] Warning: could not clear all nodes. Proceeding anyway.')
    return False


def cleanup_and_exit(signum=None, frame=None):
    """Signal handler for Ctrl+C — kill everything and exit."""
    print('\n[Runner] Caught interrupt, cleaning up...')
    global _active_proc
    if _active_proc and _active_proc.poll() is None:
        try:
            os.killpg(os.getpgid(_active_proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    nuke_all_ros_and_gz()
    wait_for_clean_state(max_retries=3)
    print('[Runner] Cleanup complete. Exiting.')
    sys.exit(0)


def run_episode(label, total, args):
    """Launch one data collection episode. Returns True on success."""
    global _active_proc

    cmd = [
        'ros2', 'launch', 'quad_training', 'data_collection.py',
        f'robot_type:={args.robot_type}',
        f'world:={args.world}',
        f'plan_delay:={args.plan_delay}',
        f'output_dir:={args.output_dir}',
    ]

    print(f'\n{"="*60}')
    print(f'  Episode {label}')
    print(f'  Ends on: goal reached OR {args.timeout:.0f}s safety timeout')
    print(f'{"="*60}\n')

    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    _active_proc = proc

    success = False
    try:
        proc.wait(timeout=args.timeout)
        # Exit code 0 means episode_monitor triggered clean shutdown (goal reached)
        success = (proc.returncode == 0)
        if success:
            print(f'[Episode {label}] Finished (goal reached)')
        else:
            print(f'[Episode {label}] Exited with code {proc.returncode} (failed)')
    except subprocess.TimeoutExpired:
        print(f'[Episode {label}] Safety timeout reached, shutting down...')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=30)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            pass

    _active_proc = None

    # Wait for Gazebo to shut down gracefully (avoid SIGKILL corruption)
    print(f'[Episode {label}] Waiting for Gazebo to exit gracefully...')
    for _ in range(30):
        result = subprocess.run(['pgrep', '-f', 'gz sim'],
                                capture_output=True)
        if result.returncode != 0:
            break
        # Send SIGTERM (not SIGKILL) to give gz sim a chance
        subprocess.run(['pkill', '-15', '-f', 'gz sim'], capture_output=True)
        time.sleep(1)
    else:
        # Only SIGKILL as absolute last resort
        print(f'[Episode {label}] Gazebo did not exit, forcing kill...')
        subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
        time.sleep(2)

    # Clean up everything else and reset DDS
    print(f'[Episode {label}] Cleaning up...')
    nuke_all_ros_and_gz()
    wait_for_clean_state()
    return success


def main():
    # Register Ctrl+C handler
    signal.signal(signal.SIGINT, cleanup_and_exit)
    signal.signal(signal.SIGTERM, cleanup_and_exit)

    parser = argparse.ArgumentParser(
        description='Run multiple training data collection episodes')
    parser.add_argument('--num_episodes', type=int, required=True,
                        help='Number of episodes to collect')
    parser.add_argument('--timeout', type=float, default=300.0,
                        help='Max seconds per episode as safety net (default: 300)')
    parser.add_argument('--robot_type', default='go2',
                        help='Robot type (default: go2)')
    parser.add_argument('--world', default='flat.sdf',
                        help='Gazebo world file (default: flat.sdf)')
    parser.add_argument('--plan_delay', type=float, default=5.0,
                        help='Seconds after stand before planning (default: 5)')
    parser.add_argument('--output_dir', default=os.path.join(os.getcwd(), 'training_bags'),
                        help='Output directory for bags (default: ./training_bags)')
    parser.add_argument('--max_retries', type=int, default=3,
                        help='Max retries per episode on failure (default: 3)')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    print(f'Starting {args.num_episodes} episodes')
    print(f'Robot: {args.robot_type} | World: {args.world}')
    print(f'Safety timeout: {args.timeout}s per episode')
    print(f'Max retries per episode: {args.max_retries}')
    print(f'Output: {args.output_dir}')

    successful = 0
    episode = 0
    while successful < args.num_episodes:
        episode += 1
        attempt = 0
        while attempt < args.max_retries:
            attempt += 1
            label = f'{successful + 1}/{args.num_episodes}'
            if attempt > 1:
                label += f' (retry {attempt}/{args.max_retries})'
            success = run_episode(label, args.num_episodes, args)
            if success:
                successful += 1
                break
            print(f'[Runner] Episode failed, retrying ({attempt}/{args.max_retries})...')
        else:
            print(f'[Runner] Episode failed after {args.max_retries} attempts, skipping.')
            successful += 1  # count it anyway to avoid infinite loop

    print(f'\n{"="*60}')
    print(f'  Done! Completed {args.num_episodes} episodes in {episode} attempts.')
    print(f'  Bags saved to: {args.output_dir}')
    print(f'{"="*60}')


if __name__ == '__main__':
    main()
