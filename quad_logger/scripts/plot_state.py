"""
State trajectory visualization module for quadruped robotics.

This module provides comprehensive visualization functionality for quadruped
robot state trajectories, including body motion (linear and angular states),
joint states (positions, velocities, efforts), and foot trajectories in 3D
space and over time.
"""

from math import pi
import numpy as np
import matplotlib.pyplot as plt


def plot_state(state_traj, t_window, *args):
    """
    Plot comprehensive state trajectory data for a quadruped robot.

    Creates multiple matplotlib figures showing body trajectories, linear and
    angular states, joint positions/velocities/efforts, and foot positions/
    velocities. All data is filtered by the specified time window.

    Args:
        state_traj (dict): State trajectory data containing:
            - "time" (np.ndarray): Time array
            - "position" (np.ndarray): Body position [x, y, z]
            - "velocity" (np.ndarray): Body linear velocity
            - "orientationRPY" (np.ndarray): Body orientation (roll, pitch, yaw)
            - "orientationQuat" (np.ndarray): Body orientation quaternion
            - "angularVelocity" (np.ndarray): Body angular velocity
            - "jointPosition" (np.ndarray): Joint angles for all legs
            - "jointVelocity" (np.ndarray): Joint velocities
            - "jointEffort" (np.ndarray): Joint efforts/currents
            - "footPosition" (list): Foot positions for each leg
            - "footVelocity" (list): Foot velocities for each leg
        t_window (list or np.ndarray): Time window for plotting [start, end].
            If empty, plots entire trajectory.
        *args: Variable length argument list:
            - args[0] (str, optional): Line style for plots (default: '-')
            - args[1] (bool, optional): Whether to show titles (default: True)
            - args[2] (list, optional): Existing figure handles to reuse,
              ordered as [com_traj, linear_state, angular_state,
              joint_position, joint_velocity, joint_effort, foot_position,
              foot_velocity]

    Returns:
        np.ndarray: Array of figure numbers for all created plots:
            [com_traj_fig, linear_state_fig, angular_state_fig,
             joint_position_fig, joint_velocity_fig, joint_effort_fig,
             foot_position_fig, foot_velocity_fig]

    Note:
        Not all state_traj keys are required. The function will only plot
        data for keys that exist in the dictionary. Reference trajectories
        may not include joint or foot data.
    """
    # Use existing figure handles if requested
    if len(args) >= 3 and len(args[2]) > 0:
        com_traj_fig = args[2][0]
        linear_state_fig = args[2][1]
        angular_state_fig = args[2][2]
        joint_position_fig = args[2][3]
        joint_velocity_fig = args[2][4]
        joint_effort_fig = args[2][5]
        foot_position_fig = args[2][6]
        foot_velocity_fig = args[2][7]
    else:
        h = plt.get_fignums()
        n = len(h)
        com_traj_fig = n + 1
        linear_state_fig = n + 2
        angular_state_fig = n + 3
        joint_position_fig = n + 4
        joint_velocity_fig = n + 5
        joint_effort_fig = n + 6
        foot_position_fig = n + 7
        foot_velocity_fig = n + 8

    titles = args[1] if len(args) >= 2 else True
    line_style = args[0] if len(args) >= 1 else '-'

    # Calculate time indices
    if len(t_window) == 0:
        traj_idx = np.arange(len(state_traj["time"]))
    else:
        traj_idx = np.where(
            (state_traj["time"] >= t_window[0]) &
            (state_traj["time"] <= t_window[1])
        )[0]

    # Only use keys that exist in the dictionary, some keys are not
    # included in reference trajectory
    keys_to_trim = ["time", "position", "velocity", "orientationRPY",
                    "orientationQuat", "angularVelocity", "jointPosition",
                    "jointVelocity", "jointEffort"]

    for key in keys_to_trim:
        if key in state_traj and len(state_traj[key]) > 0:
            state_traj[key] = state_traj[key][traj_idx]

    # Foot data
    if "footPosition" in state_traj:
        for i in range(len(state_traj["footPosition"])):
            state_traj["footPosition"][i] = (
                state_traj["footPosition"][i][traj_idx])
            state_traj["footVelocity"][i] = (
                state_traj["footVelocity"][i][traj_idx])

    # Color definitions
    foot_color_vector = [np.array([166, 25, 46]) / 255,
                         np.array([0, 45, 114]) / 255,
                         np.array([0, 132, 61]) / 255,
                         np.array([242, 169, 0]) / 255]
    body_color_vector = foot_color_vector

    num_feet = 4
    joint_color_vector = foot_color_vector

    # Plot 1: 3D Trajectory Plot
    fig1 = plt.figure(com_traj_fig)
    ax = plt.subplot(projection='3d')
    fig1.canvas.manager.set_window_title("body_and_foot_trajectories")

    if "position" in state_traj:
        ax.plot(
            state_traj["position"][:, 0],
            state_traj["position"][:, 1],
            state_traj["position"][:, 2],
            linestyle=line_style, label='Body'
        )

    if "footPosition" in state_traj:
        for i in range(len(state_traj["footPosition"])):
            ax.plot(
                state_traj["footPosition"][i][:, 0],
                state_traj["footPosition"][i][:, 1],
                state_traj["footPosition"][i][:, 2],
                color=foot_color_vector[i % 4],
                linestyle=line_style
            )

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    if titles:
        ax.set_title('Body and Foot Trajectories')

    # Plot 2: Linear States
    fig2 = plt.figure(linear_state_fig)
    if linear_state_fig not in plt.get_fignums():
        fig2.set_size_inches(12, 6)
    fig2.clf()
    fig2.canvas.manager.set_window_title("linear_states")

    ax1 = plt.subplot(2, 1, 1)
    if "position" in state_traj and state_traj["position"] is not None:
        for i in range(3):
            ax1.plot(state_traj['time'], state_traj['position'][:, i],
                     color=body_color_vector[i], linestyle=line_style)

    ax1.set_ylabel('Position (m)')
    if titles:
        ax1.set_title('Linear States')
    ax1.legend(['X', 'Y', 'Z'], loc='right')
    ax1.autoscale(enable=True, axis='both', tight=True)
    ax1.grid(True)

    # Second subplot - Velocity
    ax2 = plt.subplot(2, 1, 2)
    if "velocity" in state_traj and state_traj["velocity"] is not None:
        for i in range(3):
            ax2.plot(state_traj['time'], state_traj['velocity'][:, i],
                     color=body_color_vector[i], linestyle=line_style)

    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_xlabel('Time (s)')
    ax2.autoscale(enable=True, axis='both', tight=True)
    ax2.grid(True)

    # Align y-labels
    fig2.align_ylabels([ax1, ax2])
    plt.tight_layout()

    # Plot 3: Angular States
    if ("orientationRPY" in state_traj and
            state_traj["orientationRPY"] is not None):
        fig3 = plt.figure(angular_state_fig)
        if angular_state_fig not in plt.get_fignums():
            fig3.set_size_inches(12, 6)
        fig3.clf()
        fig3.canvas.manager.set_window_title("angular_states")

        ax1 = plt.subplot(2, 1, 1)
        for i in range(3):
            ax1.plot(state_traj['time'], state_traj['orientationRPY'][:, i],
                     color=body_color_vector[i], linestyle=line_style)

        ax1.set_ylabel('Ang. Pos. (rad)')
        if titles:
            ax1.set_title('Angular States')
        ax1.legend(['Roll', 'Pitch', 'Yaw'], loc='right')
        ax1.set_ylim([-pi, pi])
        ax1.autoscale(enable=True, axis='x', tight=True)
        ax1.grid(True)

        # Second subplot - Angular Velocity
        ax2 = plt.subplot(2, 1, 2)
        if ("angularVelocity" in state_traj and
                state_traj["angularVelocity"] is not None):
            for i in range(3):
                ax2.plot(state_traj['time'],
                         state_traj['angularVelocity'][:, i],
                         color=body_color_vector[i], linestyle=line_style)

        ax2.set_ylabel('Ang. Vel. (rad/s)')
        ax2.set_xlabel('Time (s)')
        ax2.autoscale(enable=True, axis='both', tight=True)
        ax2.grid(True)

        fig3.align_ylabels([ax1, ax2])
        plt.tight_layout()

    # Plot 4: Joint Positions
    if "jointPosition" in state_traj and state_traj["jointPosition"] is not None:
        # Specify indices for leg joints
        ab_index = [0, 3, 6, 9]
        hip_index = [1, 4, 7, 10]
        knee_index = [2, 5, 8, 11]
        abad_lim = [-1, 1]
        hip_lim = [-pi/2, pi]
        knee_lim = [0, pi]

        fig4 = plt.figure(joint_position_fig)
        if joint_position_fig not in plt.get_fignums():
            fig4.set_size_inches(12, 9)
        fig4.clf()
        fig4.canvas.manager.set_window_title("joint_positions")

        ax1 = plt.subplot(3, 1, 1)
        for i in range(num_feet):
            ax1.plot(state_traj['time'],
                     state_traj['jointPosition'][:, ab_index[i]],
                     color=joint_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax1.set_ylabel('Ab/Ad (rad)')
        if titles:
            ax1.set_title('Joint Angles')
        ax1.set_ylim(abad_lim)
        ax1.autoscale(enable=True, axis='x', tight=True)
        ax1.grid(True)

        ax2 = plt.subplot(3, 1, 2)
        for i in range(num_feet):
            ax2.plot(state_traj['time'],
                     state_traj['jointPosition'][:, hip_index[i]],
                     color=joint_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax2.set_ylabel('Hip (rad)')
        ax2.legend(['FL', 'BL', 'FR', 'BR'], loc='right')
        ax2.set_ylim(hip_lim)
        ax2.autoscale(enable=True, axis='x', tight=True)
        ax2.grid(True)

        ax3 = plt.subplot(3, 1, 3)
        for i in range(num_feet):
            ax3.plot(state_traj['time'],
                     state_traj['jointPosition'][:, knee_index[i]],
                     color=joint_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax3.set_ylabel('Knee (rad)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim(knee_lim)
        ax3.autoscale(enable=True, axis='x', tight=True)
        ax3.grid(True)

        fig4.align_ylabels([ax1, ax2, ax3])
        plt.tight_layout()

    # Plot 5: Joint Velocities
    if "jointVelocity" in state_traj and state_traj["jointVelocity"] is not None:
        abad_vel_lim = [-37.7, 37.7]
        hip_vel_lim = [-37.7, 37.7]
        knee_vel_lim = [-25, 25]

        fig5 = plt.figure(joint_velocity_fig)
        if joint_velocity_fig not in plt.get_fignums():
            fig5.set_size_inches(12, 9)
        fig5.clf()
        fig5.canvas.manager.set_window_title("joint_velocities")

        ax1 = plt.subplot(3, 1, 1)
        for i in range(num_feet):
            ax1.plot(state_traj['time'],
                     state_traj['jointVelocity'][:, ab_index[i]],
                     color=joint_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax1.set_ylabel('Ab/Ad (rad/s)')
        if titles:
            ax1.set_title('Joint Velocities')
        ax1.set_ylim(abad_vel_lim)
        ax1.autoscale(enable=True, axis='x', tight=True)
        ax1.grid(True)

        ax2 = plt.subplot(3, 1, 2)
        for i in range(num_feet):
            ax2.plot(state_traj['time'],
                     state_traj['jointVelocity'][:, hip_index[i]],
                     color=joint_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax2.set_ylabel('Hip (rad/s)')
        ax2.legend(['FL', 'BL', 'FR', 'BR'], loc='right')
        ax2.set_ylim(hip_vel_lim)
        ax2.autoscale(enable=True, axis='x', tight=True)
        ax2.grid(True)

        ax3 = plt.subplot(3, 1, 3)
        for i in range(num_feet):
            ax3.plot(state_traj['time'],
                     state_traj['jointVelocity'][:, knee_index[i]],
                     color=joint_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax3.set_ylabel('Knee (rad/s)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim(knee_vel_lim)
        ax3.autoscale(enable=True, axis='x', tight=True)
        ax3.grid(True)

        fig5.align_ylabels([ax1, ax2, ax3])
        plt.tight_layout()

    # Plot 6: Joint Effort
    if "jointEffort" in state_traj and state_traj["jointEffort"] is not None:
        abad_eff_lim = [-21, 21]
        hip_eff_lim = [-21, 21]
        knee_eff_lim = [-32, 32]

        fig6 = plt.figure(joint_effort_fig)
        if joint_effort_fig not in plt.get_fignums():
            fig6.set_size_inches(12, 9)
        fig6.clf()
        fig6.canvas.manager.set_window_title("joint_effort")

        ax1 = plt.subplot(3, 1, 1)
        for i in range(num_feet):
            ax1.plot(state_traj['time'],
                     state_traj['jointEffort'][:, ab_index[i]],
                     color=joint_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax1.set_ylabel('Ab/Ad (A)')
        if titles:
            ax1.set_title('Joint Effort')
        ax1.set_ylim(abad_eff_lim)
        ax1.autoscale(enable=True, axis='x', tight=True)
        ax1.grid(True)

        ax2 = plt.subplot(3, 1, 2)
        for i in range(num_feet):
            ax2.plot(state_traj['time'],
                     state_traj['jointEffort'][:, hip_index[i]],
                     color=joint_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax2.set_ylabel('Hip (A)')
        ax2.legend(['FL', 'BL', 'FR', 'BR'], loc='right')
        ax2.set_ylim(hip_eff_lim)
        ax2.autoscale(enable=True, axis='x', tight=True)
        ax2.grid(True)

        ax3 = plt.subplot(3, 1, 3)
        for i in range(num_feet):
            ax3.plot(state_traj['time'],
                     state_traj['jointEffort'][:, knee_index[i]],
                     color=joint_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax3.set_ylabel('Knee (A)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim(knee_eff_lim)
        ax3.autoscale(enable=True, axis='x', tight=True)
        ax3.grid(True)

        fig6.align_ylabels([ax1, ax2, ax3])
        plt.tight_layout()

    # Plot 7: Foot Positions
    if "footPosition" in state_traj and state_traj["footPosition"]:
        fig7 = plt.figure(foot_position_fig)
        if foot_position_fig not in plt.get_fignums():
            fig7.set_size_inches(12, 9)
        fig7.clf()
        fig7.canvas.manager.set_window_title("foot_positions")

        ax1 = plt.subplot(3, 1, 1)
        for i in range(num_feet):
            ax1.plot(state_traj['time'],
                     state_traj['footPosition'][i][:, 0],
                     color=foot_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax1.set_ylabel('X (m)')
        if titles:
            ax1.set_title('Foot Positions')
        ax1.autoscale(enable=True, axis='both', tight=True)
        ax1.grid(True)

        ax2 = plt.subplot(3, 1, 2)
        for i in range(num_feet):
            ax2.plot(state_traj['time'],
                     state_traj['footPosition'][i][:, 1],
                     color=foot_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax2.set_ylabel('Y (m)')
        ax2.legend(['FL', 'BL', 'FR', 'BR'], loc='right')
        ax2.autoscale(enable=True, axis='both', tight=True)
        ax2.grid(True)

        ax3 = plt.subplot(3, 1, 3)
        for i in range(num_feet):
            ax3.plot(state_traj['time'],
                     state_traj['footPosition'][i][:, 2],
                     color=foot_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax3.set_ylabel('Z (m)')
        ax3.set_xlabel('Time (s)')
        ax3.autoscale(enable=True, axis='both', tight=True)
        ax3.grid(True)

        fig7.align_ylabels([ax1, ax2, ax3])
        plt.tight_layout()

    # Plot 8: Foot Velocities
    if "footVelocity" in state_traj and state_traj["footVelocity"]:
        fig8 = plt.figure(foot_velocity_fig)
        if foot_velocity_fig not in plt.get_fignums():
            fig8.set_size_inches(12, 9)
        fig8.clf()
        fig8.canvas.manager.set_window_title("foot_velocities")

        # Calculate velocity axis limits
        vel_axis = 0
        for i in range(num_feet):
            vel_axis = max(vel_axis,
                           np.max(np.abs(state_traj['footVelocity'][i])))

        ax1 = plt.subplot(3, 1, 1)
        for i in range(num_feet):
            ax1.plot(state_traj['time'],
                     state_traj['footVelocity'][i][:, 0],
                     color=foot_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax1.set_ylabel('X (m/s)')
        if titles:
            ax1.set_title('Foot Velocities')
        ax1.set_ylim([-vel_axis, vel_axis])
        ax1.autoscale(enable=True, axis='x', tight=True)
        ax1.grid(True)

        ax2 = plt.subplot(3, 1, 2)
        for i in range(num_feet):
            ax2.plot(state_traj['time'],
                     state_traj['footVelocity'][i][:, 1],
                     color=foot_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax2.set_ylabel('Y (m/s)')
        ax2.legend(['FL', 'BL', 'FR', 'BR'], loc='right')
        ax2.set_ylim([-vel_axis, vel_axis])
        ax2.autoscale(enable=True, axis='x', tight=True)
        ax2.grid(True)

        ax3 = plt.subplot(3, 1, 3)
        for i in range(num_feet):
            ax3.plot(state_traj['time'],
                     state_traj['footVelocity'][i][:, 2],
                     color=foot_color_vector[i], linewidth=2,
                     linestyle=line_style)

        ax3.set_ylabel('Z (m/s)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim([-vel_axis, vel_axis])
        ax3.autoscale(enable=True, axis='x', tight=True)
        ax3.grid(True)

        fig8.align_ylabels([ax1, ax2, ax3])
        plt.tight_layout()

    # Return figure handles as numpy array
    fig_array = np.array([com_traj_fig, linear_state_fig, angular_state_fig,
                          joint_position_fig, joint_velocity_fig,
                          joint_effort_fig, foot_position_fig,
                          foot_velocity_fig])

    return fig_array
