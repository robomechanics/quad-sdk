"""
Ground reaction force plotting module for quadruped robotics.

This module provides visualization functionality for control trajectories,
specifically plotting ground reaction force (GRF) vectors for quadruped
robot analysis.
"""

import numpy as np
import matplotlib.pyplot as plt


def plot_control(control_traj, t_window, *args):
    """
    Plot ground reaction force vectors from control trajectory data.

    Creates a matplotlib figure showing the X, Y, and Z components of ground
    reaction forces for a selected leg over time, filtered by the specified
    time window.

    Args:
        control_traj (dict): Control trajectory data containing:
            - "time" (np.ndarray): Time array
            - "vectors" (list): List of GRF vector arrays for each leg
            - "points" (list): List of contact point arrays for each leg
            - "contactStates" (list): List of contact state arrays for each leg
        t_window (list or np.ndarray): Time window for plotting [start, end].
            If empty, plots entire trajectory.
        *args: Variable length argument list:
            - args[0] (str, optional): Line style for plots (default: '-')
            - args[1] (bool, optional): Whether to show titles (default: True)
            - args[2] (list, optional): Existing figure handles to reuse

    Returns:
        int: Figure number created.

    """
    # Use existing figure handles if requested
    if len(args) >= 3 and len(args[2]) > 0:
        grf_vectors_fig = args[2][0]
    else:
        h = plt.get_fignums()
        n = len(h)
        grf_vectors_fig = n + 1

    titles = args[1] if len(args) >= 2 else True
    line_style = args[0] if len(args) >= 1 else '-'

    # Calculate time indices and filter data
    if len(t_window) == 0:
        traj_idx = np.arange(len(control_traj["time"]))
    else:
        traj_idx = np.where(
            (control_traj["time"] >= t_window[0]) &
            (control_traj["time"] <= t_window[1])
        )[0]

    # Filter control trajectory data by time window
    control_traj["time"] = control_traj["time"][traj_idx]
    for i in range(len(control_traj["vectors"])):
        control_traj["vectors"][i] = control_traj["vectors"][i][traj_idx, :]
        control_traj["points"][i] = control_traj["points"][i][traj_idx, :]
        control_traj["contactStates"][i] = (
            control_traj["contactStates"][i][traj_idx, :]
        )

    control_color_vector = [
        np.array([166, 25, 46]) / 255,
        np.array([0, 45, 114]) / 255,
        np.array([0, 132, 61]) / 255,
        np.array([242, 169, 0]) / 255
    ]

    # Plot GRF Vectors (One leg, all XYZ components)
    leg_idx = 3  # Fourth leg (0-indexed)

    fig = plt.figure(grf_vectors_fig)
    fig.canvas.manager.set_window_title("grfs")
    fig.set_size_inches(12, 4)  # Match MATLAB's [1200 400] size

    plt.hold = True  # Equivalent to MATLAB's hold on (deprecated but conceptual)

    plt.plot(
        control_traj["time"],
        control_traj["vectors"][leg_idx][:, 0],
        color=control_color_vector[0],
        linewidth=2,
        linestyle=line_style
    )
    plt.plot(
        control_traj["time"],
        control_traj["vectors"][leg_idx][:, 1],
        color=control_color_vector[1],
        linewidth=2,
        linestyle=line_style
    )
    plt.plot(
        control_traj["time"],
        control_traj["vectors"][leg_idx][:, 2],
        color=control_color_vector[2],
        linewidth=2,
        linestyle=line_style
    )

    if titles:
        plt.title('GRF Vectors')
    plt.xlabel('Time (s)')
    plt.ylabel('GRF (N)')
    plt.axis([
        np.min(control_traj["time"]),
        np.max(control_traj["time"]),
        -200,
        200
    ])
    plt.legend(['X', 'Y', 'Z'], loc='right')

    fig_array = np.array([grf_vectors_fig])

    # Export
    return fig_array
