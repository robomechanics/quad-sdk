"""
Local planning visualization module for quadruped robotics.

This module provides functionality to visualize local planning data including
solve times, iteration counts, costs, and prediction horizon evolution for
quadruped robot motion planning analysis.
"""

import numpy as np
import matplotlib.pyplot as plt


def plot_local_plan(local_plan, t_window, *args):
    """
    Plot local planning data including solve statistics and horizon evolution.

    Creates matplotlib figures showing solver performance metrics (solve time,
    iterations, cost) and the evolution of the prediction horizon over time,
    with optional complexity schedule visualization.

    Args:
        local_plan (dict): Local planning data containing:
            - "time" (np.ndarray): Time array
            - "solveTime" (np.ndarray): Solver computation times
            - "elementTimes" (list): Prediction horizon time arrays
            - "cost" (np.ndarray): Cost function values
            - "iterations" (np.ndarray): Solver iteration counts
            - "horizonLength" (np.ndarray): Length of prediction horizon
            - "complexitySchedule" (list): Complexity mode for each element
        t_window (list or np.ndarray): Time window for plotting [start, end].
            If empty, plots entire trajectory.
        *args: Variable length argument list:
            - args[0] (str, optional): Line style for plots (default: '-')
            - args[1] (bool, optional): Whether to show titles (default: True)
            - args[2] (list, optional): Existing figure handles [solve, horizon]

    Returns:
        list: List containing two figure numbers [solve_data_fig, prediction_horizon_fig].
    """
    # Use existing figure handles if requested
    if len(args) >= 3 and len(args[2]) > 0:
        solve_data_fig = args[2][0]
        prediction_horizon_fig = args[2][1]
    else:
        h = plt.get_fignums()
        n = len(h)
        solve_data_fig = n + 1
        prediction_horizon_fig = n + 2

    titles = args[1] if len(args) >= 2 else True
    line_style = args[0] if len(args) >= 1 else '-'

    # Calculate time indices
    if len(t_window) == 0:
        traj_idx = np.arange(len(local_plan["time"]))
    else:
        traj_idx = np.where(
            (local_plan["time"] >= t_window[0]) &
            (local_plan["time"] <= t_window[1])
        )[0]

    # Trim the data based on the requested window
    local_plan["time"] = local_plan["time"][traj_idx]
    local_plan["solveTime"] = local_plan["solveTime"][traj_idx]
    local_plan["elementTimes"] = np.asarray(        local_plan["elementTimes"])[traj_idx]
    local_plan["cost"] = local_plan["cost"][traj_idx]
    local_plan["iterations"] = local_plan["iterations"][traj_idx]
    local_plan["horizonLength"] = local_plan["horizonLength"][traj_idx]
    local_plan["complexitySchedule"] = np.asarray(
        local_plan["complexitySchedule"])[traj_idx]

    local_plan_color_vector = [
        np.array([166, 25, 46]) / 255,
        np.array([0, 45, 114]) / 255,
        np.array([0, 132, 61]) / 255,
        np.array([242, 169, 0]) / 255
    ]

    # Plot Solve Data
    fig_solve = plt.figure(solve_data_fig)
    fig_solve.canvas.manager.set_window_title("Local Plan Solve Data")

    plt.subplot(3, 1, 1)
    scale = 1000  # s to ms
    plt.plot(
        local_plan["time"],
        scale * local_plan["solveTime"],
        color=local_plan_color_vector[0],
        linewidth=2,
        linestyle=line_style
    )
    if titles:
        plt.title('Solve Data')
    plt.ylabel('Solve Time (ms)')
    plt.axis([
        local_plan["time"].min(),
        local_plan["time"].max(),
        0,
        100
    ])

    plt.subplot(3, 1, 2)
    plt.plot(
        local_plan["time"],
        local_plan["iterations"],
        color=local_plan_color_vector[0],
        linewidth=2,
        linestyle=line_style
    )
    plt.ylabel('Iterations')
    plt.axis([
        local_plan["time"].min(),
        local_plan["time"].max(),
        0,
        20
    ])

    plt.subplot(3, 1, 3)
    plt.plot(
        local_plan["time"],
        local_plan["cost"],
        color=local_plan_color_vector[0],
        linewidth=2,
        linestyle=line_style
    )
    plt.xlabel('Current Time (s)')
    plt.ylabel('Cost')
    plt.axis([
        local_plan["time"].min(),
        local_plan["time"].max(),
        0,
        np.max(local_plan["cost"])
    ])

    # Plot Horizon Evolution
    fig_horizon = plt.figure(prediction_horizon_fig)
    fig_horizon.canvas.manager.set_window_title("Prediction Horizon")

    marker_size_simple = 3
    marker_size_complex = 6

    element_times_vec = []
    complexity_schedule_vec = []
    traj_times_vec = []

    for i in range(len(local_plan["elementTimes"])):
        horizon_length = int(local_plan["horizonLength"][i])
        traj_times_vec.extend([local_plan["time"][i]] * horizon_length)
        element_times_vec.extend(local_plan["elementTimes"][i])
        complexity_schedule_vec.extend(local_plan["complexitySchedule"][i])

    traj_times_vec = np.array(traj_times_vec)
    element_times_vec = np.array(element_times_vec)
    complexity_schedule_vec = np.array(complexity_schedule_vec)

    simple_idx = np.where(complexity_schedule_vec == 0)[0]
    complex_idx = np.where(complexity_schedule_vec == 1)[0]

    plt.scatter(
        traj_times_vec[simple_idx],
        element_times_vec[simple_idx],
        s=marker_size_simple,
        color=local_plan_color_vector[1]
    )

    if len(complex_idx) > 0:
        plt.scatter(
            traj_times_vec[complex_idx],
            element_times_vec[complex_idx],
            s=marker_size_complex,
            color=local_plan_color_vector[0]
        )
        plt.legend(['Simple', 'Complex'], loc='lower right')

    if titles:
        plt.title('Prediction Horizon')

    plt.xlabel('Current Time, i (s)')
    plt.ylabel('Predicted Time, k (s)')
    plt.axis([0, np.max(local_plan["time"]), 0, np.max(local_plan["time"])])

    # Export
    return [solve_data_fig, prediction_horizon_fig]
