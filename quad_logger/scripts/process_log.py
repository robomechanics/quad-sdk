""" Module for processing quadruped log files and generating plots. """

import os
import numpy as np
import matplotlib.pyplot as plt
from parse_quad_bag import parse_quad_bag
from plot_state import plot_state
from plot_control import plot_control
from plot_local_plan import plot_local_plan
from save_log import save_log


def process_log(*args):
    """Process quadruped log file and generate plots.

    Args:
        *args: Optional trial name and namespace.
    """
    # Prepare environment; close all open plots/figures
    plt.close('all')

    # Ensure that script is located in quad_logger/scripts/ directory
    if not os.getcwd().endswith("quad_logger/scripts"):
        print("This script must be run from quad_logger/scripts/")

    # Select a mcap file to parse through, if a trial name is provided use that
    if len(args) > 0:
        trial_name = args[0]
        namespace = args[1]
    else:
        trial_name = None
        namespace = ""

    # Set parameters to save and set up variables for figure titles and labels
    save = False
    titles = True
    plot_local_plan_info = True
    window_states = []
    window_control = []
    window_local_plan = []

    # Load data from parseQuadBag.py
    data, trial_name = parse_quad_bag(trial_name)

    state_estimate = data["state_estimate"]
    state_ground_truth = data["state_ground_truth"]
    state_trajectory = data["state_trajectory"]
    state_grfs = data["state_grfs"]
    control_grfs = data["control_grfs"]
    local_plan = data["local_plan"]


    # Plot everything, separate by:
    # State
    state_figs = np.array([])
    if state_ground_truth:
        state_figs = plot_state(state_ground_truth, window_states, '-', titles, state_figs)
    if state_trajectory:
        state_figs = plot_state(state_trajectory, window_states, '--', titles, state_figs)
    if state_estimate:
        state_figs = plot_state(state_estimate, window_states, ':', titles, state_figs)

    # Control
    control_figs = np.array([])
    if state_grfs:
        control_figs = plot_control(state_grfs, window_control, ':', titles, control_figs)
    if control_grfs:
        control_figs = plot_control(control_grfs, window_control, '--', titles, control_figs)

    # Local Plan Information
    local_plan_figs = np.array([])
    if plot_local_plan_info and local_plan:
        local_plan_figs = plot_local_plan(local_plan, window_local_plan, '-', titles,
                                        local_plan_figs)

    # Consolidate the figures to a single folder
    fig_array = np.concatenate([state_figs, control_figs, local_plan_figs])


    if save:
        save_log(trial_name, fig_array)

    plt.show()


if __name__ == "__main__":
    process_log()
