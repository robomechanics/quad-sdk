"""
Plot logging utility for quadruped robotics analysis.

This module provides functionality to save matplotlib figures to organized
log directories, creating a persistent record of analysis plots for each trial run.
"""

import os
import matplotlib.pyplot as plt

def save_log(trial_name, fig_array):
    """
    Save generated matplotlib figures to a trial-specific log directory.

    Creates a log directory structure under 'logs/{trial_name}/' if it doesn't
    exist, then saves each figure in the provided array as a numbered PNG file.

    Args:
        trial_name (str): Name of the trial, used to create the log subdirectory.
        fig_array (list): List of matplotlib Figure objects to be saved.
                         Can be nested lists that will be flattened.

    Returns:
        str: Path to the log directory where figures were saved.
    """

    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    print(parent_dir)
    log_dir = os.path.join(parent_dir, "logs", trial_name)
    figures_dir = os.path.join(log_dir, "figures")
    os.makedirs(figures_dir, exist_ok=True)

    # Flatten nested lists
    flat_figs = []
    for item in fig_array:
        if isinstance(item, list):
            flat_figs.extend(item)
        else:
            flat_figs.append(item)

    # Save each figure
    for i, fig in enumerate(flat_figs):
        # If needed, convert figure number to object
        if isinstance(fig, int):
            fig = plt.figure(fig)
        fig_path = os.path.join(log_dir, f"figure_{i+1}.png")
        print(f"Saving: {fig_path}")
        fig.savefig(fig_path)
    print(f"Saved to: {os.path.abspath(log_dir)}")
    
    return log_dir
