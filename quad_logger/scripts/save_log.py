"""
Plot logging utility for quadruped robotics analysis.

This module provides functionality to save matplotlib figures to organized
log directories, creating a persistent record of analysis plots for each trial run.
"""


import os

def save_log(trial_name, fig_array):
    """
    Save generated matplotlib figures to a trial-specific log directory.

    Creates a log directory structure under 'logs/{trial_name}/' if it doesn't
    exist, then saves each figure in the provided array as a numbered PNG file.

    Args:
        trial_name (str): Name of the trial, used to create the log subdirectory.
        fig_array (list): List of matplotlib Figure objects to be saved.

    Returns:
        str: Path to the log directory where figures were saved.
    """
    log_dir = os.path.join("logs", trial_name)
    os.makedirs(log_dir, exist_ok=True)

    for i, fig in enumerate(fig_array):
        fig_path = os.path.join(log_dir, f"figure_{i+1}.png")
        fig.savefig(fig_path)
    return log_dir
