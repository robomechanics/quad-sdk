import importlib
import os
import sys
import types

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


def test_save_log_flattens_figures_and_writes_pngs(tmp_path, monkeypatch):
    import save_log

    scripts_dir = tmp_path / "quad_logger" / "scripts"
    scripts_dir.mkdir(parents=True)
    monkeypatch.setattr(save_log, "__file__", str(scripts_dir / "save_log.py"))

    fig1 = plt.figure()
    fig2 = plt.figure()

    log_dir = save_log.save_log("trial_a", [[fig1], fig2])

    assert log_dir == str(tmp_path / "quad_logger" / "logs" / "trial_a")
    assert (tmp_path / "quad_logger" / "logs" / "trial_a" / "figure_1.png").exists()
    assert (tmp_path / "quad_logger" / "logs" / "trial_a" / "figure_2.png").exists()
    assert (tmp_path / "quad_logger" / "logs" / "trial_a" / "figures").is_dir()


def test_plot_control_filters_time_window_and_returns_figure():
    from plot_control import plot_control

    control = {
        "time": np.array([0.0, 0.5, 1.0]),
        "vectors": [np.ones((3, 3)) * i for i in range(4)],
        "points": [np.ones((3, 3)) * (i + 10) for i in range(4)],
        "contactStates": [np.ones((3, 3), dtype=bool) for _ in range(4)],
    }

    figures = plot_control(control, [0.0, 0.75], "--", False, [])

    assert len(figures) == 1
    assert control["time"].tolist() == [0.0, 0.5]
    assert control["vectors"][3].shape == (2, 3)


def test_plot_local_plan_filters_solver_and_horizon_data():
    from plot_local_plan import plot_local_plan

    local_plan = {
        "time": np.array([0.0, 0.5, 1.0]),
        "solveTime": np.array([0.01, 0.02, 0.03]),
        "elementTimes": [
            np.array([0.0, 0.1]),
            np.array([0.5, 0.6]),
            np.array([1.0, 1.1]),
        ],
        "cost": np.array([1.0, 2.0, 3.0]),
        "iterations": np.array([4, 5, 6]),
        "horizonLength": np.array([2, 2, 2]),
        "complexitySchedule": [
            np.array([0, 0]),
            np.array([0, 1]),
            np.array([1, 1]),
        ],
    }

    figures = plot_local_plan(local_plan, [0.25, 1.0], "-", False, [])

    assert len(figures) == 2
    assert local_plan["time"].tolist() == [0.5, 1.0]
    assert local_plan["solveTime"].tolist() == [0.02, 0.03]
    assert local_plan["complexitySchedule"].shape == (2, 2)


def test_plot_state_handles_full_state_and_foot_data():
    from plot_state import plot_state

    state = {
        "time": np.array([0.0, 0.5, 1.0]),
        "position": np.array([[0.0, 0.0, 0.3], [0.1, 0.0, 0.3], [0.2, 0.0, 0.3]]),
        "velocity": np.ones((3, 3)) * 0.1,
        "orientationRPY": np.zeros((3, 3)),
        "orientationQuat": np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (3, 1)),
        "angularVelocity": np.zeros((3, 3)),
        "jointPosition": np.zeros((3, 12)),
        "jointVelocity": np.zeros((3, 12)),
        "jointEffort": np.zeros((3, 12)),
        "footPosition": [np.ones((3, 3)) * i for i in range(4)],
        "footVelocity": [np.ones((3, 3)) * 0.1 for _ in range(4)],
    }

    figures = plot_state(state, [0.25, 0.75], "-", False, [])

    assert len(figures) == 8
    assert state["time"].tolist() == [0.5]
    assert state["position"].shape == (1, 3)
    assert state["footPosition"][0].shape == (1, 3)


def test_process_log_dispatches_available_data(monkeypatch, tmp_path):
    parse_module = types.ModuleType("parse_quad_bag")
    plot_state_module = types.ModuleType("plot_state")
    plot_control_module = types.ModuleType("plot_control")
    plot_local_plan_module = types.ModuleType("plot_local_plan")
    save_log_module = types.ModuleType("save_log")

    calls = []

    def fake_parse_quad_bag(trial_name=None):
        calls.append(("parse", trial_name))
        return (
            {
                "state_estimate": {},
                "state_ground_truth": {"time": np.array([0.0])},
                "state_trajectory": [],
                "state_grfs": {"time": np.array([0.0])},
                "control_grfs": [],
                "local_plan": {"time": np.array([0.0])},
            },
            "trial_b",
        )

    def fake_plot_state(data, window, line_style, titles, figures):
        calls.append(("state", line_style, bool(titles)))
        return ["state_fig"]

    def fake_plot_control(data, window, line_style, titles, figures):
        calls.append(("control", line_style, bool(titles)))
        return ["control_fig"]

    def fake_plot_local_plan(data, window, line_style, titles, figures):
        calls.append(("local_plan", line_style, bool(titles)))
        return ["local_plan_fig"]

    def fake_save_log(trial_name, fig_array):
        calls.append(("save", trial_name, fig_array))
        return str(tmp_path)

    parse_module.parse_quad_bag = fake_parse_quad_bag
    plot_state_module.plot_state = fake_plot_state
    plot_control_module.plot_control = fake_plot_control
    plot_local_plan_module.plot_local_plan = fake_plot_local_plan
    save_log_module.save_log = fake_save_log

    monkeypatch.setitem(sys.modules, "parse_quad_bag", parse_module)
    monkeypatch.setitem(sys.modules, "plot_state", plot_state_module)
    monkeypatch.setitem(sys.modules, "plot_control", plot_control_module)
    monkeypatch.setitem(sys.modules, "plot_local_plan", plot_local_plan_module)
    monkeypatch.setitem(sys.modules, "save_log", save_log_module)
    monkeypatch.setattr(plt, "show", lambda: calls.append(("show",)))

    sys.modules.pop("process_log", None)
    process_log = importlib.import_module("process_log")
    process_log.process_log("trial_b", "robot_1")

    assert ("parse", "trial_b") in calls
    assert ("state", "-", True) in calls
    assert ("control", ":", True) in calls
    assert ("local_plan", "-", True) in calls
    assert any(call[0] == "save" and call[1] == "trial_b" for call in calls)
    assert ("show",) in calls
