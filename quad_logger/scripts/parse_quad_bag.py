""" MCAP log file parser for quadruped robotics data. """

import os
from collections import defaultdict
from tkinter import Tk, filedialog
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from scipy.spatial.transform import Rotation as R


def parse_quad_bag(trial_name=None):
    """
    Parse MCAP log file and extract quadruped robotics data.

    Args:
        trial_name (str, optional): Name of the trial log file (without .mcap extension).
            Defaults to "quad_log_current" if None.
        namespace (str, optional): Robot namespace (currently unused). Defaults to "".

    Returns:
        list: A list containing:
            - dict: Parsed data with keys for state_estimate, state_ground_truth,
                   state_trajectory, control_grfs, state_grfs, and local_plan
            - str: The trial name

    Raises:
        FileNotFoundError: If no log file is selected when file dialog is shown.
    """
    def load_mcap(trial_name=None):
        # Default to quad_log_current
        if trial_name is None:
            trial_name = "quad_log_current"

        # Construct expected path
        file_path = f"../bags/{trial_name}.mcap"

        # If file does not exist, open file chooser
        if not os.path.exists(file_path):
            print(f"{file_path} does not exist, choose a file manually...")

            root = Tk()
            root.withdraw()
            file_path = filedialog.askopenfilename(
                title="Select log file",
                filetypes=[("MCAP files", "*.mcap")]
            )
            root.destroy()

            if not file_path:
                raise FileNotFoundError("No log file selected.")

            # Extract trial name from selected file
            trial_name = os.path.basename(file_path).replace(".mcap", "")

        print(f"Loading: {file_path}...")

        # Open the MCAP file for reading
        with open(file_path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])

        return reader, file_path, trial_name

    reader, file_path, trial_name = load_mcap()
    print(file_path)

    topic_data = defaultdict(list)
    topic_timestamps = defaultdict(list)

    with open(file_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=[
                "/robot_1/state/estimate",
                "/robot_1/state/ground_truth",
                "/robot_1/state/trajectory",
                "/robot_1/control/grfs",
                "/robot_1/state/grfs",
                "/robot_1/local_plan"]):
            topic = channel.topic
            timestamp = message.log_time / 1e9  # convert nanosec to sec
            topic_data[topic].append(ros_msg)
            topic_timestamps[topic].append(timestamp)

    # Read state estimate data
    state_estimate_data = topic_data.get("/robot_1/state/estimate")
    state_estimate = {}

    if state_estimate_data is None:
        print('No data on state estimate topic')
    else:
        state_estimate["time"] = np.array(
            topic_timestamps.get("/robot_1/state/estimate"))
        state_estimate["position"] = np.array([[
            m.body.pose.position.x,
            m.body.pose.position.y,
            m.body.pose.position.z] for m in state_estimate_data])
        state_estimate["velocity"] = np.array([[
            m.body.twist.linear.x,
            m.body.twist.linear.y,
            m.body.twist.linear.z] for m in state_estimate_data])
        state_estimate["orientationQuat"] = np.array([[
            m.body.pose.orientation.w,
            m.body.pose.orientation.x,
            m.body.pose.orientation.y,
            m.body.pose.orientation.z] for m in state_estimate_data])
        state_estimate["orientationRPY"] = np.array([
            R.from_quat([m.body.pose.orientation.x,
                         m.body.pose.orientation.y,
                         m.body.pose.orientation.z,
                         m.body.pose.orientation.w]).as_euler("xyz")
            for m in state_estimate_data])
        state_estimate["angularVelocity"] = np.array([[
            m.body.twist.angular.x,
            m.body.twist.angular.y,
            m.body.twist.angular.z] for m in state_estimate_data])
        state_estimate["jointPosition"] = np.array([
            np.array(m.joints.position) for m in state_estimate_data])
        state_estimate["jointVelocity"] = np.array([
            np.array(m.joints.velocity) for m in state_estimate_data])
        state_estimate["jointEffort"] = np.array([
            np.array(m.joints.effort) for m in state_estimate_data])

    # Read ground truth data
    state_ground_truth_data = topic_data.get("/robot_1/state/ground_truth")
    state_ground_truth = {}

    if state_ground_truth_data is None:
        print('No data on ground truth state topic')
    else:
        state_ground_truth["time"] = np.array([
            float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9
            for m in state_ground_truth_data])
        state_ground_truth["position"] = np.array([
            [m.body.pose.position.x,
             m.body.pose.position.y,
             m.body.pose.position.z]
            for m in state_ground_truth_data])
        state_ground_truth["velocity"] = np.array([
            [m.body.twist.linear.x,
             m.body.twist.linear.y,
             m.body.twist.linear.z]
            for m in state_ground_truth_data])
        state_ground_truth["orientationQuat"] = np.array([
            [m.body.pose.orientation.w,
             m.body.pose.orientation.x,
             m.body.pose.orientation.y,
             m.body.pose.orientation.z]
            for m in state_ground_truth_data])
        state_ground_truth["orientationRPY"] = np.array([
            R.from_quat([m.body.pose.orientation.x,
                         m.body.pose.orientation.y,
                         m.body.pose.orientation.z,
                         m.body.pose.orientation.w]).as_euler("xyz")
            for m in state_ground_truth_data])
        state_ground_truth["angularVelocity"] = np.array([
            [m.body.twist.angular.x,
             m.body.twist.angular.y,
             m.body.twist.angular.z]
            for m in state_ground_truth_data])
        state_ground_truth["jointPosition"] = np.vstack([
            np.array(m.joints.position)
            for m in state_ground_truth_data])
        state_ground_truth["jointVelocity"] = np.vstack([
            np.array(m.joints.velocity)
            for m in state_ground_truth_data])
        state_ground_truth["jointEffort"] = np.vstack([
            np.array(m.joints.effort)
            for m in state_ground_truth_data])
        num_feet = len(state_ground_truth_data[0].feet.feet)
        state_ground_truth["footPosition"] = []
        state_ground_truth["footVelocity"] = []
        for i in range(num_feet):
            state_ground_truth["footPosition"].append(np.array([
                [m.feet.feet[i].position.x,
                 m.feet.feet[i].position.y,
                 m.feet.feet[i].position.z]
                for m in state_ground_truth_data]))
            state_ground_truth["footVelocity"].append(np.array([
                [m.feet.feet[i].velocity.x,
                 m.feet.feet[i].velocity.y,
                 m.feet.feet[i].velocity.z]
                for m in state_ground_truth_data]))

    # Read trajectory data
    state_trajectory_data = topic_data.get("/robot_1/state/trajectory")
    state_trajectory = {}

    if state_trajectory_data is None:
        print('No data on trajectory topic')
    else:
        state_trajectory["time"] = np.array([
            float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9
            for m in state_trajectory_data])
        state_trajectory["position"] = np.array([
            [m.body.pose.position.x,
             m.body.pose.position.y,
             m.body.pose.position.z]
            for m in state_trajectory_data])
        state_trajectory["velocity"] = np.array([
            [m.body.twist.linear.x,
             m.body.twist.linear.y,
             m.body.twist.linear.z]
            for m in state_trajectory_data])
        state_trajectory["orientationQuat"] = np.array([
            [m.body.pose.orientation.w,
             m.body.pose.orientation.x,
             m.body.pose.orientation.y,
             m.body.pose.orientation.z]
            for m in state_trajectory_data])
        state_trajectory["orientationRPY"] = np.array([
            R.from_quat([m.body.pose.orientation.x,
                         m.body.pose.orientation.y,
                         m.body.pose.orientation.z,
                         m.body.pose.orientation.w]).as_euler("xyz")
            for m in state_trajectory_data])
        state_trajectory["angularVelocity"] = np.array([
            [m.body.twist.angular.x,
             m.body.twist.angular.y,
             m.body.twist.angular.z]
            for m in state_trajectory_data])

    # Read control GRF data
    control_grfs_data = topic_data.get("/robot_1/control/grfs")
    control_grfs = {}
    control_grfs["vectors"] = []
    control_grfs["points"] = []
    control_grfs["contactStates"] = []

    if control_grfs_data is None:
        print('No data on grf control topic')
    else:
        control_grfs["time"] = np.array([
            float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9
            for m in control_grfs_data])
        num_feet = 4
        for i in range(num_feet):
            try:
                control_grfs["vectors"].append(np.array([
                    [m.vectors[i].x, m.vectors[i].y, m.vectors[i].z]
                    for m in control_grfs_data]))
                control_grfs["points"].append(np.array([
                    [m.points[i].x, m.points[i].y, m.points[i].z]
                    for m in control_grfs_data]))
                control_grfs["contactStates"].append(np.array([
                    [m.contact_states[i],
                     m.contact_states[i],
                     m.contact_states[i]]
                    for m in control_grfs_data]))
            except (IndexError, AttributeError):
                control_grfs["vectors"].append(np.array([
                    [0, 0, 0] for m in control_grfs_data]))
                control_grfs["points"].append(np.array([
                    [0, 0, 0] for m in control_grfs_data]))
                control_grfs["contactStates"].append(np.array([
                    [0, 0, 0] for m in control_grfs_data]))

    # Read state GRF data
    state_grfs_data = topic_data.get("/robot_1/state/grfs")
    state_grfs = {}
    state_grfs["vectors"] = []
    state_grfs["points"] = []
    state_grfs["contactStates"] = []

    if len(state_grfs_data) == 0:
        print('No data on grf state topic')
    else:
        state_grfs["time"] = np.array([
            float(m.header.stamp.sec) + float(m.header.stamp.nanosec) * 1e-9
            for m in state_grfs_data])
        num_feet = 4
        for i in range(num_feet):
            try:
                state_grfs["vectors"].append(np.array([
                    [m.vectors[i].x, m.vectors[i].y, m.vectors[i].z]
                    for m in state_grfs_data]))
                state_grfs["points"].append(np.array([
                    [m.points[i].x, m.points[i].y, m.points[i].z]
                    for m in state_grfs_data]))
                state_grfs["contactStates"].append(np.array([
                    [m.contact_states[i],
                     m.contact_states[i],
                     m.contact_states[i]]
                    for m in state_grfs_data]))
            except (IndexError, AttributeError):
                state_grfs["vectors"].append(np.array([
                    [0, 0, 0] for m in state_grfs_data]))
                state_grfs["points"].append(np.array([
                    [0, 0, 0] for m in state_grfs_data]))
                state_grfs["contactStates"].append(np.array([
                    [0, 0, 0] for m in state_grfs_data]))

    # Read local plan data
    local_plan_data = topic_data.get("/robot_1/local_plan")
    local_plan = {}

    for _, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=["/local_plan"]):
        local_plan_data.append(ros_msg)

    if len(local_plan_data) == 0:
        print('No data on local plan topic')
    else:
        local_plan["time"] = np.array([
            float(m.state_timestamp.sec) +
            float(m.state_timestamp.nanosec) * 1e-9
            for m in local_plan_data])
        local_plan["elementTimes"] = [
            np.array(m.diagnostics.element_times, dtype=float)
            for m in local_plan_data]
        local_plan["solveTime"] = np.array([
            m.diagnostics.compute_time for m in local_plan_data])
        local_plan["cost"] = np.array([
            m.diagnostics.cost for m in local_plan_data])
        local_plan["iterations"] = np.array([
            m.diagnostics.iterations for m in local_plan_data])
        local_plan["horizonLength"] = np.array([
            m.diagnostics.horizon_length for m in local_plan_data])
        local_plan["complexitySchedule"] = [
            np.array(m.diagnostics.complexity_schedule, dtype=float)
            for m in local_plan_data]

    # Localize time to the first message
    start_time = state_ground_truth["time"][0]
    data = {}

    # Update time of existing messages and pack them into the dictionary
    if len(state_estimate.keys()) != 0:
        state_estimate["time"] = state_estimate["time"] - start_time
        data["state_estimate"] = state_estimate
    else:
        data["state_estimate"] = []

    if len(state_ground_truth.keys()) != 0:
        state_ground_truth["time"] = state_ground_truth["time"] - start_time
        data["state_ground_truth"] = state_ground_truth
    else:
        data["state_ground_truth"] = []

    if len(state_trajectory.keys()) != 0:
        state_trajectory["time"] = state_trajectory["time"] - start_time
        data["state_trajectory"] = state_trajectory
    else:
        data["state_trajectory"] = []

    if len(control_grfs.keys()) != 0:
        control_grfs["time"] = control_grfs["time"] - start_time
        data["control_grfs"] = control_grfs
    else:
        data["control_grfs"] = []

    if len(state_grfs.keys()) != 0:
        state_grfs["time"] = state_grfs["time"] - start_time
        data["state_grfs"] = state_grfs
    else:
        data["state_grfs"] = []

    if len(local_plan.keys()) != 0:
        local_plan["time"] = local_plan["time"] - start_time
        for i in range(len(local_plan["elementTimes"])):
            shifted_element_times = (local_plan["elementTimes"][i] +
                                     local_plan["time"][i])
            local_plan["elementTimes"][i] = shifted_element_times
        data["local_plan"] = local_plan
    else:
        data["local_plan"] = []

    return [data, trial_name]
