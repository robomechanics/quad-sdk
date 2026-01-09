# Specify file path
import os
from tkinter import Tk, filedialog
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

def load_mcap(trial_name=None, namespace=""):
    # Default to quad_log_current
    if trial_name is None:
        trial_name = "quad_log_current"

    # Construct expected path
    filepath = f"../bags/{trial_name}.mcap"

    # If file does not exist, open file chooser
    if not os.path.exists(filepath):
        print(f"{filepath} does not exist, choose a file manually...")
        
        # Open file selection window
        root = Tk()
        root.withdraw()  # Hide GUI window
        filepath = filedialog.askopenfilename(
            title="Select log file",
            filetypes=[("MCAP files", "*.mcap")]
        )
        root.destroy()

        if not filepath:
            raise FileNotFoundError("No log file selected.")

        # Extract trial name from selected file
        trial_name = os.path.basename(filepath).replace(".mcap", "")

    print(f"Loading: {filepath}...")

    # Open the MCAP file for reading
    f = open(filepath, "rb")
    reader = make_reader(f, decoder_factories=[DecoderFactory()])

    return reader, filepath, trial_name

reader, filepath, trial_name = load_mcap()


# Load bag / mcap file    
timestamps = []

joint_angle_data = []
joint_velocity_data = []
joint_effort_data = []

pos_foot_x = []
pos_foot_y = []
pos_foot_z = []

vel_foot_x = []
vel_foot_y = []
vel_foot_z = []

pos_body_x = []
pos_body_y = []
pos_body_z = []

linear_quaternion_data = [] 
angular_quaternion_data = [] 
for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=["/robot_1/state/ground_truth"]):
    timestamp = message.log_time / 1e9
    timestamps.append(timestamp)

    joint_angle_data.append(ros_msg.joints.position) 
    joint_velocity_data.append(ros_msg.joints.velocity) 
    joint_effort_data.append(ros_msg.joints.effort) 

    pos_foot_x.append([foot.position.x for foot in ros_msg.feet.feet])
    pos_foot_y.append([foot.position.y for foot in ros_msg.feet.feet])
    pos_foot_z.append([foot.position.z for foot in ros_msg.feet.feet])

    vel_foot_x.append([foot.velocity.x for foot in ros_msg.feet.feet])
    vel_foot_y.append([foot.velocity.y for foot in ros_msg.feet.feet])
    vel_foot_z.append([foot.velocity.z for foot in ros_msg.feet.feet])

    # Body pose
    pos_body_x.append(ros_msg.body.pose.position.x)
    pos_body_y.append(ros_msg.body.pose.position.y)
    pos_body_z.append(ros_msg.body.pose.position.z)


    linear_q = ros_msg.body.twist.linear
    linear_quaternion_data.append([linear_q.x, linear_q.y, linear_q.z])
    angular_q = ros_msg.body.twist.angular
    angular_quaternion_data.append([angular_q.x, angular_q.y, angular_q.z])
# Read state estimate data

# Read ground truth data

# Read ground truth body and frame data

# Read trajectory data

# Read control GRF data

# Read state GRF data

# Read cmd vel data

# Read local plan data

# Localize time to the first message

# Update time of existing messages and pack it into a struct variable

# If prompted, return name of filename