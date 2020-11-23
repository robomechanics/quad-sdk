close all;

% Load bag and split into components
bag = rosbag('../bags/spirit_log_current.bag');
ground_truth_state_bag = select(bag, 'Topic', '/ground_truth_state');
state_estimate_bag = select(bag, 'Topic', '/state_estimate');
controller_bag = select(bag, 'Topic', '/spirit/joint_controller/command');
contact_mode_bag = select(bag, 'Topic', '/contact_mode');

% Read data from component bags
ground_truth_state = readMessages(ground_truth_state_bag,'DataFormat','struct');
state_estimate = readMessages(state_estimate_bag,'DataFormat','struct');
controller = readMessages(controller_bag,'DataFormat','struct');
contact_mode = readMessages(contact_mode_bag,'DataFormat','struct');

% Initialize data variables
num_msgs = length(ground_truth_state);
t = 0; x = 0; y = 0; z = 0; roll = 0; pitch = 0; yaw = 0;
t_start = double(ground_truth_state{1}.Header.Stamp.Sec)+...
    double(ground_truth_state{1}.Header.Stamp.Nsec)*10^-9;

% Loop through messages pulling relevant data (replace with timeseries in
% 2019b with rosgenmsg
for i = 1:num_msgs
    t_local = ground_truth_state{i}.Header.Stamp;
    t = [t; double(t_local.Sec)+double(t_local.Nsec)*10^-9 - t_start];
    x = [x; ground_truth_state{i}.Body.Pose.Pose.Position.X];
    y = [y; ground_truth_state{i}.Body.Pose.Pose.Position.Y];
    z = [z; ground_truth_state{i}.Body.Pose.Pose.Position.Z];
    
    quat = ground_truth_state{i}.Body.Pose.Pose.Orientation;
    eul = quat2eul([quat.W,quat.X,quat.Y,quat.Z]);
    roll = [roll,eul(3)];
    pitch = [pitch,eul(2)];
    yaw = [yaw,eul(1)];
end

% Plot the results
plot(t,z)
xlabel('Time (s)')
ylabel('Z (m)')