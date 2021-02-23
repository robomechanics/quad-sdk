function [data, varargout] = parseSpiritBag(varargin)
% parseSpiritBag parse a spirit data log file
%   DATA = parseSpiritBag uses the default 'spirit_log_current' file name to
%   yield a data structure containing select topic data. If this bag does
%   not exist, the user can select the bag via a UI.
%
%   DATA = parseSpiritBag(FILENAME) uses the data in a bag with the specified
%   file name, looking in '../bags/'.

% Default to spirit_log_current
if nargin == 0
    trialName = 'spirit_log_current';
else
    trialName = varargin{1};
end

% Specify the path
filepath = ['../bags/', trialName,'.bag'];
if ~(exist(filepath,'file'))
    disp([filepath, ' does not exist, using UI to specify path']);
    [fileName,pathname] = uigetfile('.bag', 'Select a Bag');
    
    suffix = '.bag';
    trialName = fileName(1:(end-length(suffix)));
    filepath = fullfile(pathname, fileName);
end

% Load the bag
bag = rosbag(filepath);


% Read the state estimate data
state_estimate_data = readMessages(select(bag,'Topic','/state/estimate'),'DataFormat','struct');
state_estimate = struct;
if isempty(state_estimate_data)
    warning('Warning, no data on state estimate topic');
else
    state_estimate.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, state_estimate_data, 'UniformOutput', 0));
    
    state_estimate.position = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Pose.Position.X, m.Body.Pose.Pose.Position.Y, m.Body.Pose.Pose.Position.Z], state_estimate_data, 'UniformOutput', 0));
    state_estimate.velocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Twist.Linear.X, m.Body.Twist.Twist.Linear.Y, m.Body.Twist.Twist.Linear.Z], state_estimate_data, 'UniformOutput', 0));
    
    state_estimate.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.Body.Pose.Pose.Orientation.W, m.Body.Pose.Pose.Orientation.X, m.Body.Pose.Pose.Orientation.Y, m.Body.Pose.Pose.Orientation.Z])), state_estimate_data, 'UniformOutput', 0));
    state_estimate.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Pose.Orientation.W, m.Body.Pose.Pose.Orientation.X, m.Body.Pose.Pose.Orientation.Y, m.Body.Pose.Pose.Orientation.Z], state_estimate_data, 'UniformOutput', 0));
    state_estimate.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Twist.Angular.X, m.Body.Twist.Twist.Angular.Y, m.Body.Twist.Twist.Angular.Z], state_estimate_data, 'UniformOutput', 0));
    
    state_estimate.jointPosition = cell2mat(cellfun(@(m) m.Joints.Position.', state_estimate_data, 'UniformOutput', 0));
    state_estimate.jointVelocity = cell2mat(cellfun(@(m) m.Joints.Velocity.', state_estimate_data, 'UniformOutput', 0));
    state_estimate.jointEffort = cell2mat(cellfun(@(m) m.Joints.Effort.', state_estimate_data, 'UniformOutput', 0));
end

% Read the ground truth data
state_ground_truth_data = readMessages(select(bag,'Topic','/state/ground_truth'),'DataFormat','struct');
state_ground_truth = struct;
if isempty(state_ground_truth_data)
    warning('Warning, no data on ground truth state topic');
else
    
    state_ground_truth.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, state_ground_truth_data, 'UniformOutput', 0));
    
    state_ground_truth.position = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Pose.Position.X, m.Body.Pose.Pose.Position.Y, m.Body.Pose.Pose.Position.Z], state_ground_truth_data, 'UniformOutput', 0));
    state_ground_truth.velocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Twist.Linear.X, m.Body.Twist.Twist.Linear.Y, m.Body.Twist.Twist.Linear.Z], state_ground_truth_data, 'UniformOutput', 0));
    
    state_ground_truth.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.Body.Pose.Pose.Orientation.W, m.Body.Pose.Pose.Orientation.X, m.Body.Pose.Pose.Orientation.Y, m.Body.Pose.Pose.Orientation.Z])), state_ground_truth_data, 'UniformOutput', 0));
    state_ground_truth.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Pose.Orientation.W, m.Body.Pose.Pose.Orientation.X, m.Body.Pose.Pose.Orientation.Y, m.Body.Pose.Pose.Orientation.Z], state_ground_truth_data, 'UniformOutput', 0));
    state_ground_truth.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Twist.Angular.X, m.Body.Twist.Twist.Angular.Y, m.Body.Twist.Twist.Angular.Z], state_ground_truth_data, 'UniformOutput', 0));
    
    state_ground_truth.jointPosition = cell2mat(cellfun(@(m) m.Joints.Position.', state_ground_truth_data, 'UniformOutput', 0));
    state_ground_truth.jointVelocity = cell2mat(cellfun(@(m) m.Joints.Velocity.', state_ground_truth_data, 'UniformOutput', 0));
    state_ground_truth.jointEffort = cell2mat(cellfun(@(m) m.Joints.Effort.', state_ground_truth_data, 'UniformOutput', 0));
end

% Read the ground truth data
state_trajectory_data = readMessages(select(bag,'Topic','/state/trajectory'),'DataFormat','struct');
state_trajectory = struct;
if isempty(state_trajectory_data)
    warning('Warning, no data on ground truth state topic');
else
    
    state_trajectory.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, state_trajectory_data, 'UniformOutput', 0));
    
    state_trajectory.position = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Pose.Position.X, m.Body.Pose.Pose.Position.Y, m.Body.Pose.Pose.Position.Z], state_trajectory_data, 'UniformOutput', 0));
    state_trajectory.velocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Twist.Linear.X, m.Body.Twist.Twist.Linear.Y, m.Body.Twist.Twist.Linear.Z], state_trajectory_data, 'UniformOutput', 0));
    
    state_trajectory.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.Body.Pose.Pose.Orientation.W, m.Body.Pose.Pose.Orientation.X, m.Body.Pose.Pose.Orientation.Y, m.Body.Pose.Pose.Orientation.Z])), state_trajectory_data, 'UniformOutput', 0));
    state_trajectory.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Pose.Orientation.W, m.Body.Pose.Pose.Orientation.X, m.Body.Pose.Pose.Orientation.Y, m.Body.Pose.Pose.Orientation.Z], state_trajectory_data, 'UniformOutput', 0));
    state_trajectory.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Twist.Angular.X, m.Body.Twist.Twist.Angular.Y, m.Body.Twist.Twist.Angular.Z], state_trajectory_data, 'UniformOutput', 0));
    
    state_trajectory.jointPosition = cell2mat(cellfun(@(m) m.Joints.Position.', state_trajectory_data, 'UniformOutput', 0));
    state_trajectory.jointVelocity = cell2mat(cellfun(@(m) m.Joints.Velocity.', state_trajectory_data, 'UniformOutput', 0));
    state_trajectory.jointEffort = cell2mat(cellfun(@(m) m.Joints.Effort.', state_trajectory_data, 'UniformOutput', 0));
end

% Localize time to the first message
startTime = min([state_ground_truth.time(1), state_estimate.time(1), state_trajectory.time(1)]);
state_estimate.time = state_estimate.time - startTime;
state_ground_truth.time = state_ground_truth.time - startTime;
state_trajectory.time = state_trajectory.time - startTime;

% Pack data into a struct for namespace purposes
data = struct;
data.state_estimate = state_estimate;
data.state_ground_truth = state_ground_truth;
data.state_trajectory = state_trajectory;

% If prompted, return the name of the filename
if (nargout>1)
    varargout{1} = trialName;
end