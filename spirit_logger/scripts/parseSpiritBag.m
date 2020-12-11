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
stateEstimateData = readMessages(select(bag,'Topic','/state_estimate'),'DataFormat','struct');
stateEstimate = struct;
if isempty(stateEstimateData)
    warning('Warning, no data on state estimate topic');
else
    stateEstimate.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, stateEstimateData, 'UniformOutput', 0));
    
    stateEstimate.position = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Pose.Position.X, m.Body.Pose.Pose.Position.Y, m.Body.Pose.Pose.Position.Z], stateEstimateData, 'UniformOutput', 0));
    stateEstimate.velocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Twist.Linear.X, m.Body.Twist.Twist.Linear.Y, m.Body.Twist.Twist.Linear.Z], stateEstimateData, 'UniformOutput', 0));
    
    stateEstimate.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.Body.Pose.Pose.Orientation.W, m.Body.Pose.Pose.Orientation.X, m.Body.Pose.Pose.Orientation.Y, m.Body.Pose.Pose.Orientation.Z])), stateEstimateData, 'UniformOutput', 0));
    stateEstimate.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Pose.Orientation.W, m.Body.Pose.Pose.Orientation.X, m.Body.Pose.Pose.Orientation.Y, m.Body.Pose.Pose.Orientation.Z], stateEstimateData, 'UniformOutput', 0));
    stateEstimate.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Twist.Angular.X, m.Body.Twist.Twist.Angular.Y, m.Body.Twist.Twist.Angular.Z], stateEstimateData, 'UniformOutput', 0));
    
    stateEstimate.jointPosition = cell2mat(cellfun(@(m) m.Joints.Position.', stateEstimateData, 'UniformOutput', 0));
    stateEstimate.jointVelocity = cell2mat(cellfun(@(m) m.Joints.Velocity.', stateEstimateData, 'UniformOutput', 0));
    stateEstimate.jointEffort = cell2mat(cellfun(@(m) m.Joints.Effort.', stateEstimateData, 'UniformOutput', 0));
end

% Read the ground truth data
stateGroundTruthData = readMessages(select(bag,'Topic','/ground_truth_state'),'DataFormat','struct');
stateGroundTruth = struct;
if isempty(stateGroundTruthData)
    stateGroundTruthData = readMessages(select(bag,'Topic','gazebo/ground_truth_state'),'DataFormat','struct');
    if isempty(stateGroundTruthData)
        warning('Warning, no data on ground truth state topic');
    else
        disp('Found data on the gazebo/ground_truth_state topic')
    end
end

if ~isempty(stateEstimateData)
    stateGroundTruth.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, stateGroundTruthData, 'UniformOutput', 0));

    stateGroundTruth.position = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Pose.Position.X, m.Body.Pose.Pose.Position.Y, m.Body.Pose.Pose.Position.Z], stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.velocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Twist.Linear.X, m.Body.Twist.Twist.Linear.Y, m.Body.Twist.Twist.Linear.Z], stateGroundTruthData, 'UniformOutput', 0));

    stateGroundTruth.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.Body.Pose.Pose.Orientation.W, m.Body.Pose.Pose.Orientation.X, m.Body.Pose.Pose.Orientation.Y, m.Body.Pose.Pose.Orientation.Z])), stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Pose.Orientation.W, m.Body.Pose.Pose.Orientation.X, m.Body.Pose.Pose.Orientation.Y, m.Body.Pose.Pose.Orientation.Z], stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Twist.Angular.X, m.Body.Twist.Twist.Angular.Y, m.Body.Twist.Twist.Angular.Z], stateGroundTruthData, 'UniformOutput', 0));

    stateGroundTruth.jointPosition = cell2mat(cellfun(@(m) m.Joints.Position.', stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.jointVelocity = cell2mat(cellfun(@(m) m.Joints.Velocity.', stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.jointEffort = cell2mat(cellfun(@(m) m.Joints.Effort.', stateGroundTruthData, 'UniformOutput', 0));

    % Localize time to the first message
    startTime = min([stateGroundTruth.time(1), stateEstimate.time(1)]);
    stateGroundTruth.time = stateGroundTruth.time - startTime;
    stateEstimate.time = stateEstimate.time - startTime;
end

% Pack data into a struct for namespace purposes
data = struct;
data.stateEstimate = stateEstimate;
data.stateGroundTruth = stateGroundTruth;

% If prompted, return the name of the filename
if (nargout>1)
    varargout{1} = trialName;
end