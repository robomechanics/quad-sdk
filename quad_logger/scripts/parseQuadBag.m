function [data, varargout] = parseQuadBag(varargin)
% parseQuadBag parse a quad data log file
%   DATA = parseQuadBag uses the default 'quad_log_current' file name to
%   yield a data structure containing select topic data. If this bag does
%   not exist, the user can select the bag via a UI.
%
%   DATA = parseQuadBag(FILENAME) uses the data in a bag with the specified
%   file name, looking in '../bags/'.

% Default to quad_log_current
if nargin == 0
    trialName = 'quad_log_current';
else
    trialName = varargin{1};
end

% Specify the path
filepath = trialName;
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
stateEstimateData = readMessages(select(bag,'Topic','/state/estimate'),'DataFormat','struct');
stateEstimate = struct;
if isempty(stateEstimateData)
    warning('Warning, no data on state estimate topic');
else
    stateEstimate.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, stateEstimateData, 'UniformOutput', 0));
    
    stateEstimate.position = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Position.X, m.Body.Pose.Position.Y, m.Body.Pose.Position.Z], stateEstimateData, 'UniformOutput', 0));
    stateEstimate.velocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Linear.X, m.Body.Twist.Linear.Y, m.Body.Twist.Linear.Z], stateEstimateData, 'UniformOutput', 0));
    
    stateEstimate.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.Body.Pose.Orientation.W, m.Body.Pose.Orientation.X, m.Body.Pose.Orientation.Y, m.Body.Pose.Orientation.Z])), stateEstimateData, 'UniformOutput', 0));
    stateEstimate.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Orientation.W, m.Body.Pose.Orientation.X, m.Body.Pose.Orientation.Y, m.Body.Pose.Orientation.Z], stateEstimateData, 'UniformOutput', 0));
    stateEstimate.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Angular.X, m.Body.Twist.Angular.Y, m.Body.Twist.Angular.Z], stateEstimateData, 'UniformOutput', 0));
    
    stateEstimate.jointPosition = cell2mat(cellfun(@(m) m.Joints.Position.', stateEstimateData, 'UniformOutput', 0));
    stateEstimate.jointVelocity = cell2mat(cellfun(@(m) m.Joints.Velocity.', stateEstimateData, 'UniformOutput', 0));
    stateEstimate.jointEffort = cell2mat(cellfun(@(m) m.Joints.Effort.', stateEstimateData, 'UniformOutput', 0));
end

% Read the ground truth data
stateGroundTruthData = readMessages(select(bag,'Topic','/state/ground_truth'),'DataFormat','struct');
stateGroundTruth = struct;
if isempty(stateGroundTruthData)
    warning('Warning, no data on ground truth state topic');
else
    
    stateGroundTruth.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, stateGroundTruthData, 'UniformOutput', 0));
    
    stateGroundTruth.position = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Position.X, m.Body.Pose.Position.Y, m.Body.Pose.Position.Z], stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.velocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Linear.X, m.Body.Twist.Linear.Y, m.Body.Twist.Linear.Z], stateGroundTruthData, 'UniformOutput', 0));
    
    stateGroundTruth.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.Body.Pose.Orientation.W, m.Body.Pose.Orientation.X, m.Body.Pose.Orientation.Y, m.Body.Pose.Orientation.Z])), stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Orientation.W, m.Body.Pose.Orientation.X, m.Body.Pose.Orientation.Y, m.Body.Pose.Orientation.Z], stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Angular.X, m.Body.Twist.Angular.Y, m.Body.Twist.Angular.Z], stateGroundTruthData, 'UniformOutput', 0));
    
    stateGroundTruth.jointPosition = cell2mat(cellfun(@(m) m.Joints.Position.', stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.jointVelocity = cell2mat(cellfun(@(m) m.Joints.Velocity.', stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.jointEffort = cell2mat(cellfun(@(m) m.Joints.Effort.', stateGroundTruthData, 'UniformOutput', 0));
    
    if ~isempty(stateGroundTruthData{1}.TailJoints.Position)
        stateGroundTruth.tailJointPosition = cell2mat(cellfun(@(m) m.TailJoints.Position.', stateGroundTruthData, 'UniformOutput', 0));
        stateGroundTruth.tailJointVelocity = cell2mat(cellfun(@(m) m.TailJoints.Velocity.', stateGroundTruthData, 'UniformOutput', 0));
        stateGroundTruth.tailJointEffort = cell2mat(cellfun(@(m) m.TailJoints.Effort.', stateGroundTruthData, 'UniformOutput', 0));
    else
        stateGroundTruth.tailJointPosition = [];
        stateGroundTruth.tailJointVelocity = [];
        stateGroundTruth.tailJointEffort = [];
    end
        
    num_feet = size(stateGroundTruthData{1}.Feet.Feet, 2);
    for i = 1:num_feet
        stateGroundTruth.footPosition{i} = cell2mat(cellfun(@(m) ...
            [m.Feet.Feet(i).Position.X, m.Feet.Feet(i).Position.Y, m.Feet.Feet(i).Position.Z], stateGroundTruthData, 'UniformOutput', 0));
        stateGroundTruth.footVelocity{i} = cell2mat(cellfun(@(m) ...
            [m.Feet.Feet(i).Velocity.X, m.Feet.Feet(i).Velocity.Y, m.Feet.Feet(i).Velocity.Z], stateGroundTruthData, 'UniformOutput', 0));
    end
end

% Read the trajectory data
stateTrajectoryData = readMessages(select(bag,'Topic','/state/trajectory'),'DataFormat','struct');
stateTrajectory = struct;
if isempty(stateTrajectoryData)
    warning('Warning, no data on trajectory topic');
    stateTrajectory.time = [];
    stateTrajectory.position = [];
    stateTrajectory.velocity = [];
    stateTrajectory.orientationRPY = [];
    stateTrajectory.orientationQuat = [];
    stateTrajectory.angularVelocity = [];
    stateTrajectory.jointPosition = [];
    stateTrajectory.jointVelocity = [];
    stateTrajectory.jointEffort = [];
    stateTrajectory.footPosition = [];
    stateTrajectory.footVelocity = [];
else
    
    stateTrajectory.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, stateTrajectoryData, 'UniformOutput', 0));
    
    stateTrajectory.position = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Position.X, m.Body.Pose.Position.Y, m.Body.Pose.Position.Z], stateTrajectoryData, 'UniformOutput', 0));
    stateTrajectory.velocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Linear.X, m.Body.Twist.Linear.Y, m.Body.Twist.Linear.Z], stateTrajectoryData, 'UniformOutput', 0));
    
    stateTrajectory.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.Body.Pose.Orientation.W, m.Body.Pose.Orientation.X, m.Body.Pose.Orientation.Y, m.Body.Pose.Orientation.Z])), stateTrajectoryData, 'UniformOutput', 0));
    stateTrajectory.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.Body.Pose.Orientation.W, m.Body.Pose.Orientation.X, m.Body.Pose.Orientation.Y, m.Body.Pose.Orientation.Z], stateTrajectoryData, 'UniformOutput', 0));
    stateTrajectory.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.Body.Twist.Angular.X, m.Body.Twist.Angular.Y, m.Body.Twist.Angular.Z], stateTrajectoryData, 'UniformOutput', 0));
    
    stateTrajectory.jointPosition = cell2mat(cellfun(@(m) m.Joints.Position.', stateTrajectoryData, 'UniformOutput', 0));
    stateTrajectory.jointVelocity = cell2mat(cellfun(@(m) m.Joints.Velocity.', stateTrajectoryData, 'UniformOutput', 0));
    stateTrajectory.jointEffort = cell2mat(cellfun(@(m) m.Joints.Effort.', stateTrajectoryData, 'UniformOutput', 0));
    
    num_feet = size(stateTrajectoryData{1}.Feet.Feet, 2);
    for i = 1:num_feet
        stateTrajectory.footPosition{i} = cell2mat(cellfun(@(m) ...
            [m.Feet.Feet(i).Position.X, m.Feet.Feet(i).Position.Y, m.Feet.Feet(i).Position.Z], stateTrajectoryData, 'UniformOutput', 0));
        stateTrajectory.footVelocity{i} = cell2mat(cellfun(@(m) ...
            [m.Feet.Feet(i).Velocity.X, m.Feet.Feet(i).Velocity.Y, m.Feet.Feet(i).Velocity.Z], stateTrajectoryData, 'UniformOutput', 0));
    end
    
    
end

% Read the trajectory data
controlGRFsData = readMessages(select(bag,'Topic','/control/grfs'),'DataFormat','struct');
controlGRFs = struct;
if isempty(controlGRFsData)
    warning('Warning, no data on grf control topic');
    controlGRFs.time = [];
    controlGRFs.vectors = [];
    controlGRFs.points = [];
    controlGRFs.contactStates = [];
else
    
    controlGRFs.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, controlGRFsData, 'UniformOutput', 0));
    num_feet = 4;
    for i = 1:num_feet
        try
            controlGRFs.vectors{i} = cell2mat(cellfun(@(m) ...
                [m.Vectors(i).X, m.Vectors(i).Y, m.Vectors(i).Z], controlGRFsData, 'UniformOutput', 0));
            controlGRFs.points{i} = cell2mat(cellfun(@(m) ...
                [m.Points(i).X, m.Points(i).Y, m.Points(i).Z], controlGRFsData, 'UniformOutput', 0));
            controlGRFs.contactStates{i} = cell2mat(cellfun(@(m) ...
                [m.ContactStates(i), m.ContactStates(i), m.ContactStates(i)], controlGRFsData, 'UniformOutput', 0));
        catch
            controlGRFs.vectors{i} = cell2mat(cellfun(@(m) ...
                [0,0,0], controlGRFsData, 'UniformOutput', 0));
            controlGRFs.points{i} = cell2mat(cellfun(@(m) ...
                [0,0,0], controlGRFsData, 'UniformOutput', 0));
            controlGRFs.contactStates{i} = cell2mat(cellfun(@(m) ...
                [0,0,0], controlGRFsData, 'UniformOutput', 0));
        end
    end
end

% Localize time to the first message
% startTime = min([stateGroundTruth.time(1), stateEstimate.time(1), stateTrajectory.time(1)]);
startTime = stateGroundTruth.time(1);
% stateEstimate.time = stateEstimate.time - startTime;
stateGroundTruth.time = stateGroundTruth.time - startTime;
stateTrajectory.time = stateTrajectory.time - startTime;
controlGRFs.time = controlGRFs.time - startTime;

% Pack data into a struct for namespace purPoses
data = struct;
data.stateEstimate = [];% stateEstimate;
data.stateGroundTruth = stateGroundTruth;
data.stateTrajectory = stateTrajectory;
data.controlGRFs = controlGRFs;

% If prompted, return the name of the filename
if (nargout>1)
    varargout{1} = trialName;
end