function [data, varargout] = parseQuadBag(varargin)
% parseQuadBag parse a quad data log file
%   DATA = parseQuadBag uses the default 'quad_log_current' file name to
%   yield a data structure containing select topic data. If this bag does
%   not exist, the user can select the bag via a UI.
%
%   DATA = parseQuadBag(FILENAME) uses the data in a bag with the specified
%   file name, looking in '../bags/'.

% Default empty namespace
namespace = '';

% Default to quad_log_current
if nargin >= 2
    if ~isempty(varargin{2})
        namespace = [varargin{2}, '/'];
    end
end

if nargin == 0
    trialName = 'quad_log_current';
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
stateEstimateData = readMessages(select(bag,'Topic',['/', namespace, 'state/estimate']),'DataFormat','struct');
stateEstimate = struct;
if isempty(stateEstimateData)
    warning('No data on state estimate topic');
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
stateGroundTruthData = readMessages(select(bag,'Topic',['/', namespace, 'state/ground_truth']),'DataFormat','struct');
stateGroundTruth = struct;
if isempty(stateGroundTruthData)
    warning('No data on ground truth state topic');
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
    
    num_feet = size(stateGroundTruthData{1}.Feet.Feet, 2);
    for i = 1:num_feet
        stateGroundTruth.footPosition{i} = cell2mat(cellfun(@(m) ...
            [m.Feet.Feet(i).Position.X, m.Feet.Feet(i).Position.Y, m.Feet.Feet(i).Position.Z], stateGroundTruthData, 'UniformOutput', 0));
        stateGroundTruth.footVelocity{i} = cell2mat(cellfun(@(m) ...
            [m.Feet.Feet(i).Velocity.X, m.Feet.Feet(i).Velocity.Y, m.Feet.Feet(i).Velocity.Z], stateGroundTruthData, 'UniformOutput', 0));
    end
end

% Read the trajectory data
stateTrajectoryData = readMessages(select(bag,'Topic',['/', namespace, 'state/trajectory']),'DataFormat','struct');
stateTrajectory = struct;
if isempty(stateTrajectoryData)
    warning('No data on trajectory topic');
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
    
    % Omit joint and foot data (not included in reference trajectory)
    stateTrajectory.jointPosition = nan(size(stateTrajectory.jointPosition));
    stateTrajectory.jointVelocity = nan(size(stateTrajectory.jointVelocity));
    stateTrajectory.jointEffort = nan(size(stateTrajectory.jointEffort));
    
    num_feet = size(stateTrajectoryData{1}.Feet.Feet, 2);
    for i = 1:num_feet
        stateTrajectory.footPosition{i} = cell2mat(cellfun(@(m) ...
            [m.Feet.Feet(i).Position.X, m.Feet.Feet(i).Position.Y, m.Feet.Feet(i).Position.Z], stateTrajectoryData, 'UniformOutput', 0));
        stateTrajectory.footVelocity{i} = cell2mat(cellfun(@(m) ...
            [m.Feet.Feet(i).Velocity.X, m.Feet.Feet(i).Velocity.Y, m.Feet.Feet(i).Velocity.Z], stateTrajectoryData, 'UniformOutput', 0));
        
        stateTrajectory.footPosition{i} = nan(size(stateTrajectory.footPosition{i}));
        stateTrajectory.footVelocity{i} = nan(size(stateTrajectory.footVelocity{i}));
    end
end

% Read the control GRFs data
controlGRFsData = readMessages(select(bag,'Topic',['/', namespace, 'control/grfs']),'DataFormat','struct');
controlGRFs = struct;
if isempty(controlGRFsData)
    warning('No data on grf control topic');
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

% Read the state GRFs data
stateGRFsData = readMessages(select(bag,'Topic',['/', namespace, 'state/grfs']),'DataFormat','struct');
stateGRFs = struct;
if isempty(stateGRFsData)
    warning('No data on grf state topic');
else
    
    stateGRFs.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, stateGRFsData, 'UniformOutput', 0));
    num_feet = 4;
    for i = 1:num_feet
        try
            stateGRFs.vectors{i} = cell2mat(cellfun(@(m) ...
                [m.Vectors(i).X, m.Vectors(i).Y, m.Vectors(i).Z], stateGRFsData, 'UniformOutput', 0));
            stateGRFs.points{i} = cell2mat(cellfun(@(m) ...
                [m.Points(i).X, m.Points(i).Y, m.Points(i).Z], stateGRFsData, 'UniformOutput', 0));
            stateGRFs.contactStates{i} = cell2mat(cellfun(@(m) ...
                [m.ContactStates(i), m.ContactStates(i), m.ContactStates(i)], stateGRFsData, 'UniformOutput', 0));
        catch
            stateGRFs.vectors{i} = cell2mat(cellfun(@(m) ...
                [0,0,0], stateGRFsData, 'UniformOutput', 0));
            stateGRFs.points{i} = cell2mat(cellfun(@(m) ...
                [0,0,0], stateGRFsData, 'UniformOutput', 0));
            stateGRFs.contactStates{i} = cell2mat(cellfun(@(m) ...
                [0,0,0], stateGRFsData, 'UniformOutput', 0));
        end
    end
end

% Read the local plan data
localPlanData = readMessages(select(bag,'Topic',['/', namespace, 'local_plan']),'DataFormat','struct');
localPlan = struct;
if isempty(localPlanData)
    warning('No data on local plan topic');
else
    localPlan.time = cell2mat(cellfun(@(m) double(m.StateTimestamp.Sec) + double(m.StateTimestamp.Nsec)*1E-9, localPlanData, 'UniformOutput', 0));
    localPlan.elementTimes = cellfun(@(m) double(m.Diagnostics.ElementTimes'), localPlanData, 'UniformOutput', 0);
    localPlan.solveTime = cell2mat(cellfun(@(m) m.Diagnostics.ComputeTime, localPlanData, 'UniformOutput', 0));
    localPlan.cost = cell2mat(cellfun(@(m) m.Diagnostics.Cost, localPlanData, 'UniformOutput', 0));
    localPlan.iterations = cell2mat(cellfun(@(m) m.Diagnostics.Iterations, localPlanData, 'UniformOutput', 0));
    localPlan.horizonLength = cell2mat(cellfun(@(m) m.Diagnostics.HorizonLength, localPlanData, 'UniformOutput', 0));
    localPlan.complexitySchedule = cellfun(@(m) double(m.Diagnostics.ComplexitySchedule'), localPlanData, 'UniformOutput', 0);
end

% Localize time to the first message
startTime = stateGroundTruth.time(1);
data = struct;

% Update time of existing messages and pack into struct
if ~isempty(fieldnames(stateEstimate))
    stateEstimate.time = stateEstimate.time - startTime;
    data.stateEstimate = stateEstimate;
else
    data.stateEstimate = [];
end

if ~isempty(fieldnames(stateGroundTruth))
    stateGroundTruth.time = stateGroundTruth.time - startTime;
    data.stateGroundTruth = stateGroundTruth;
else
    data.stateGroundTruth = [];
end

if ~isempty(fieldnames(stateTrajectory))
    stateTrajectory.time = stateTrajectory.time - startTime;
    data.stateTrajectory = stateTrajectory;
else
    data.stateTrajectory = [];
end

if ~isempty(fieldnames(controlGRFs))
    controlGRFs.time = controlGRFs.time - startTime;
    data.controlGRFs = controlGRFs;
else
    data.controlGRFs = [];
end

if ~isempty(fieldnames(stateGRFs))
    stateGRFs.time = stateGRFs.time - startTime;
    data.stateGRFs = stateGRFs;
else
    data.stateGRFs = [];
end

if ~isempty(fieldnames(localPlan))
    localPlan.time = localPlan.time - startTime;
    for i = 1:length(localPlan.elementTimes)
        shiftedElementTimes = localPlan.elementTimes{i} + localPlan.time(i);
        localPlan.elementTimes{i} = shiftedElementTimes;
    end
    data.localPlan = localPlan;
else
    data.localPlan = [];
end

% If prompted, return the name of the filename
if (nargout>1)
    varargout{1} = trialName;
end