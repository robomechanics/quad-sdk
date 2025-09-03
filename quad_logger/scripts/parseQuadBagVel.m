function [data] = parseQuadBagVel(filePath, namespace)
% parseQuadBag parse a quad data log file
%   DATA = parseQuadBag uses the default 'quad_log_current' file name to
%   yield a data structure containing select topic data. If this bag does
%   not exist, the user can select the bag via a UI.
%
%   DATA = parseQuadBag(FILENAME) uses the data in a bag with the specified
%   file name, looking in '../bags/'.

% Default empty namespace
if nargin < 2, namespace = 'robot_1/'; else, namespace = [namespace, '/']; end

% Load the bag
disp(filePath);
bag = ros2bagreader(filePath);

% Read the state estimate data
% stateEstimateData = readMessages(bag,'Topic',['/', namespace, 'state/estimate'],'DataFormat','struct');
% stateEstimate = struct;
stateEstimateSelection = select(bag, 'Topic',['/', namespace, 'state/estimate']);
stateEstimateData = readMessages(stateEstimateSelection);
stateEstimate = struct;

if isempty(stateEstimateData)
    warning('No data on state estimate topic');
else
    stateEstimate.time = cell2mat(cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1E-9, stateEstimateData, 'UniformOutput', 0));
    
    stateEstimate.position = cell2mat(cellfun(@(m) ...
        [m.body.pose.position.x, m.body.pose.position.y, m.body.pose.position.z], stateEstimateData, 'UniformOutput', 0));
    stateEstimate.velocity = cell2mat(cellfun(@(m) ...
        [m.body.twist.linear.x, m.body.twist.linear.y, m.body.twist.linear.z], stateEstimateData, 'UniformOutput', 0));
    
    stateEstimate.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z])), stateEstimateData, 'UniformOutput', 0));
    stateEstimate.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z], stateEstimateData, 'UniformOutput', 0));
    stateEstimate.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.body.twist.angular.x, m.body.twist.angular.y, m.body.twist.angular.z], stateEstimateData, 'UniformOutput', 0));
    
    stateEstimate.jointPosition = cell2mat(cellfun(@(m) m.joints.position.', stateEstimateData, 'UniformOutput', 0));
    stateEstimate.jointVelocity = cell2mat(cellfun(@(m) m.joints.velocity.', stateEstimateData, 'UniformOutput', 0));
    stateEstimate.jointEffort = cell2mat(cellfun(@(m) m.joints.effort.', stateEstimateData, 'UniformOutput', 0));
end

% Read the ground truth data
% stateGroundTruthData = readMessages(select(bag,'Topic',['/', namespace, 'state/ground_truth']),'DataFormat','struct');
% stateGroundTruth = struct;
stateGroundTruthData = readMessages(select(bag,'Topic',['/', namespace, 'state/ground_truth']));
stateGroundTruth = struct;
if isempty(stateGroundTruthData)
    warning('No data on ground truth state topic');
else
    
    stateGroundTruth.time = cell2mat(cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1E-9, stateGroundTruthData, 'UniformOutput', 0));
    % disp("First 10 message times (s):")
    % disp(stateGroundTruth.time(1:min(10,end)))
    stateGroundTruth.position = cell2mat(cellfun(@(m) ...
        [m.body.pose.position.x, m.body.pose.position.y, m.body.pose.position.z], stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.velocity = cell2mat(cellfun(@(m) ...
        [m.body.twist.linear.x, m.body.twist.linear.y, m.body.twist.linear.z], stateGroundTruthData, 'UniformOutput', 0));
    
    stateGroundTruth.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z])), stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z], stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.body.twist.angular.x, m.body.twist.angular.y, m.body.twist.angular.z], stateGroundTruthData, 'UniformOutput', 0));
    
    stateGroundTruth.jointPosition = cell2mat(cellfun(@(m) m.joints.position.', stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.jointVelocity = cell2mat(cellfun(@(m) m.joints.velocity.', stateGroundTruthData, 'UniformOutput', 0));
    stateGroundTruth.jointEffort = cell2mat(cellfun(@(m) m.joints.effort.', stateGroundTruthData, 'UniformOutput', 0));
    
    num_feet = size(stateGroundTruthData{1}.feet.feet, 1);
    for i = 1:num_feet
        stateGroundTruth.footPosition{i} = cell2mat(cellfun(@(m) ...
            [m.feet.feet(i).position.x, m.feet.feet(i).position.y, m.feet.feet(i).position.z], stateGroundTruthData, 'UniformOutput', 0));
        stateGroundTruth.footVelocity{i} = cell2mat(cellfun(@(m) ...
            [m.feet.feet(i).velocity.x, m.feet.feet(i).velocity.y, m.feet.feet(i).velocity.z], stateGroundTruthData, 'UniformOutput', 0));
    end
end

% Read the Ground Truth Body Frame Data
stateGroundTruthBodyFrameData = readMessages(select(bag,'Topic',['/', namespace, 'state/ground_truth_body_frame']));
stateGroundTruthBodyFrame = struct;
if isempty(stateGroundTruthBodyFrameData)
    warning('No data on ground truth state topic');
else
    
    stateGroundTruthBodyFrame.time = cell2mat(cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1E-9, stateGroundTruthBodyFrameData, 'UniformOutput', 0));
    % disp("First 10 message times (s):")
    % disp(stateGroundTruth.time(1:min(10,end)))
    stateGroundTruthBodyFrame.position = cell2mat(cellfun(@(m) ...
        [m.body.pose.position.x, m.body.pose.position.y, m.body.pose.position.z], stateGroundTruthBodyFrameData, 'UniformOutput', 0));
    stateGroundTruthBodyFrame.velocity = cell2mat(cellfun(@(m) ...
        [m.body.twist.linear.x, m.body.twist.linear.y, m.body.twist.linear.z], stateGroundTruthBodyFrameData, 'UniformOutput', 0));
    
    stateGroundTruthBodyFrame.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z])), stateGroundTruthBodyFrameData, 'UniformOutput', 0));
    stateGroundTruthBodyFrame.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z], stateGroundTruthBodyFrameData, 'UniformOutput', 0));
    stateGroundTruthBodyFrame.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.body.twist.angular.x, m.body.twist.angular.y, m.body.twist.angular.z], stateGroundTruthBodyFrameData, 'UniformOutput', 0));
    
    stateGroundTruthBodyFrame.jointPosition = cell2mat(cellfun(@(m) m.joints.position.', stateGroundTruthBodyFrameData, 'UniformOutput', 0));
    stateGroundTruthBodyFrame.jointVelocity = cell2mat(cellfun(@(m) m.joints.velocity.', stateGroundTruthBodyFrameData, 'UniformOutput', 0));
    stateGroundTruthBodyFrame.jointEffort = cell2mat(cellfun(@(m) m.joints.effort.', stateGroundTruthBodyFrameData, 'UniformOutput', 0));
    
    num_feet = size(stateGroundTruthBodyFrameData{1}.feet.feet, 1);
    for i = 1:num_feet
        stateGroundTruthBodyFrame.footPosition{i} = cell2mat(cellfun(@(m) ...
            [m.feet.feet(i).position.x, m.feet.feet(i).position.y, m.feet.feet(i).position.z], stateGroundTruthBodyFrameData, 'UniformOutput', 0));
        stateGroundTruthBodyFrame.footVelocity{i} = cell2mat(cellfun(@(m) ...
            [m.feet.feet(i).velocity.x, m.feet.feet(i).velocity.y, m.feet.feet(i).velocity.z], stateGroundTruthBodyFrameData, 'UniformOutput', 0));
    end
end

% Read the trajectory data
stateTrajectoryData = readMessages(select(bag,'Topic',['/', namespace, 'state/trajectory']));
stateTrajectory = struct;
if isempty(stateTrajectoryData)
    warning('No data on trajectory topic');
else
    
    stateTrajectory.time = cell2mat(cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1E-9, stateTrajectoryData, 'UniformOutput', 0));
    
    stateTrajectory.position = cell2mat(cellfun(@(m) ...
        [m.body.pose.position.x, m.body.pose.position.y, m.body.pose.position.z], stateTrajectoryData, 'UniformOutput', 0));
    stateTrajectory.velocity = cell2mat(cellfun(@(m) ...
        [m.body.twist.linear.x, m.body.twist.linear.y, m.body.twist.linear.z], stateTrajectoryData, 'UniformOutput', 0));
    
    stateTrajectory.orientationRPY = cell2mat(cellfun(@(m) ...
        fliplr(quat2eul([m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z])), stateTrajectoryData, 'UniformOutput', 0));
    stateTrajectory.orientationQuat = cell2mat(cellfun(@(m) ...
        [m.body.pose.orientation.w, m.body.pose.orientation.x, m.body.pose.orientation.y, m.body.pose.orientation.z], stateTrajectoryData, 'UniformOutput', 0));
    stateTrajectory.angularVelocity = cell2mat(cellfun(@(m) ...
        [m.body.twist.angular.x, m.body.twist.angular.y, m.body.twist.angular.z], stateTrajectoryData, 'UniformOutput', 0));
    
    stateTrajectory.jointPosition = cell2mat(cellfun(@(m) m.joints.position.', stateTrajectoryData, 'UniformOutput', 0));
    stateTrajectory.jointVelocity = cell2mat(cellfun(@(m) m.joints.velocity.', stateTrajectoryData, 'UniformOutput', 0));
    stateTrajectory.jointEffort = cell2mat(cellfun(@(m) m.joints.effort.', stateTrajectoryData, 'UniformOutput', 0));
    
    % Omit joint and foot data (not included in reference trajectory)
    stateTrajectory.jointPosition = nan(size(stateTrajectory.jointPosition));
    stateTrajectory.jointVelocity = nan(size(stateTrajectory.jointVelocity));
    stateTrajectory.jointEffort = nan(size(stateTrajectory.jointEffort));
    
    num_feet = size(stateTrajectoryData{1}.feet.feet, 1);
    for i = 1:num_feet
        stateTrajectory.footPosition{i} = cell2mat(cellfun(@(m) ...
            [m.feet.feet(i).position.x, m.feet.feet(i).position.y, m.feet.feet(i).position.z], stateTrajectoryData, 'UniformOutput', 0));
        stateTrajectory.footVelocity{i} = cell2mat(cellfun(@(m) ...
            [m.feet.feet(i).velocity.x, m.feet.feet(i).velocity.y, m.feet.feet(i).velocity.z], stateTrajectoryData, 'UniformOutput', 0));
        
        stateTrajectory.footPosition{i} = nan(size(stateTrajectory.footPosition{i}));
        stateTrajectory.footVelocity{i} = nan(size(stateTrajectory.footVelocity{i}));
    end
end
% Read the control GRFs data
% controlGRFsData = readMessages(select(bag,'Topic',['/', namespace, 'control/grfs']),'DataFormat','struct');
% controlGRFs = struct;
controlGRFsData = readMessages(select(bag,'Topic',['/', namespace, 'control/grfs']));
controlGRFs = struct;
if isempty(controlGRFsData)
    warning('No data on grf control topic');
else
    
    controlGRFs.time = cell2mat(cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1E-9, controlGRFsData, 'UniformOutput', 0));
    num_feet = 4;
    for i = 1:num_feet
        try
            controlGRFs.vectors{i} = cell2mat(cellfun(@(m) ...
                [m.vectors(i).x, m.vectors(i).y, m.vectors(i).z], controlGRFsData, 'UniformOutput', 0));
            controlGRFs.points{i} = cell2mat(cellfun(@(m) ...
                [m.points(i).x, m.points(i).y, m.points(i).z], controlGRFsData, 'UniformOutput', 0));
            controlGRFs.contactStates{i} = cell2mat(cellfun(@(m) ...
                [m.contact_states(i), m.contact_states(i), m.contact_states(i)], controlGRFsData, 'UniformOutput', 0));
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
% stateGRFsData = readMessages(select(bag,'Topic',['/', namespace, 'state/grfs']),'DataFormat','struct');
% stateGRFs = struct;
stateGRFsData = readMessages(select(bag,'Topic',['/', namespace, 'state/grfs']));
stateGRFs = struct;
if isempty(stateGRFsData)
    warning('No data on grf state topic');
else
    stateGRFs.time = cell2mat(cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1E-9, stateGRFsData, 'UniformOutput', 0));
    num_feet = 4;
    for i = 1:num_feet
        try
            stateGRFs.vectors{i} = cell2mat(cellfun(@(m) ...
                [m.vectors(i).x, m.vectors(i).y, m.vectors(i).z], stateGRFsData, 'UniformOutput', 0));
            stateGRFs.points{i} = cell2mat(cellfun(@(m) ...
                [m.points(i).x, m.points(i).y, m.points(i).z], stateGRFsData, 'UniformOutput', 0));
            stateGRFs.contactStates{i} = cell2mat(cellfun(@(m) ...
                [m.contact_states(i), m.contact_states(i), m.contact_states(i)], stateGRFsData, 'UniformOutput', 0));
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

% Read the cmd vel data
cmdVelData = readMessages(select(bag, 'Topic',['/', namespace, 'cmd_vel_stamped']));
cmdVels = struct;
if isempty(cmdVelData)
    warning('No data on cmd vel topic');
else
    cmdVels.time = cell2mat(cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1E-9, cmdVelData, 'UniformOutput', 0));
    cmdVels.velocity = cell2mat(cellfun(@(m) ...
        [m.twist.linear.x, m.twist.linear.y, m.twist.linear.z], cmdVelData, 'UniformOutput', 0));
    cmdVels.angularVelocity= cell2mat(cellfun(@(m) ...
        [m.twist.angular.x, m.twist.angular.y, m.twist.angular.z], cmdVelData, 'UniformOutput', 0));
end


% Read the local plan data
% localPlanData = readMessages(select(bag,'Topic',['/', namespace, 'local_plan']),'DataFormat','struct');
% localPlan = struct;
% localPlanData = readMessages(select(bag,'Topic',['/', namespace, 'local_plan']));
% localPlan = struct;
% if isempty(localPlanData)
%     warning('No data on local plan topic');
% else
%     localPlan.time = cell2mat(cellfun(@(m) double(m.state_timestamp.sec) + double(m.state_timestamp.nanosec)*1E-9, localPlanData, 'UniformOutput', 0));
%     localPlan.elementTimes = cellfun(@(m) double(m.diagnostics.element_times'), localPlanData, 'UniformOutput', 0);
%     localPlan.solveTime = cell2mat(cellfun(@(m) m.diagnostics.compute_time, localPlanData, 'UniformOutput', 0));
%     localPlan.cost = cell2mat(cellfun(@(m) m.diagnostics.cost, localPlanData, 'UniformOutput', 0));
%     localPlan.iterations = cell2mat(cellfun(@(m) m.diagnostics.iterations, localPlanData, 'UniformOutput', 0));
%     localPlan.horizonLength = cell2mat(cellfun(@(m) m.diagnostics.horizon_length, localPlanData, 'UniformOutput', 0));
%     localPlan.complexitySchedule = cellfun(@(m) double(m.diagnostics.complexity_schedule'), localPlanData, 'UniformOutput', 0);
% end

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

if ~isempty(fieldnames(stateGroundTruthBodyFrame))
    stateGroundTruthBodyFrame.time = stateGroundTruthBodyFrame.time - startTime;
    data.stateGroundTruthBodyFrame = stateGroundTruthBodyFrame;
else
    data.stateGroundTruthBodyFrame = [];
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

if ~isempty(fieldnames(cmdVels))
    cmdVels.time = cmdVels.time - startTime;
    data.cmdVels = cmdVels;
else
    data.stateGRFs = [];
end

% if ~isempty(fieldnames(localPlan))
%     localPlan.time = localPlan.time - startTime;
%     for i = 1:length(localPlan.elementTimes)
%         shiftedElementTimes = localPlan.elementTimes{i} + localPlan.time(i);
%         localPlan.elementTimes{i} = shiftedElementTimes;
%     end
%     data.localPlan = localPlan;
% else
%     data.localPlan = [];
% end

