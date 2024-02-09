function processLog(varargin)
% processLog Process a quad data log file to generate figures
%   processLog uses the default 'quad_log_current' file name to
%   yield a data structure containing select topic data. If this bag does
%   not exist, the user can select the bag via a UI.
%
%   processLog(FILENAME) uses the data in a bag with the specified
%   file name, looking in '../bags/'.

%% Prepare the environment
close all;clc;

% Check that this is the right current directory otherwise paths won't work
if ~endsWith(pwd, '/quad_logger/scripts')
    error('This script must be run from quad_logger/scripts/');
end

%% Select rosbag to parse

% If a trial name is provided, use that to save everything
if nargin>0
    trialName = varargin{1};
    trialName2 = varargin{2};
    %namespace = varargin{2};
    namespace = 'robot_1';
else
    trialName = 'Constant_Speed_BFE_Baseline'; % Set to '' to load via GUI
    trialName2 = 'Constant_Speed_BFE_8Kg'; % Second Bag to compare
    namespace = 'robot_1'; % Namespace of the robot bag, set to '' if none
end

%% Set parameters

bSave = false;                       % Save the figures/videos
bAnimate = false;                   % Animate the trajectory (no translation)
bTitles = true;                     % Turn on figure titles
bPlotLocalPlanInfo = true;          % Turn on to plot local plan information
tWindowStates = [];                 % Specify time window for state (use [] for no clipping)
tWindowStates2 = [];
tWindowControl = [];                % Specify time window for control (use [] for no clipping)
tWindowLocalPlan = [];              % Specify time window for local plan (use [] for no clipping)

%% Load the data

% Load the data
[data, trialName] = parseQuadBag(trialName, namespace);
[data2, trialName2] = parseQuadBag(trialName2, namespace);
stateEstimate = data.stateEstimate;
stateGroundTruth = data.stateGroundTruth;
stateGroundTruth2 = data2.stateGroundTruth;
stateTrajectory = data.stateTrajectory;
stateTrajectory2 = data2.stateTrajectory;
stateGRFs = data.stateGRFs;
stateGRFs2 = data2.stateGRFs;
controlGRFs = data.controlGRFs;
localPlan = data.localPlan;

%% Tracking Velocity & Position Error
% figure;
% subplot(3,1,1);
% plot(stateGroundTruth2.time, stateGroundTruth2.position(:,1),'-b', stateGroundTruth.time, stateGroundTruth.position(:,1),'-r');
% title('X Position Vs. Time');
% subplot(3,1,2);
% plot(stateGroundTruth2.time, stateGroundTruth2.position(:,2),'-b', stateGroundTruth.time, stateGroundTruth.position(:,2),'-r');
% title('Y Position Vs. Time');
% subplot(3,1,3);
% plot(stateGroundTruth2.time, stateGroundTruth2.position(:,3),'-b', stateGroundTruth.time, stateGroundTruth.position(:,3),'-r');
% title('Z Position Vs. Time');

%% Aligning data initialization

t1 = 0;
t2 = 0;
k = 1;

while(t1==0)

    if(stateGroundTruth.position(k,1)-stateGroundTruth.position(1,1) >= 0.1)
        t1 = k;
        break;
    end

    k=k+1;
end
k = 1;
while(t2==0)

    if(stateGroundTruth2.position(k,1)-stateGroundTruth2.position(1,1) >= 0.1)
        t2 = k;
        break;
    end
    k = k+1;

end

% Getting new Data
stateGroundTruth.time = stateGroundTruth.time - stateGroundTruth.time(t1);
stateGroundTruth2.time = stateGroundTruth2.time - stateGroundTruth2.time(t2);

stateGT_positions_1 = [stateGroundTruth.time(t1:end),stateGroundTruth.position(t1:end,:)];
stateGT_positions_2 = [stateGroundTruth2.time(t2:end),stateGroundTruth2.position(t2:end,:)];
stateGT_vel_1 = [stateGroundTruth.time(t1:end), stateGroundTruth.velocity(t1:end,:)];
stateGT_vel_2 = [stateGroundTruth2.time(t2:end), stateGroundTruth2.velocity(t2:end,:)];

%% Plotting states comparison between payload on and off

% figure('Name','Position State Data');
% subplot(3,1,1);
% plot(stateGT_positions_1(:,1), stateGT_positions_1(:,2),'-r', stateGT_positions_2(:,1), stateGT_positions_2(:,2),'-b');
% title('X Position Vs. Time');
% subplot(3,1,2);
% plot(stateGT_positions_1(:,1), stateGT_positions_1(:,3),'-r', stateGT_positions_2(:,1), stateGT_positions_2(:,3),'-b');
% title('Y Position Vs. Time');
% subplot(3,1,3);
% plot(stateGT_positions_1(:,1), stateGT_positions_1(:,4),'-r', stateGT_positions_2(:,1), stateGT_positions_2(:,4),'-b');
% title('Z Position Vs. Time');
% 
% 
% figure('Name','Velocity State Data');
% subplot(3,1,1);
% plot(stateGT_vel_1(:,1), stateGT_vel_1(:,2),'-r', stateGT_vel_2(:,1), stateGT_vel_2(:,2),'-b');
% title('X Position Vs. Time');
% subplot(3,1,2);
% plot(stateGT_vel_1(:,1), stateGT_vel_1(:,3),'-r', stateGT_vel_2(:,1), stateGT_vel_2(:,3),'-b');
% title('Y Position Vs. Time');
% subplot(3,1,3);
% plot(stateGT_vel_1(:,1), stateGT_vel_1(:,4),'-r', stateGT_vel_2(:,1), stateGT_vel_2(:,4),'-b');
% title('Z Position Vs. Time');

%% IMPLEMENTATION OF MAS UPDATE CALCULATION

%% Creating state error struct (e)

% creating empty structs for calculations
e = stateGroundTruth;
e.time = [];
e.position = [];
e.velocity = [];
e.orientationPRY = [];
e.orientationQuat = [];
e.angularVelocity = [];
e.jointPosition = [];
e.jointVelocity = [];
e.jointEffort = [];
e.footPosition{1} = [];
e.footPosition{2} = [];
e.footPosition{3} = [];
e.footPosition{4} = [];
e.footVelocity{1} = [];
e.footVelocity{2} = [];
e.footVelocity{3} = [];
e.footVelocity{4} = [];

% creating derivative of error
e_dot = e;

% creating second derrivative of error
e_ddot = e;







%% Plot the data

% Plot the state
stateFigs = [];
stateFigs2 = [];
if ~isempty(stateGroundTruth)
    [stateFigs] = plotState(stateGroundTruth, tWindowStates,'-', bTitles, stateFigs);
    [stateFigs2] = plotState(stateGroundTruth2, tWindowStates, '-', bTitles, stateFigs2);
end
if ~isempty(stateTrajectory)
     [stateFigs] = plotState(stateTrajectory,tWindowStates, ':', bTitles, stateFigs);
     [sttateFigs2] = plotState(stateTrajectory2, tWindowStates2, ':', bTitles, stateFigs2);
end
if ~isempty(stateEstimate)
    [stateFigs] = plotState(stateEstimate,tWindowStates, '--', bTitles, stateFigs);
end

% Plot the control
controlFigs = [];
if ~isempty(stateGRFs)
    controlFigs = plotControl(stateGRFs,tWindowControl,'-', bTitles, controlFigs);
end
if ~isempty(controlGRFs)
    controlFigs = plotControl(controlGRFs,tWindowControl,':', bTitles,controlFigs);
end

% Plot local plan information if desired
localPlanFigs = [];
if bPlotLocalPlanInfo && ~isempty(localPlan)
     localPlanFigs = plotLocalPlan(localPlan,tWindowLocalPlan,'-', bTitles,localPlanFigs);
end

% Add figures to array
figArray = [stateFigs, controlFigs, localPlanFigs];
%figArray = [stateFigs, stateFigs2, controlFigs];

%% Save the logs and figures in one directory
logDir = [];
if bSave
    logDir = saveLog(trialName, figArray);
end

%% Animate and save

if bAnimate
    robot_path = '../../quad_simulator/spirit_description/urdf/spirit.urdf';
    robot = importrobot(robot_path);
    videosDir = fullfile(logDir,'videos/');
    animateData(robot,stateGroundTruth, fullfile(videosDir, trialName), bSave);
end
