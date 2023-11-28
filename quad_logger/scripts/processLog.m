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
    namespace = varargin{2};
else
<<<<<<< HEAD
    trialName = ''; % Set to '' to load via GUI
    namespace = 'robot_1'; % Namespace of the robot bag, set to '' if none
=======
    % Specify the trial name and settings
    trialName = 'quad_log_current';
    bAnimate = true;
    bSave = true;
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
end

%% Set parameters

<<<<<<< HEAD
bSave = true;                       % Save the figures/videos
bAnimate = false;                   % Animate the trajectory (no translation)
bTitles = true;                     % Turn on figure titles
bPlotLocalPlanInfo = true;          % Turn on to plot local plan information
tWindowStates = [];                 % Specify time window for state (use [] for no clipping)
tWindowControl = [];                % Specify time window for control (use [] for no clipping)
tWindowLocalPlan = [];              % Specify time window for local plan (use [] for no clipping)

%% Load the data
=======
% Import URDF
quad40 = importrobot('../../quad_simulator/spirit_description/urdf/spirit.urdf');
%figure
% homeConfig = homeConfiguration(quad40);
% show(quad40,homeConfig);
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

% Load the data
[data, trialName] = parseQuadBag(trialName, namespace);
stateEstimate = data.stateEstimate;
stateGroundTruth = data.stateGroundTruth;
stateTrajectory = data.stateTrajectory;
stateGRFs = data.stateGRFs;
controlGRFs = data.controlGRFs;
localPlan = data.localPlan;

%% Plot the data

% Plot the state
<<<<<<< HEAD
stateFigs = [];
if ~isempty(stateGroundTruth)
    [stateFigs] = plotState(stateGroundTruth,tWindowStates,'-', bTitles, stateFigs);
end
if ~isempty(stateTrajectory)
    [stateFigs] = plotState(stateTrajectory,tWindowStates, ':', bTitles, stateFigs);
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
=======
[figArray] = plotState(stateGroundTruth,'-');
[figArray] = plotState(stateTrajectory, ':', figArray);
% plotState(stateEstimate);

% Compute and plot the toe forces - in progress
% toeForces = getToeForces(quad40, stateEstimate);
% plotToeForces(toeForces);
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

% Plot local plan information if desired
localPlanFigs = [];
if bPlotLocalPlanInfo && ~isempty(localPlan)
    localPlanFigs = plotLocalPlan(localPlan,tWindowLocalPlan,'-', bTitles,localPlanFigs);
end

% Add figures to array
figArray = [stateFigs, controlFigs, localPlanFigs];

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
<<<<<<< HEAD
    animateData(robot,stateGroundTruth, fullfile(videosDir, trialName), bSave);
end
=======
    animateData(quad40,stateTrajectory, fullfile(videosDir, trialName), bSave);
end
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
