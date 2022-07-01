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
    trialName = ''; % Set to '' to load via GUI
    namespace = 'robot_1'; % Namespace of the robot bag, set to '' if none
end

%% Set parameters

bSave = true;                       % Save the figures/videos
bAnimate = false;                   % Animate the trajectory (no translation)
bTitles = true;                     % Turn on figure titles
bPlotLocalPlanInfo = true;          % Turn on to plot local plan information
tWindowStates = [];                 % Specify time window for state (use [] for no clipping)
tWindowControl = [];                % Specify time window for control (use [] for no clipping)
tWindowLocalPlan = [];              % Specify time window for local plan (use [] for no clipping)

%% Load the data

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
    animateData(robot,stateGroundTruth, fullfile(videosDir, trialName), bSave);
end
