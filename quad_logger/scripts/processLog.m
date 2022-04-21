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
if ~endsWith(pwd, 'quad-software/quad_logger/scripts')
    error('This script must be run from quad-software/quad_logger/scripts/');
end

%% Select rosbag to parse

% If a trial name is provided, use that to save everything
if nargin>0
    trialName = varargin{1};
else
    trialName = ''; % Set to '' to load via GUI
end

%% Set parameters

bSave = true;                       % Save the figures/videos
bAnimate = false;                   % Animate the trajectory (no translation)
bTitles = true;                     % Turn on figure titles
tWindowStates = [];                 % Specify time window for state (use [] for no clipping)
tWindowControl = [];                % Specify time window for control (use [] for no clipping)

%% Load the data

% Load the data
[data, trialName] = parseQuadBag(trialName);
stateEstimate = data.stateEstimate;
stateGroundTruth = data.stateGroundTruth;
stateTrajectory = data.stateTrajectory;
stateGRFs = data.stateGRFs;
controlGRFs = data.controlGRFs;

%% Plot the data

% Plot the state
if ~isempty(stateGroundTruth)
    [figArray] = plotState(stateGroundTruth,tWindowStates,'-', bTitles);
end
if ~isempty(stateTrajectory)
    [figArray] = plotState(stateTrajectory,tWindowStates, ':', bTitles, figArray);
end
if ~isempty(stateEstimate)
    [figArray] = plotState(stateEstimate,tWindowStates, '--', bTitles, figArray);
end

% Plot the control
if ~isempty(stateGRFs)
    GRFVectorsFig = plotControl(stateGRFs,tWindowControl,'-', bTitles);
end
if ~isempty(controlGRFs)
    GRFVectorsFig = plotControl(controlGRFs,tWindowControl,':', bTitles,GRFVectorsFig);
end

% Add figures to array
figArray = [figArray, GRFVectorsFig];

%% Save the logs and figures in one directory
logDir = [];
if bSave
    logDir = saveLog(trialName, figArray);
end

%% Animate and save

if bAnimate
    robot = importrobot('../../quad_simulator/spirit_description/urdf/spirit.urdf');
    videosDir = fullfile(logDir,'videos/');
    animateData(robot,stateGroundTruth, fullfile(videosDir, trialName), bSave);
end
