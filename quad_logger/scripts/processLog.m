function processLog(varargin)
% Process a bagfile and save the resulting images
close all;clc;

% If a trial name is provided, use that to save everything
if nargin>0
    % Leave these as is so the default is to animate and save
    trialName = varargin{1};
    bAnimate = true;
    bSave = true;
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
[data, trialName] = parseQuadBag(trialName);
stateEstimate = data.stateEstimate;
stateGroundTruth = data.stateGroundTruth;
stateTrajectory = data.stateTrajectory;

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

% Animate and save if desired
if bAnimate
    videosDir = fullfile(logDir,'videos/');
    animateData(robot,stateGroundTruth, fullfile(videosDir, trialName), bSave);
end
