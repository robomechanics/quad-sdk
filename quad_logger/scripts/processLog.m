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
    % Specify the trial name and settings
    trialName = '';
    bAnimate = false;
    bSave = true;
end

% Check that this is the right current directory otherwise paths won't work
if ~endsWith(pwd, 'spirit-software/quad_logger/scripts')
    error('This script must be run from quad-software/quad_logger/scripts/');
end

% Import URDF
quad40 = importrobot('../../quad_simulator/spirit_description/urdf/spirit.urdf');
%figure
% homeConfig = homeConfiguration(quad40);
% show(quad40,homeConfig);

% Load the data
[data, trialName] = parseQuadBag(trialName);
stateEstimate = data.stateEstimate;
stateGroundTruth = data.stateGroundTruth;
stateTrajectory = data.stateTrajectory;

% Plot the state
[figArray] = plotState(stateGroundTruth,'-');
plotControl(data.controlGRFs,'-');
% [figArray] = plotState(stateTrajectory, ':', figArray);
% plotState(stateEstimate);

% Compute and plot the toe forces - in progress
% toeForces = getToeForces(quad40, stateEstimate);
% plotToeForces(toeForces);

% Save the data if desired
logDir = [];
if bSave
    logDir = saveLog(trialName, figArray);
end

% Animate and save if desired
if bAnimate
    videosDir = fullfile(logDir,'videos/');
    animateData(quad40,stateTrajectory, fullfile(videosDir, trialName), bSave);
end