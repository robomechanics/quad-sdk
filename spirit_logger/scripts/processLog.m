% Open a bag files
close all;clear;clc;

% Check that this is the right current directory otherwise paths won't work
if ~endsWith(pwd, 'spirit-software/spirit_logger/scripts')
    error('This script must be run from spirit-software/spirit_logger/scripts/');
end

% Specify the trial name and settings
trialName = 'spirit_framework_test_2020-11-23';
bAnimate = true;
bSave = true;

% Import URDF
figure
spirit40 = importrobot('../../spirit_simulator/spirit_description/urdf/spirit.urdf');
homeConfig = homeConfiguration(spirit40);
show(spirit40,homeConfig);

% Load the data
[data, trialName] = parseSpiritBag(trialName);
stateEstimate = data.stateEstimate;
stateGroundTruth = data.stateGroundTruth;

% Plot the state
[COMTrajFig, linearStateFig, angularStateFig, jointPositionFig, jointVelocityFig, jointEffortFig] = plotState(stateGroundTruth);
% plotState(stateEstimate);

% Compute and plot the toe forces - in progress
% toeForces = getToeForces(spirit40, stateEstimate);
% plotToeForces(toeForces);

% Save the data if desired
logDir = [];
if bSave
    logDir = saveLog(bSave, trialName, COMTrajFig, linearStateFig, angularStateFig, jointPositionFig, jointVelocityFig, jointEffortFig);
end

% Animate and save if desired
if bAnimate
    videosDir = fullfile(logDir,'videos/');
    animateData(spirit40,stateGroundTruth, fullfile(videosDir, trialName), bSave);
end