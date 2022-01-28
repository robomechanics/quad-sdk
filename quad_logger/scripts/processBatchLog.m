% Process a bagfile and save the resulting images
close all;clc;

% Check that this is the right current directory otherwise paths won't work
if ~endsWith(pwd, 'quad-software/quad_logger/scripts')
    error('This script must be run from quad-software/quad_logger/scripts/');
end

bagPath = '../bags/archive/';
bagNameList = dir(strcat(bagPath, '*.bag'));
bAnimate = false;
bSave = false;

maxError = [];
bagList = {};

% Loop all bags
for i = 1:size(bagNameList, 1)
        
    % Load bag files
    trialName = strcat(bagPath, bagNameList(i).name);

    % Import URDF
    spirit40 = importrobot('../../quad_simulator/spirit_description/urdf/spirit.urdf');
    %figure
    % homeConfig = homeConfiguration(spirit40);
    % show(spirit40,homeConfig);

    % Load the data
    [data, bagList{i}] = parseQuadBag(trialName);
    data.stateGroundTruth.axisAngleRP = rotm2axang(eul2rotm([zeros(size(data.stateGroundTruth.orientationRPY, 1), 1), fliplr(data.stateGroundTruth.orientationRPY(:, 1:2))]));
    stateEstimate{i} = data.stateEstimate;
    stateGroundTruth{i} = data.stateGroundTruth;
    stateTrajectory{i} = data.stateTrajectory;

    % Plot the state
    % [figArray] = plotState(stateGroundTruth,'-');
    % GRFVectorsFig = plotControl(data.controlGRFs,'-');
    % figArray = [figArray, GRFVectorsFig];

    % Save the data if desired
    logDir = [];
    if bSave
        logDir = saveLog(trialName, figArray);
    end

    % Animate and save if desired
    if bAnimate
        videosDir = fullfile(logDir,'videos/');
        animateData(spirit40,stateGroundTruth, fullfile(videosDir, trialName), bSave);
    end

    % Analyse the orientation error
    if isempty(data.stateGroundTruth.tailJointPosition)
        maxError(end+1, 1) = max(abs(wrapToPi(data.stateGroundTruth.axisAngleRP(:, 4))));
    else
        maxError(end+1, 2) = max(abs(wrapToPi(data.stateGroundTruth.axisAngleRP(:, 4))));
    end
    
end

mean(maxError(maxError(:, 1)<2, 1))
mean(maxError(maxError(:, 2)<2, 2))
sum(maxError(:, 1)>2, 1)
sum(maxError(:, 2)>2, 1)