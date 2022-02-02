function logDir = saveLog(trialName,figArray)

% If they don't already exist, make the log directories for this trial
logDir = fullfile('../logs/',trialName);
if ~exist(logDir, 'dir')
    mkdir(logDir);
end
figuresDir = fullfile(logDir,'figures/');
if ~exist(figuresDir, 'dir')
    mkdir(figuresDir);
end
videosDir = fullfile(logDir,'videos/');
if ~exist(videosDir, 'dir')
    mkdir(videosDir);
end

COMTrajFig = figArray(1);
linearStateFig = figArray(2);
angularStateFig = figArray(3);
jointPositionFig = figArray(4);
jointVelocityFig = figArray(5);
jointEffortFig = figArray(6);
footPositionFig = figArray(7);
footVelocityFig = figArray(8);
grfsFig = figArray(9);

% Only save if requested
% Copy the bag
copyfile(['../bags/', trialName, '.bag'],logDir)

% Save the figures
saveas(COMTrajFig, [figuresDir, trialName, '_com_trajectory'], 'fig');
saveas(COMTrajFig, [figuresDir, trialName, '_com_trajectory'], 'pdf');
saveas(COMTrajFig, [figuresDir, trialName, '_com_trajectory'], 'png');
saveas(linearStateFig, [figuresDir, trialName, '_linear_states'], 'fig');
saveas(linearStateFig, [figuresDir, trialName, '_linear_states'], 'pdf');
saveas(linearStateFig, [figuresDir, trialName, '_linear_states'], 'png');
saveas(angularStateFig, [figuresDir, trialName, '_angular_states'], 'fig');
saveas(angularStateFig, [figuresDir, trialName, '_angular_states'], 'pdf');
saveas(angularStateFig, [figuresDir, trialName, '_angular_states'], 'png');
saveas(jointPositionFig, [figuresDir, trialName, '_joint_positions'], 'fig');
saveas(jointPositionFig, [figuresDir, trialName, '_joint_positions'], 'pdf');
saveas(jointPositionFig, [figuresDir, trialName, '_joint_positions'], 'png');
saveas(jointVelocityFig, [figuresDir, trialName, '_joint_velocities'], 'fig');
saveas(jointVelocityFig, [figuresDir, trialName, '_joint_velocities'], 'pdf');
saveas(jointVelocityFig, [figuresDir, trialName, '_joint_velocities'], 'png');
saveas(jointEffortFig, [figuresDir, trialName, '_joint_efforts'], 'fig');
saveas(jointEffortFig, [figuresDir, trialName, '_joint_efforts'], 'pdf');
saveas(jointEffortFig, [figuresDir, trialName, '_joint_efforts'], 'png');
saveas(footPositionFig, [figuresDir, trialName, '_foot_positions'], 'fig');
saveas(footPositionFig, [figuresDir, trialName, '_foot_positions'], 'pdf');
saveas(footPositionFig, [figuresDir, trialName, '_foot_positions'], 'png');
saveas(footVelocityFig, [figuresDir, trialName, '_foot_velocities'], 'fig');
saveas(footVelocityFig, [figuresDir, trialName, '_foot_velocities'], 'pdf');
saveas(footVelocityFig, [figuresDir, trialName, '_foot_velocities'], 'png');
saveas(grfsFig, [figuresDir, trialName, '_grfs'], 'fig');
saveas(grfsFig, [figuresDir, trialName, '_grfs'], 'pdf');
saveas(grfsFig, [figuresDir, trialName, '_grfs'], 'png');