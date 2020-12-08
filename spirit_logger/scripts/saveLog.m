function logDir = saveLog(trialName,COMTrajFig, linearStateFig, angularStateFig, jointPositionFig, jointVelocityFig, jointEffortFig)

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
