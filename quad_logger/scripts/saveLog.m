function logDir = saveLog(trialName,figArray)
% saveLog Save the bag data and figures into a generated log file
%   saveLog(trialName, figArray) generates a new directory named trialName
%   located in ../logs/ if one does not already exist, copies the bag to
%   this log directory, and saves .fig, .png, and .pdf copies of each of
%   the figures included in figArray into trialName/figures.

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

<<<<<<< HEAD
=======
COMTrajFig = figArray(1);
linearStateFig = figArray(2);
angularStateFig = figArray(3);
jointPositionFig = figArray(4);
jointVelocityFig = figArray(5);
jointEffortFig = figArray(6);
footPositionFig = figArray(7);
footVelocityFig = figArray(8);

% Only save if requested
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
% Copy the bag
copyfile(['../bags/', trialName, '.bag'],logDir)

% Save the figures
<<<<<<< HEAD
for i = 1:length(figArray)
    figFullFile = [figuresDir, trialName, '_', figArray(i).Name];
    saveas(figArray(i), [figFullFile, '.fig']);
    exportgraphics(figArray(i), [figFullFile, '.png']);
    exportgraphics(figArray(i), [figFullFile, '.pdf']);
end
=======
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
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
