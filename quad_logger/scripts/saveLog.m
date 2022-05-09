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

% Copy the bag
copyfile(['../bags/', trialName, '.bag'],logDir)

% Specifiy the name of each figure
figNameArray = {'_com_trajectory', '_linear_states', '_angular_states', ...
    '_joint_positions', '_joint_velocities', '_joint_efforts', ...
    '_foot_positions', '_foot_velocities', '_grfs'};

assert(length(figNameArray) == length(figArray));

% Save the figures
for i = 1:length(figArray)
    saveas(figArray(i), [figuresDir, trialName, figNameArray{i}], 'fig');
    exportgraphics(figArray(i), [figuresDir, trialName, figNameArray{i}, '.png']);
    exportgraphics(figArray(i), [figuresDir, trialName, figNameArray{i}, '.pdf']);
end