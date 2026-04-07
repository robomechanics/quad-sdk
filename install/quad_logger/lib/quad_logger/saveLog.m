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
% videosDir = fullfile(logDir,'videos/');
% if ~exist(videosDir, 'dir')
%     mkdir(videosDir);
% end

% Copy the bag
% copyfile(['../bags/', trialName, '.mcap'],logDir)

% Save the figures
for i = 1:length(figArray)
    figFullFile = [figuresDir, trialName, '_', figArray(i).Name];
    saveas(figArray(i), [figFullFile, '.fig']);
    exportgraphics(figArray(i), [figFullFile, '.png']);
    exportgraphics(figArray(i), [figFullFile, '.pdf']);
end