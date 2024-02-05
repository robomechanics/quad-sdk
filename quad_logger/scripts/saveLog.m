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

% Copy the bag
copyfile(['../bags/', trialName, '.bag'],logDir)

% Save the figures
for i = 1:length(figArray)
    figFullFile = [figuresDir, trialName, '_', figArray(i).Name];
    saveas(figArray(i), [figFullFile, '.fig']);
    exportgraphics(figArray(i), [figFullFile, '.png']);
    exportgraphics(figArray(i), [figFullFile, '.pdf']);
end