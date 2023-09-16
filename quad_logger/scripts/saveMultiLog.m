function logDir = saveMultiLog(bagDir,envName,nameSuffix,configNames,figArray)
% saveLog Save the bag data and figures into a generated log file
%   saveLog(trialName, figArray) generates a new directory named trialName
%   located in ../logs/ if one does not already exist, copies the bag to
%   this log directory, and saves .fig, .png, and .pdf copies of each of
%   the figures included in figArray into trialName/figures.

% If they don't already exist, make the log directories for this trial
logDir = fullfile('../logs/',envName);
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

% Save the figures
for i = 1:length(figArray)
    figFullFile = [figuresDir, envName, '_', figArray(i).Name];
    saveas(figArray(i), [figFullFile, '.fig']);
    exportgraphics(figArray(i), [figFullFile, '.png']);
    exportgraphics(figArray(i), [figFullFile, '.pdf']);
end

% Copy the bag
for i = 1:length(configNames)
    % Search bags for the config
    filename = [envName, '_', nameSuffix, '_', lower(configNames{i}), '*'];
    bagNameList = dir([bagDir, filename]);

    for l = 1:length(bagNameList)
        % Loop through each bag`
        trialName = bagNameList(l).name(1:end-4);
        copyfile([bagDir, trialName, '.bag'],logDir)
    end
end