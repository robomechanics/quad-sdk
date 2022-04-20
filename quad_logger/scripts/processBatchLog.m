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
bSampleNum = 100;
bTypeNum = 2;

bagNameList = struct2table(bagNameList); % convert the struct array to a table
bagNameList = sortrows(bagNameList, 'date'); % sort the table by 'DOB'
bagNameList = table2struct(bagNameList); % change it back to struct array if necessary

maxError = zeros(size(bagNameList));
sucList = zeros(size(bagNameList));
stateEstimate = cell(size(bagNameList));
stateGroundTruth = cell(size(bagNameList));
controlGRFs = cell(size(bagNameList));
contactSensing = cell(size(bagNameList));

% parpool(4);

% Loop all bags
parfor i = 1:size(bagNameList, 1)

    % Load bag files
    trialName = strcat(bagPath, bagNameList(i).name);

    % Import URDF
    %         spirit40 = importrobot('../../quad_simulator/spirit_description/urdf/spirit.urdf');
    %figure
    % homeConfig = homeConfiguration(spirit40);
    % show(spirit40,homeConfig);

    % Load the data
    data = parseQuadBag(trialName);
    data.stateGroundTruth.axisAngleRP = rotm2axang(eul2rotm([zeros(size(data.stateGroundTruth.orientationRPY, 1), 1), fliplr(data.stateGroundTruth.orientationRPY(:, 1:2))]));

    data.contactSensing.missTime = find(sum(data.contactSensing.contactStates, 2), 1, 'first');
    data.contactSensing.missTime = data.contactSensing.time(data.contactSensing.missTime);
    data.stateGroundTruth.syncTime = data.stateGroundTruth.time - data.contactSensing.missTime;

    stateEstimate{i} = data.stateEstimate;
    stateGroundTruth{i} = data.stateGroundTruth;
    controlGRFs{i} = data.controlGRFs;
    contactSensing{i} = data.contactSensing;

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
        %         animateData(spirit40,stateGroundTruth, fullfile(videosDir, trialName), bSave);
    end

    % Analyse the orientation error
    maxError(i) = max(abs(wrapToPi(data.stateGroundTruth.axisAngleRP(:, 4))));

    if maxError(i) < pi/3
        sucList(i) = true;
    else
        sucList(i) = false;
    end

%         fprintf('Success: %d, total: %d \n', ...
%             sum(sucList((i - 1)*bSampleNum + 1:(i - 1)*bSampleNum + bSampleNum)), ...
%             bSampleNum);

    %     for k=1:bTypeNum
    %         figure()
    %         ylim([0, pi/3])
    %         hold on
    %         for i=1:bSampleNum
    %             if sucList((j - 1)*bSampleNum*bTypeNum + (k-1)*bSampleNum + i)
    %                 h = plot(stateGroundTruth{(j - 1)*bSampleNum*bTypeNum + (k-1)*bSampleNum + i}.syncTime, ...
    %                     abs(wrapToPi(stateGroundTruth{(j - 1)*bSampleNum*bTypeNum + (k-1)*bSampleNum + i}.axisAngleRP(:, 4))), ...
    %                     ':k');
    %                 h.Color(4) = 0.5;
    %             end
    %         end
    %         hold off
    %     end
end

%{
% Save all plots
FolderName = '/home/rml/catkin_ws/src/quad-software/quad_logger/scripts/fig';   % Your destination folder
FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
for iFig = 1:length(FigList)
FigHandle = FigList(iFig);
FigName   = num2str(get(FigHandle, 'Number'));
saveas(FigHandle, fullfile(FolderName, [FigName '.jpg']));
end
%}
