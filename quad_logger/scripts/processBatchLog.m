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
bSampleNum = 24;
bTypeNum = 1;

maxError = [];
sucList = [];
bagList = {};
stateEstimate = {};
stateGroundTruth = {};
controlGRFs = {};
contactSensing = {};

% Loop all bags
for j = 1:size(bagNameList, 1)/bSampleNum/bTypeNum

    for i = 1:bSampleNum*bTypeNum

        % Load bag files
        trialName = strcat(bagPath, bagNameList(i + (j - 1)*bSampleNum*bTypeNum).name);

        % Import URDF
        %         spirit40 = importrobot('../../quad_simulator/spirit_description/urdf/spirit.urdf');
        %figure
        % homeConfig = homeConfiguration(spirit40);
        % show(spirit40,homeConfig);

        % Load the data
        [data, bagList{end + 1}] = parseQuadBag(trialName);
        data.stateGroundTruth.axisAngleRP = rotm2axang(eul2rotm([zeros(size(data.stateGroundTruth.orientationRPY, 1), 1), fliplr(data.stateGroundTruth.orientationRPY(:, 1:2))]));

        [row, col] = find(data.contactSensing.contactStates == 3, 1, 'first');
        data.contactSensing.missTime = data.contactSensing.time(row);
        data.stateGroundTruth.syncTime = data.stateGroundTruth.time - data.contactSensing.missTime;

        stateEstimate{end + 1} = data.stateEstimate;
        stateGroundTruth{end + 1} = data.stateGroundTruth;
        controlGRFs{end + 1} = data.controlGRFs;
        contactSensing{end + 1} = data.contactSensing;

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
        maxError(end+1) = max(abs(wrapToPi(data.stateGroundTruth.axisAngleRP(:, 4))));

        if maxError((j - 1)*bSampleNum*bTypeNum + i) < pi/3
            sucList((j - 1)*bSampleNum*bTypeNum + i) = true;
        else
            sucList((j - 1)*bSampleNum*bTypeNum + i) = false;
        end

    end

    %     fprintf('Tail success: %d, Leg success: %d, total: %d \n', ...
    %         sum(sucList((j - 1)*bSampleNum*bTypeNum + 1:(j - 1)*bSampleNum*bTypeNum + bSampleNum)), ...
    %         sum(sucList((j - 1)*bSampleNum*bTypeNum + bSampleNum + 1:j*bSampleNum*bTypeNum)), ...
    %         bSampleNum);

    for k=1:bTypeNum
        figure()
        ylim([0, pi/3])
        hold on
        for i=1:bSampleNum
            if sucList((j - 1)*bSampleNum*bTypeNum + (k-1)*bSampleNum + i)
                h = plot(stateGroundTruth{(j - 1)*bSampleNum*bTypeNum + (k-1)*bSampleNum + i}.syncTime, ...
                    abs(wrapToPi(stateGroundTruth{(j - 1)*bSampleNum*bTypeNum + (k-1)*bSampleNum + i}.axisAngleRP(:, 4))), ...
                    ':k');
                h.Color(4) = 0.5;
            end
        end
        hold off
    end
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
