function processMultiLog(varargin)
% processLog Process a quad data log file to generate figures
%   processLog uses the default 'quad_log_current' file name to
%   yield a data structure containing select topic data. If this bag does
%   not exist, the user can select the bag via a UI.
%
%   processLog(FILENAME) uses the data in a bag with the specified
%   file name, looking in '../bags/'.

%% Prepare the environment
close all;clc;

% Check that this is the right current directory otherwise paths won't work
if ~endsWith(pwd, '/quad_logger/scripts')
    error('This script must be run from quad_logger/scripts/');
end

%% Select rosbag to parse

envName = 'gap_40cm_02realtime';    tWindow = [3, 6]; tLocalPlanWindow = [3, 6];
% envName = 'step_20cm_05realtime';     tWindow = [2, 5]; tLocalPlanWindow = [2, 4];
% envName = 'rough_25cm_02realtime';    tWindow = [3, 6]; tLocalPlanWindow = [3, 6];
configNames = {'Simple', 'Complex', 'Mixed', 'Adaptive'};
configLinestyles = {'--', ':', '-.', '-'};
configColors = {[0,130,186]/255, [166,25,46]/255, [242,169,0]/255,  [0,132,61]/255};
lineWidth = 3;

%% Set parameters

bSave = true;                       % Save the figures/videos
bAnimate = false;                   % Animate the trajectory (no translation)
bTitles = false;                     % Turn on figure titles
bPlotLocalPlanInfo = true;          % Turn on to plot local plan information
tWindowStates = [];                 % Specify time window for state (use [] for no clipping)
tWindowControl = [];                % Specify time window for control (use [] for no clipping)
tWindowLocalPlan = [];              % Specify time window for local plan (use [] for no clipping)

%% Load the data
linearStateFig = 1;
GRFVectorsFig = 2;
solveTimeFig = 3;
horizonLengthFig = 4;
predictionHorizonFig = 5;
simplePercentageFig = 6;
stateFigs = [];
controlFigs = [];
localPlanFigs = [];

maxTime = 0;

for i = 1:length(configNames)
    trialName = [envName, '_', lower(configNames{i})];
    
    % Load the data
    [data, trialName] = parseQuadBag(trialName);
    stateGroundTruth{i} = data.stateGroundTruth;
    stateTrajectory{i} = data.stateTrajectory;
    stateGRFs{i} = data.stateGRFs;
    controlGRFs{i} = data.controlGRFs;
    localPlan{i} = data.localPlan;
    maxTime = max(maxTime, max(localPlan{i}.time));
    
    %% Plot the data
    
%     tWindow = [stateGroundTruth{i}.time(1), stateGroundTruth{i}.time(end)];
%     tLocalPlanWindow = [localPlan{i}.time(1), localPlan{i}.time(end)];
        
    % Plot the state
    linearStateFig = figure(linearStateFig);
    linearStateFig.Name = "linear_states";
    traj_idx = find(stateGroundTruth{i}.time>= tWindow(1) & stateGroundTruth{i}.time<= tWindow(2));
    
    % z state
    subplot(2,2,1); hold on
    plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.position(traj_idx,3), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
    ylabel('Z Position (m)')
    axis tight
    
    subplot(2,2,3); hold on
    plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.velocity(traj_idx,3), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
    ylabel('Z Velocity (m/s)')
    xlabel('Time (s)')
    axis tight
    
    % x state
    subplot(2,2,2); hold on
    plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.position(traj_idx,1), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
    ylabel('X Position (m)')
    axis tight
    
    subplot(2,2,4); hold on
    plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.velocity(traj_idx,1), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
    ylabel('X Velocity (m/s)')
    xlabel('Time (s)')
    axis tight

%     % pitch
%     subplot(2,2,1); hold on
%     plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.orientationRPY(traj_idx,2), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
%     ylabel('Pitch (rad)')
%     axis tight
%     
%     subplot(2,2,3); hold on
%     plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.angularVelocity(traj_idx,2), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
%     ylabel('Pitch rate (rad/s)')
%     xlabel('Time (s)')
%     axis tight
%     
%     % yaw state
%     subplot(2,2,2); hold on
%     plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.orientationRPY(traj_idx,3), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
%     ylabel('Yaw (rad)')
%     axis tight
%     
%     subplot(2,2,4); hold on
%     plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.angularVelocity(traj_idx,3), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
%     ylabel('Yaw rate (rad/s)')
%     xlabel('Time (s)')
%     axis tight
%     set(linearStateFig, 'Position', [100 100 1200 1200])
    
    %
    % ~~~~~~~~~~~~~~~~~~~
    %
        
    % Plot the controls
    traj_idx = find(controlGRFs{i}.time>= tWindow(1) & controlGRFs{i}.time<= tWindow(2));

    totalGRF = zeros(length(controlGRFs{i}.vectors{1}), 3);
    for j = 1:length(controlGRFs{i}.vectors{1})
        for k = 1:length(controlGRFs{i}.vectors)
            totalGRF(j,:) = totalGRF(j,:) + controlGRFs{i}.vectors{k}(j,:);
        end
        if sum(totalGRF(j,:)) == 0
            totalGRF(j,:) = NaN;
        end
    end
    bw = 13.0*9.81;
    grfNorm = vecnorm(totalGRF - [0,0,bw],2,2);
    meanGRFNorm(i) = mean(grfNorm, 'omitnan');
    
    GRFVectorsFig = figure(GRFVectorsFig);
    GRFVectorsFig.Name = "grfs";
    
    hold on;
    plot(controlGRFs{i}.time(traj_idx), grfNorm(traj_idx), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
    ylabel('Control error norm $||u - u_{nom}||_2$ (N)')
    xlabel('Time (s)')
    axis tight
    set(GRFVectorsFig, 'Position', [100 100 1200 600])
        
    %
    % ~~~~~~~~~~~~~~~~~~~
    %
    
    % Plot the solve times
    solveTimeFig = figure(solveTimeFig);
    solveTimeFig.Name = "solve_time";
    scale = 1; % s to ms
    semilogy(localPlan{i}.time, smooth(scale*localPlan{i}.solveTime,5), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
    hold on;
    xlabel('Time (s)')
    ylabel('Solve Time (s)');
    axis tight
    set(solveTimeFig, 'Position', [100 100 1200 600])
    
    meanSolveTimes(i) = mean(localPlan{i}.solveTime);
    stdSolveTimes(i) = std(localPlan{i}.solveTime);
    medianSolveTimes(i) = median(localPlan{i}.solveTime);
    
    %
    % ~~~~~~~~~~~~~~~~~~~
    %
    
    % Plot the horizon lengths
    horizonLengthFig = figure(horizonLengthFig);
    horizonLengthFig.Name = "horizon_length";
    plot(localPlan{i}.time, localPlan{i}.horizonLength, 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
    hold on;
    xlabel('Time (s)')
    ylabel('Horizon Length');
    axis tight
    set(horizonLengthFig, 'Position', [100 100 1200 600])
    
    %
    % ~~~~~~~~~~~~~~~~~~~
    %
    
    simplePercentageVec = zeros(size(localPlan{i}.elementTimes));
    for j = 1:length(localPlan{i}.elementTimes)
        simplePercentageVec(j) = 100*sum(1-localPlan{i}.complexitySchedule{j}(1:end-1))/(double(localPlan{i}.horizonLength(j))-1);
    end
    
    % Plot the simple percentage over time
    simplePercentageFig = figure(simplePercentageFig);
    simplePercentageFig.Name = "simple_percentage";
    plot(localPlan{i}.time, simplePercentageVec, 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
    hold on;
    xlabel('Time (s)')
    ylabel('\% of Horizon Simplified');
%     ytickformat('percentage')
    yticks([0 25 50 75 100]);
    yticklabels({'0\%', '25\%', '50\%', '75\%', '100\%'});
    axis tight
    set(simplePercentageFig, 'Position', [100 100 1200 600])
    
    %
    % ~~~~~~~~~~~~~~~~~~~
    %
    
    
    % Plot the prediction horizons
    predictionHorizonFig = figure(predictionHorizonFig);
    predictionHorizonFig.Name = "prediction_horizon";
    subplot(2,2,i)
%     subplot(4,1,i)
    
    % Declare sizes
    markerSizeSimple = 10;
    markerSizeComplex = 10;
    
    % Extract times into vectors
    elementTimesVec = [];
    complexityScheduleVec = [];
    trajTimesVec = [];
    traj_idx = find(localPlan{i}.time>= tLocalPlanWindow(1) & localPlan{i}.time<= tLocalPlanWindow(2));
    
    localPlan{i}.time = localPlan{i}.time(traj_idx);
    localPlan{i}.solveTime = localPlan{i}.solveTime(traj_idx,:);
    localPlan{i}.elementTimes = localPlan{i}.elementTimes(traj_idx,:);
    localPlan{i}.cost = localPlan{i}.cost(traj_idx,:);
    localPlan{i}.iterations = localPlan{i}.iterations(traj_idx,:);
    localPlan{i}.horizonLength = localPlan{i}.horizonLength(traj_idx,:);
    localPlan{i}.complexitySchedule = localPlan{i}.complexitySchedule(traj_idx,:);

    for j = 1:length(localPlan{i}.elementTimes)
        horizonLength = localPlan{i}.horizonLength(j);
        trajTimesVec = [trajTimesVec; localPlan{i}.time(j)*ones(horizonLength,1)];
        elementTimesVec = [elementTimesVec; localPlan{i}.elementTimes{j}'];
        complexityScheduleVec = [complexityScheduleVec; localPlan{i}.complexitySchedule{j}'];
    end
    
    % Sort into simple and complex sets
    simpleIdx = find(complexityScheduleVec==0);
    complexIdx = find(complexityScheduleVec==1);
    trajTimesVecSimple = trajTimesVec(simpleIdx);
    elementTimesVecSimple = elementTimesVec(simpleIdx);
    trajTimesVecComplex = trajTimesVec(complexIdx);
    elementTimesVecComplex = elementTimesVec(complexIdx);
    
%     hold on;
%     if ~isempty(trajTimesVecComplex)
%         scatter(trajTimesVecComplex, elementTimesVecComplex, markerSizeComplex, configColors{2}, 'filled');
%     end
%     scatter(trajTimesVecSimple, elementTimesVecSimple, markerSizeSimple, configColors{1}, 'filled'); hold on;

%     if i >= 3
%         xlabel('Current Time, $k$ (s)');
%     end
%     if mod(i,2) == 1
%         ylabel('Predicted Time, $k + i$ (s)');
%     end
%     if (i == 2)
%         legend(predictionHorizonFig.CurrentAxes, 'Simple', 'Complex');
%     end
%     
%     axis equal
%     axis([tLocalPlanWindow(1), tLocalPlanWindow(2), tLocalPlanWindow(1), tLocalPlanWindow(2) + 0.72])
%     title(configNames{i})
%     set(predictionHorizonFig, 'Position', [100 100 1000 1200])
%     hold on;
    
    hold on;
    if ~isempty(trajTimesVecComplex)
        scatter(elementTimesVecComplex, trajTimesVecComplex, markerSizeComplex, configColors{2}, 'filled');
    end
    scatter(elementTimesVecSimple, trajTimesVecSimple, markerSizeSimple, configColors{1}, 'filled'); hold on;
    set(gca, 'YDir','reverse')
    
    if i >= 3
        xlabel('Predicted Time, $k + i$ (s)');
    end
    if mod(i,2) == 1
        ylabel('Current Time, $k$ (s)');
    end
    if (i == 2)
        legend(predictionHorizonFig.CurrentAxes, 'Simple', 'Complex');
    end
    
    axis equal
    axis([tLocalPlanWindow(1), tLocalPlanWindow(2) + 0.72, tLocalPlanWindow(1), tLocalPlanWindow(2)])
    title(configNames{i})
    set(predictionHorizonFig, 'Position', [100 100 1200 1000])
    hold on;
    
end
legend(linearStateFig.CurrentAxes, configNames);
legend(GRFVectorsFig.CurrentAxes, configNames);
legend(solveTimeFig.CurrentAxes, configNames);
legend(horizonLengthFig.CurrentAxes, configNames);
legend(simplePercentageFig.CurrentAxes, configNames, 'location', 'southeast');

1000*meanSolveTimes
medianSolveTimes
meanGRFNorm

% Add figures to array
figArray = [linearStateFig, GRFVectorsFig, solveTimeFig, horizonLengthFig, predictionHorizonFig, simplePercentageFig];

%% Save the logs and figures in one directory
logDir = [];
if bSave
    logDir = saveMultiLog(envName, configNames, figArray);
end
end
