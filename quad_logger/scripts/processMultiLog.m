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

envName = 'leap_gap_40cm_01realtime';
configNames = {'Simple', 'Complex', 'Mixed', 'Adaptive'};
configLinestyles = {'-', '--', ':', '-.'};
configColors = {[0,45,114]/255, [166,25,46]/255, [242,169,0]/255,  [0,132,61]/255};

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
    
    % Plot the state
    linearStateFig = figure(linearStateFig);
    linearStateFig.Name = "linear_states";
    
    subplot(2,2,1); hold on
    plot(stateGroundTruth{i}.time, stateGroundTruth{i}.position(:,3), 'Color', configColors{i});
    ylabel('Z Position (m)')
    axis tight
    
    subplot(2,2,3); hold on
    plot(stateGroundTruth{i}.time, stateGroundTruth{i}.velocity(:,3), 'Color', configColors{i});
    ylabel('Z Velocity (m/s)')
    xlabel('Time (s)')
    axis tight
    
    subplot(2,2,2); hold on
    plot(stateGroundTruth{i}.time, stateGroundTruth{i}.orientationRPY(:,3), 'Color', configColors{i});
    ylabel('Yaw (rad)')
    axis tight
    
    subplot(2,2,4); hold on
    plot(stateGroundTruth{i}.time, stateGroundTruth{i}.angularVelocity(:,3), 'Color', configColors{i});
    ylabel('Yaw rate (rad/s)')
    xlabel('Time (s)')
    axis tight
    set(linearStateFig, 'Position', [100 100 1200 1200])
    
    %     align_Ylabels(linearStateFig);
    
    % Plot the controls
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
    
    GRFVectorsFig = figure(GRFVectorsFig);
    GRFVectorsFig.Name = "grfs";
    
    hold on;
    plot(controlGRFs{i}.time, grfNorm, 'Color', configColors{i});
    ylabel('Control error norm $||u - u_{nom}||_2$ (N)')
    xlabel('Time (s)')
    axis tight
    set(GRFVectorsFig, 'Position', [100 100 1200 600])
    
    align_Ylabels(GRFVectorsFig);
    
    % Plot the solve times
    solveTimeFig = figure(solveTimeFig);
    solveTimeFig.Name = "solve_time";
    scale = 1; % s to ms
    semilogy(localPlan{i}.time, scale*localPlan{i}.solveTime, 'Color', configColors{i});
    hold on;
    xlabel('Time (s)')
    ylabel('Solve Time (s)');
    axis tight
    set(solveTimeFig, 'Position', [100 100 1200 600])
    
    % Plot the horizon lengths
    horizonLengthFig = figure(horizonLengthFig);
    horizonLengthFig.Name = "horizon_length";
    plot(localPlan{i}.time, localPlan{i}.horizonLength, 'Color', configColors{i});
    hold on;
    xlabel('Time (s)')
    ylabel('Horizon Length');
    axis tight
    set(horizonLengthFig, 'Position', [100 100 1200 600])
    
    % Plot the prediction horizons
    predictionHorizonFig = figure(predictionHorizonFig);
    predictionHorizonFig.Name = "prediction_horizon";
    subplot(2,2,i)
    
    % Declare sizes
    markerSizeSimple = 3;
    markerSizeComplex = 3;
    localPlanColorVector = {[166,25,46]/255, [0,45,114]/255, [0,132,61]/255, [242,169,0]/255};
    
    % Extract times into vectors
    elementTimesVec = [];
    complexityScheduleVec = [];
    trajTimesVec = [];
    tWindow = [0, 5];
    traj_idx = find(localPlan{i}.time>= tWindow(1) & localPlan{i}.time<= tWindow(2));
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
    
    scatter(trajTimesVecSimple, elementTimesVecSimple, markerSizeSimple, localPlanColorVector{2}, 'filled'); hold on;
    
    if ~isempty(trajTimesVecComplex)
        scatter(trajTimesVecComplex, elementTimesVecComplex, markerSizeComplex, localPlanColorVector{1});
    end
    
    if i >= 3
        xlabel('Current Time, $k$ (s)');
    end
    if mod(i,2) == 1
        ylabel('Predicted Time, $k + i$ (s)');
    end
    if (i == 2)
        legend(predictionHorizonFig.CurrentAxes, 'Simple', 'Complex');
    end
    
    axis equal
    axis([tWindow(1), tWindow(2), tWindow(1), tWindow(2) + 0.72])
    set(predictionHorizonFig, 'Position', [100 100 1000 1200])
    hold on;
    
    
end
legend(linearStateFig.CurrentAxes, configNames);
legend(GRFVectorsFig.CurrentAxes, configNames);
legend(solveTimeFig.CurrentAxes, configNames);
legend(horizonLengthFig.CurrentAxes, configNames);

% Add figures to array
figArray = [linearStateFig, GRFVectorsFig, solveTimeFig, horizonLengthFig, predictionHorizonFig];

%% Save the logs and figures in one directory
logDir = [];
if bSave
    logDir = saveMultiLog(envName, configNames, figArray);
end
