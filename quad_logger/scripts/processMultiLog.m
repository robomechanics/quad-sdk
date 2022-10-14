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

% envName = 'gap_40cm';    tWindow = [3, 6]; tLocalPlanWindow = [3, 6];
% bStateBasedWindow = false;
% envName = 'step_20cm';     tWindow = [3, 6]; tLocalPlanWindow = [2.5, 5];
% bStateBasedWindow = false;
envName = 'flat';    tWindow = [0, 6]; tLocalPlanWindow = [0, 5]; bStateBasedWindow = true;
configNames = {'Simple', 'Complex', 'Mixed', 'Adaptive'};
configLinestyles = {'--', ':', '-.', '-'};
configColors = {[0,130,186]/255, [166,25,46]/255, [242,169,0]/255,  [0,132,61]/255};
lineWidth = 3;

% Bag index used for plot, 1-based indexing
plotIndex = [1, 1, 1, 1];

% Planned goal position for success check

% Distance tolerance for success check
success_tol = 0.5;
start_diff_tol = 0.5;
if contains(envName, 'step')
    name_prefix = '05realtime';
    duration = 16.0;
    goal_position = [5.0, 0.0];

elseif contains(envName, 'gap')
    name_prefix = '02realtime';
    duration = 35.0;
    goal_position = [5.0, 0.0];
elseif contains(envName, 'flat')
    name_prefix = '10realtime';
    duration = 10.0;
    goal_position = [3.0, 0.0];
end

bProcessAllBags = true;

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
dt = 0.030;

dataMat = {};

for i = 1:length(configNames)
    % Search bags for the config
    bagNameList = dir(['../bags/', envName, '_', name_prefix, '_', lower(configNames{i}), '*']);
    
    % Create storage list
    solveTimesList = cell(size(bagNameList));
    grfNormList = cell(size(bagNameList));
    successList = cell(size(bagNameList));
    maxVelList = cell(size(bagNameList));
    settlingTimeList = cell(size(bagNameList));
    fracSimplifiedList = cell(size(bagNameList));
    
    % Set flag for plotting
    bPlotting = true;
    
    for l = 1:length(bagNameList)
        % Loop through each bag`
        trialName = bagNameList(l).name(1:end-4);
        
        if ~bPlotting && ~bProcessAllBags
            continue;
        end
        
        % Load the data
        [data, trialName] = parseQuadBag(trialName);
        dataMat{i,l} = data;
        stateGroundTruth{i} = data.stateGroundTruth;
        if norm(stateGroundTruth{i}.position(1,1:2) - stateGroundTruth{i}.position(end,1:2)) < start_diff_tol
            fprintf("No data for %s\n", bagNameList(l).name);
            successList{l} = false;
            continue;
        end
        stateTrajectory{i} = data.stateTrajectory;
        stateGRFs{i} = data.stateGRFs;
        controlGRFs{i} = data.controlGRFs;
        localPlan{i} = data.localPlan;
        maxTime = max(maxTime, max(localPlan{i}.time));
        
        % Record success states
        successList{l} = norm(stateGroundTruth{i}.position(end,1:2) - goal_position) < success_tol;
        
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
        
        % Record grf norm
        grfNormList{l} = grfNorm;

        % Record max vel
        maxVelList{l} = max(stateGroundTruth{i}.velocity(:,1));

        % Record solving time
        solveTimesList{l} = localPlan{i}.solveTime;
        highSolveTimesList{l} = sum(localPlan{i}.solveTime(localPlan{i}.solveTime > dt))/duration;

        % Identify the start and end indices
        start_idx = find((vecnorm(stateGroundTruth{i}.velocity(:,1),2,2) > 1e-2),1);
        end_idx = find((vecnorm(stateGroundTruth{i}.position(:,1:2) - goal_position, 2, 2) < success_tol) & ...
            (vecnorm(stateGroundTruth{i}.velocity(:,1),2,2) < 1e-2),1);
        if (isempty(end_idx))
            end_idx = size(stateGroundTruth{i}.time,1);
        end

        settlingTimeList{l} = stateGroundTruth{i}.time(end_idx) - stateGroundTruth{i}.time(start_idx);

        % Only plot specified index
        if bPlotting && (successList{l} || (l == length(bagNameList) && sum(successList{l}) == 0))
            %% Plot the data
            
            % Only plot once per config
            bPlotting = false;
            
            %     tWindow = [stateGroundTruth{i}.time(1), stateGroundTruth{i}.time(end)];
            %     tLocalPlanWindow = [localPlan{i}.time(1), localPlan{i}.time(end)];
            
            % Plot the state
            linearStateFig = figure(linearStateFig);
            linearStateFig.Name = "linear_states";

            traj_idx = find(stateGroundTruth{i}.time>= tWindow(1) & stateGroundTruth{i}.time<= tWindow(2));

            if (bStateBasedWindow)
                stateGroundTruth{i}.time = stateGroundTruth{i}.time - stateGroundTruth{i}.time(start_idx);
                controlGRFs{i}.time = controlGRFs{i}.time - controlGRFs{i}.time(start_idx);
                localPlan{i}.time = localPlan{i}.time - localPlan{i}.time(start_idx);
                stateGroundTruth{i}.position(:,1) = stateGroundTruth{i}.position(:,1)  - stateGroundTruth{i}.position(start_idx,1) ;

                traj_idx = traj_idx(traj_idx >= start_idx & traj_idx <= end_idx);
            end

%             traj_idx = find(stateGroundTruth{i}.time>= tWindow(1) & stateGroundTruth{i}.time<= tWindow(2));
            
            if contains(envName, 'step')
                % pitch
                subplot(1,2,1); hold on
                plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.orientationRPY(traj_idx,2), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
                ylabel('Pitch (rad)')
                xlabel('Time (s)')
                annotation('arrow',[0.15 0.19],[0.6 0.74])
                axis tight
                
                % yaw
                subplot(1,2,2); hold on
                plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.orientationRPY(traj_idx,3), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
                ylabel('Yaw (rad)')
                xlabel('Time (s)')
                axis tight
                set(linearStateFig, 'Position', [100 100 1200 600])
                annotation('arrow',[0.58 0.63],[0.81 0.67])
            elseif contains(envName, 'gap')
                % z state
                subplot(1,2,1); hold on
                plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.velocity(traj_idx,1), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
                ylabel('X Velocity (m/s)')
                xlabel('Time (s)')
                annotation('arrow',[0.16 0.24],[0.86 0.84])
                axis tight
                
                subplot(1,2,2); hold on
                plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.position(traj_idx,3), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
                ylabel('Z Position (m)')
                xlabel('Time (s)')
                annotation('arrow',[0.62 0.67],[0.56 0.7])
                axis tight
                set(linearStateFig, 'Position', [100 100 1200 600])
            elseif contains(envName, 'flat')
                % z state
                subplot(1,2,1); hold on
                plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.position(traj_idx,1), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
                ylabel('X Position (m)')
                xlabel('Time (s)')
                axis tight
                
                subplot(1,2,2);
                hold on;
                plot(stateGroundTruth{i}.time(traj_idx), stateGroundTruth{i}.velocity(traj_idx,1), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
                ylabel('X Velocity (m/s)')
                xlabel('Time (s)')
                axis tight
                xlim([0 inf])
                ylim([0 inf])
                set(linearStateFig, 'Position', [100 100 1200 600])
            end
            
            %
            % ~~~~~~~~~~~~~~~~~~~
            %
            
            % Plot the controls
            traj_idx = find(controlGRFs{i}.time>= tWindow(1) & controlGRFs{i}.time<= tWindow(2));
            
            GRFVectorsFig = figure(GRFVectorsFig);
            GRFVectorsFig.Name = "grfs";
            
            hold on;
            plot(controlGRFs{i}.time(traj_idx), grfNorm(traj_idx), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
            ylabel('Control error norm (N)')
            xlabel('Time (s)')
            axis tight
            set(GRFVectorsFig, 'Position', [100 100 1200 600])
            box off
            %
            % ~~~~~~~~~~~~~~~~~~~
            %
            
            % Plot the solve times
            solveTimeFig = figure(solveTimeFig);
            solveTimeFig.Name = "solve_time";
            scale = 1; % s to ms
            semilogy(localPlan{i}.time, scale*localPlan{i}.solveTime, 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
            hold on;
            xlabel('Time (s)')
            ylabel('Solve Time (s)');
            axis tight
            set(solveTimeFig, 'Position', [100 100 1200 600])
            box off
            
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
            box off
            
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
            axis([0, max(localPlan{i}.time), -5 105])
            set(simplePercentageFig, 'Position', [100 100 1200 830])
            box off
            
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
            
            numSimpleElements = 0;
            numTotalElements = 0;
            for j = 1:length(localPlan{i}.elementTimes)
                horizonLength = localPlan{i}.horizonLength(j);
                trajTimesVec = [trajTimesVec; localPlan{i}.time(j)*ones(horizonLength,1)];
                elementTimesVec = [elementTimesVec; localPlan{i}.elementTimes{j}'];
                complexityScheduleVec = [complexityScheduleVec; localPlan{i}.complexitySchedule{j}'];

                numSimpleElements = numSimpleElements + sum(1 - localPlan{i}.complexitySchedule{j}(1:end-1));
                numTotalElements = numTotalElements + double(horizonLength) - 1;
            end
            
            % Sort into simple and complex sets
            simpleIdx = find(complexityScheduleVec==0);
            complexIdx = find(complexityScheduleVec==1);
            trajTimesVecSimple = trajTimesVec(simpleIdx);
            elementTimesVecSimple = elementTimesVec(simpleIdx);
            trajTimesVecComplex = trajTimesVec(complexIdx);
            elementTimesVecComplex = elementTimesVec(complexIdx);

            fracSimplifiedList{l} = numSimpleElements/numTotalElements;
            
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
            scatter(elementTimesVecSimple, trajTimesVecSimple, markerSizeSimple, configColors{1}, 'filled'); hold on;
            if ~isempty(trajTimesVecComplex)
                scatter(elementTimesVecComplex, trajTimesVecComplex, markerSizeComplex, configColors{2}, 'filled');
            end
            set(gca, 'YDir','reverse')
            
            if i >= 3
                xlabel('Horizon Time, $k + i$ (s)');
            end
            if mod(i,2) == 1
                ylabel('Wall Time, $k$ (s)');
            end
            if (i == 2)
                legend(predictionHorizonFig.CurrentAxes, 'Simple', 'Complex');
            end
            
            axis equal
            axis([tLocalPlanWindow(1), tLocalPlanWindow(2) + 0.72, tLocalPlanWindow(1), tLocalPlanWindow(2)])
            title(configNames{i})
            set(predictionHorizonFig, 'Position', [100 100 1200 830])
            hold on;
        end
    end
    
    % Compute statistics for the success runs
    if bProcessAllBags
        meanGRFNorm(i) = mean(cell2mat(grfNormList(cell2mat(successList))), 'omitnan')
        maxVel(i) = mean(cell2mat(maxVelList(cell2mat(successList))), 'omitnan')
        meanSettlingTime(i) = mean(cell2mat(settlingTimeList(cell2mat(successList))), 'omitnan')
        meanSolveTimes(i) = mean(cell2mat(solveTimesList(cell2mat(successList))))
        meanFracSimplified(i) = mean(cell2mat(fracSimplifiedList(cell2mat(successList))))
        highSolveTimes(i) = mean(cell2mat(highSolveTimesList(cell2mat(successList))))
        stdSolveTimes(i) = std(cell2mat(solveTimesList(cell2mat(successList))))
        medianSolveTimes(i) = median(cell2mat(solveTimesList(cell2mat(successList))))
        successCount(i) = sum(cell2mat(successList))
    end
end
if contains(envName, 'flat')
    legend(linearStateFig.CurrentAxes, configNames, 'location', 'northeast');
else
    legend(linearStateFig.CurrentAxes, configNames, 'location', 'southwest');
end
legend(GRFVectorsFig.CurrentAxes, configNames);
legend(solveTimeFig.CurrentAxes, configNames);
legend(horizonLengthFig.CurrentAxes, configNames);
legend(simplePercentageFig.CurrentAxes, configNames, 'location', 'east');

if bProcessAllBags
    meanSolveTimesMs = 1000*meanSolveTimes;
    highSolveTimesPerc = 100*highSolveTimes;
    meanPercSimplified = 100*meanFracSimplified;

    successCount
    meanSolveTimesMs
    highSolveTimesPerc
    meanGRFNorm
    maxVel
    meanSettlingTime
    meanPercSimplified
end

save('~/dataMat.m', 'dataMat')

% Add figures to array
figArray = [linearStateFig, GRFVectorsFig, solveTimeFig, horizonLengthFig, predictionHorizonFig, simplePercentageFig];

%% Save the logs and figures in one directory
logDir = [];
if bSave
    logDir = saveMultiLog(envName, name_prefix, configNames, figArray);
end
end