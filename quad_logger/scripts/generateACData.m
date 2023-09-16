function generateACData(varargin)
% generateACData Generate data and figures for Adaptive Complexity Model Predictive Control
% Assumes the following file structure:
%   - scripts/
%       - generateACData.m
%       - parseQuadBag.m
%       - saveMultiLog.m
%       - cmuColor.m
%   - bags/
%       - <misc bags located here>
%       - flat_final/
%           - flat_10realtime_simple_0.bag
%           - ...
%       - step_20cm_final/
%           - step_20cm_05realtime_simple_0.bag
%           - ...
%       - gap_40cm_final/
%           - gap_40cm_02realtime_simple_0.bag
%           - ...
%   - logs/

%% Prepare the environment
close all;clc;

% Check that this is the right current directory otherwise paths won't work
if ~endsWith(pwd, '/scripts')
    error('This script must be run from scripts/');
end

%% Select experiment to parse

envName = 'flat';
% envName = 'flat_offline';
% envName = 'step_20cm';
% envName = 'gap_40cm';

%% Set options

bLoadFinal = true;                  % Load the final data (if false loads whatever is in /bags/)
bSave = false;                       % Save the figures
bComputeStats = true;               % Compute statistical tests for significance (chi-squared)
bProcessAllBags = true;             % Process all bags (needed for data collection, not plotting)

% Only allow saving if loading the final bags
if ~bLoadFinal
    bSave = false;
end

%% Set params

% Plotting params
configNames = {'Simple', 'Complex', 'Mixed', 'Adaptive'};
configLinestyles = {'--', ':', '-.', '-'};
configColors = {[0,130,186]/255, [166,25,46]/255, [242,169,0]/255,  [0,132,61]/255};
lineWidth = 3;

% Set some general params
success_tol = 0.5;
start_diff_tol = 0.5;
maxTime = 0;
dt = 0.030;

% Configuration dependent params
if strcmp(envName, 'flat')
    tWindow = [0, 6];
    tLocalPlanWindow = [0, 5];
    bStateBasedWindow = true;
    bZeroOrigin = true;
    nameSuffix = '10realtime';
    duration = 10.0;
    goal_position = [3.0, 0.0];
elseif strcmp(envName, 'flat_offline')
    tWindow = [0, 6];
    tLocalPlanWindow = [0, 5];
    bStateBasedWindow = true;
    bZeroOrigin = true;
    nameSuffix = '005realtime';
    duration = 10.0 * 20;
    goal_position = [3.0, 0.0];
elseif contains(envName, 'step')
    tWindow = [3, 6];
    tLocalPlanWindow = [2.5, 5];
    bStateBasedWindow = false;
    bZeroOrigin = false;
    nameSuffix = '05realtime';
    duration = 16.0;
    goal_position = [5.0, 0.0];
elseif contains(envName, 'gap')
    tWindow = [3, 6];
    tLocalPlanWindow = [3, 6];
    bStateBasedWindow = false;
    bZeroOrigin = false;
    nameSuffix = '02realtime';
    duration = 35.0;
    goal_position = [5.0, 0.0];
end

% Set the bag dir
bagDir = '../bags/';
if bLoadFinal
    bagDir = [bagDir, envName, '_final/'];
end

%% Load the data
linearStateFig = 1;
GRFVectorsFig = 2;
solveTimeFig = 3;
horizonLengthFig = 4;
predictionHorizonFig = 5;
simplePercentageFig = 6;
dataMat = {};

for i = 1:length(configNames)
    % Search bags for the config
    bagNameList = dir([bagDir, envName, '_', nameSuffix, '_', lower(configNames{i}), '*']);
    
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
        [data, trialName] = parseQuadBag(trialName, bagDir);
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

        % Record solving time
        solveTimesList{l} = localPlan{i}.solveTime;
        highSolveTimesList{l} = sum(localPlan{i}.solveTime(localPlan{i}.solveTime > dt))/duration;

        % Identify the start and end indices
        end_idx = find((vecnorm(stateGroundTruth{i}.position(:,1:2) - goal_position, 2, 2) < success_tol) & ...
            (vecnorm(stateGroundTruth{i}.velocity(:,1),2,2) < 1e-2),1);
        start_time = localPlan{i}.time(1);
        if (isempty(end_idx))
            end_idx = size(stateGroundTruth{i}.time,1);
        end
        end_time = stateGroundTruth{i}.time(end_idx);

        % Trim all data based on start/end times
        bStateBasedTrim = true;
        if bStateBasedTrim
            % Identify valid indices for each source of data
            state_trim_idx = stateGroundTruth{i}.time < start_time | stateGroundTruth{i}.time > end_time;
            ctrl_trim_idx = controlGRFs{i}.time < start_time | controlGRFs{i}.time > end_time;
            local_trim_plan_idx = localPlan{i}.time < start_time | localPlan{i}.time > end_time;

            % Trim all out-of-range state data
            stateGroundTruth{i}.position(state_trim_idx,:) = [];
            stateGroundTruth{i}.velocity(state_trim_idx,:) = [];
            stateGroundTruth{i}.orientationRPY(state_trim_idx,:) = [];
            stateGroundTruth{i}.angularVelocity(state_trim_idx,:) = [];
            stateGroundTruth{i}.time(state_trim_idx) = [];

            % Trim all out-of-range control data
            for j = 1:length(controlGRFs{i}.vectors)
                controlGRFs{i}.vectors{j}(ctrl_trim_idx,:) = [];
                controlGRFs{i}.points{j}(ctrl_trim_idx,:) = [];
            end
            controlGRFs{i}.time(ctrl_trim_idx) = [];

            % Trim all out-of-range local plan data
            localPlan{i}.complexitySchedule(local_trim_plan_idx) = [];
            localPlan{i}.elementTimes(local_trim_plan_idx) = [];
            localPlan{i}.cost(local_trim_plan_idx) = [];
            localPlan{i}.horizonLength(local_trim_plan_idx) = [];
            localPlan{i}.iterations(local_trim_plan_idx) = [];
            localPlan{i}.solveTime(local_trim_plan_idx) = [];
            localPlan{i}.time(local_trim_plan_idx) = [];

            % Align times
            stateGroundTruth{i}.time = stateGroundTruth{i}.time - start_time;
            controlGRFs{i}.time = controlGRFs{i}.time - start_time;
            localPlan{i}.time = localPlan{i}.time - start_time;
            for j = 1:length(localPlan{i}.elementTimes)
                shiftedElementTimes = localPlan{i}.elementTimes{j} - start_time;
                localPlan{i}.elementTimes{j} = shiftedElementTimes;
            end

            % Set x(0) = 0
            if bZeroOrigin
                stateGroundTruth{i}.position(:,1) = stateGroundTruth{i}.position(:,1)  - stateGroundTruth{i}.position(1,1) ;
            end
        end

        maxTime = max(maxTime, max(localPlan{i}.time));
        settlingTimeList{l} = stateGroundTruth{i}.time(end) - stateGroundTruth{i}.time(1);

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

        % Record percent of horizon simplified
        trajTimesVec = [];
        elementTimesVec = [];
        complexityScheduleVec = [];
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

        % Only plot specified index
        if bPlotting && (successList{l} || (l == length(bagNameList) && sum(successList{l}) == 0))
            %% Plot the data
            
            % Only plot once per config
            bPlotting = false;
            
            % Plot the state
            linearStateFig = figure(linearStateFig);
            linearStateFig.Name = "linear_states";

            if bStateBasedWindow
                tWindow(1) = stateGroundTruth{i}.time(1);
                tWindow(2) = stateGroundTruth{i}.time(end);
            end
            plot_idx = find(stateGroundTruth{i}.time>= tWindow(1) & stateGroundTruth{i}.time<= tWindow(2));            

            if contains(envName, 'step')
                % pitch
                subplot(1,2,1); hold on
                plot(stateGroundTruth{i}.time(plot_idx), stateGroundTruth{i}.orientationRPY(plot_idx,2), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
                ylabel('Pitch (rad)')
                xlabel('Time (s)')
                annotation('arrow',[0.15 0.19],[0.6 0.74])
                axis tight
                
                % yaw
                subplot(1,2,2); hold on
                plot(stateGroundTruth{i}.time(plot_idx), stateGroundTruth{i}.orientationRPY(plot_idx,3), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
                ylabel('Yaw (rad)')
                xlabel('Time (s)')
                axis tight
                set(linearStateFig, 'Position', [100 100 1200 600])
                annotation('arrow',[0.58 0.63],[0.81 0.67])
            elseif contains(envName, 'gap')
                % z state
                subplot(1,2,1); hold on
                plot(stateGroundTruth{i}.time(plot_idx), stateGroundTruth{i}.velocity(plot_idx,1), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
                ylabel('X Velocity (m/s)')
                xlabel('Time (s)')
                annotation('arrow',[0.16 0.24],[0.86 0.84])
                axis tight
                
                subplot(1,2,2); hold on
                plot(stateGroundTruth{i}.time(plot_idx), stateGroundTruth{i}.position(plot_idx,3), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
                ylabel('Z Position (m)')
                xlabel('Time (s)')
                annotation('arrow',[0.62 0.67],[0.56 0.7])
                axis tight
                set(linearStateFig, 'Position', [100 100 1200 600])
                
            elseif contains(envName, 'flat')
                % z state
                subplot(1,2,1); hold on
                plot(stateGroundTruth{i}.time(plot_idx), stateGroundTruth{i}.position(plot_idx,1), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
                ylabel('X Position (m)')
                xlabel('Time (s)')
                axis tight
                
                subplot(1,2,2);
                hold on;
                plot(stateGroundTruth{i}.time(plot_idx), stateGroundTruth{i}.velocity(plot_idx,1), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
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
            plot_idx = find(controlGRFs{i}.time>= tWindow(1) & controlGRFs{i}.time<= tWindow(2));
            
            GRFVectorsFig = figure(GRFVectorsFig);
            GRFVectorsFig.Name = "grfs";
            
            hold on;
            plot(controlGRFs{i}.time(plot_idx), grfNorm(plot_idx), 'Color', configColors{i}, 'LineWidth', lineWidth, 'LineStyle', configLinestyles{i});
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
            ylabel('Simplicity Set Size (\% of horizon)');
            %     ytickformat('percentage')
            yticks([0 25 50 75 100]);
            yticklabels({'0\%', '25\%', '50\%', '75\%', '100\%'});
            axis([0, maxTime, -5 105])
            set(simplePercentageFig, 'Position', [100 100 1200 830])
            box off
            
            %
            % ~~~~~~~~~~~~~~~~~~~
            %
            
            
            % Plot the prediction horizons
            predictionHorizonFig = figure(predictionHorizonFig);
            predictionHorizonFig.Name = "prediction_horizon";
            subplot(2,2,i)
            
            % Declare sizes
            markerSizeSimple = 10;
            markerSizeComplex = 10;
            
            % Extract times into vectors
            elementTimesVec = [];
            complexityScheduleVec = [];
            trajTimesVec = [];

            if bStateBasedWindow
                tLocalPlanWindow(1) = 0;
                tLocalPlanWindow(2) = localPlan{i}.time(end);
            end

            plot_idx = find(localPlan{i}.time>= tLocalPlanWindow(1) & localPlan{i}.time<= tLocalPlanWindow(2));
            
            localPlan{i}.time = localPlan{i}.time(plot_idx);
            localPlan{i}.solveTime = localPlan{i}.solveTime(plot_idx,:);
            localPlan{i}.elementTimes = localPlan{i}.elementTimes(plot_idx,:);
            localPlan{i}.cost = localPlan{i}.cost(plot_idx,:);
            localPlan{i}.iterations = localPlan{i}.iterations(plot_idx,:);
            localPlan{i}.horizonLength = localPlan{i}.horizonLength(plot_idx,:);
            localPlan{i}.complexitySchedule = localPlan{i}.complexitySchedule(plot_idx,:);
            
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

            hold on;
            scatter(elementTimesVecSimple, trajTimesVecSimple, markerSizeSimple, configColors{1}, 'filled'); hold on;
            if ~isempty(trajTimesVecComplex)
                scatter(elementTimesVecComplex, trajTimesVecComplex, markerSizeComplex, configColors{2}, 'filled');
            end
            set(gca, 'YDir','reverse')

            axis equal
            axis([tLocalPlanWindow(1), tLocalPlanWindow(2) + 0.72, tLocalPlanWindow(1), tLocalPlanWindow(2)])

            if i >= 3
                xlabel('Horizon Time, $k + i$ (s)');
            end
            if mod(i,2) == 1
                ylabel('Wall Time, $k$ (s)');
            end
            if (i == 2)
                legend(predictionHorizonFig.CurrentAxes, 'Simple', 'Complex');
            end

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

if bComputeStats
    % Define the observed frequencies for each configuration
    successes = successCount; % Number of successes for each configuration
    failures = size(successList,1) - successCount; % Number of failures for each configuration

    % Create a contingency table
    observed = [successes; failures];

    % Calculate the expected frequencies
    total_trials = successes + failures;
    total_successes = sum(successes);
    expected_successes = total_trials * (total_successes / sum(total_trials));
    expected_failures = total_trials - expected_successes;
    expected = [expected_successes; expected_failures];

    % Perform the chi-squared test
    chi2 = sum(sum((observed - expected).^2 ./ expected));
    degrees_of_freedom = (size(observed, 1) - 1) * (size(observed, 2) - 1);
    p_value = 1 - chi2cdf(chi2, degrees_of_freedom);

    % Display the results
    disp(['Success rate chi-squared statistic: ' num2str(chi2)]);
    disp(['Success rate degrees of freedom: ' num2str(degrees_of_freedom)]);
    disp(['Success rate p-value: ' num2str(p_value)]);
end

save('~/dataMat.m', 'dataMat')

% Add figures to array
figArray = [linearStateFig, GRFVectorsFig, solveTimeFig, horizonLengthFig, predictionHorizonFig, simplePercentageFig];

%% Save the logs and figures in one directory
logDir = [];
if bSave
    logDir = saveMultiLog(bagDir, envName, nameSuffix, configNames, figArray);
end
end