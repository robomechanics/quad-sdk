function varargout = plotLocalPlan(localPlan,tWindow,varargin)
% plotLocalPlan Plot control data for a trajectory.
%   plotLocalPlan(controlTraj, tWindow) generates plots for local plan
%   information.
%
%   plotLocalPlan(localPlan, tWindow, lineStyle) plots all data with the line
%   style specified by lineStyle.
% 
%   plotLocalPlan(localPlan, tWindow, lineStyle, titles) adds figure titles if
%   titles == true.
%
%   plotLocalPlan(localPlan, tWindow, lineStyle, titles, figArray) plots data
%   from controlTraj on existing figure handles specified by figArray.
%
%   figArray = plotLocalPlan(___) returns the handles of each figure contained
%   within figArray

if (length(varargin)>=3) && ~isempty(varargin{3})
    solveDataFig = varargin{3}(1);
    predictionHorizonFig = varargin{3}(2);
else
    h =  findobj('type','figure');
    n = length(h);
    
    solveDataFig = n+1;
    predictionHorizonFig = n+2;
end

if (length(varargin)>=2)
    titles = varargin{2};
else
    titles = true;
end


if (length(varargin)>=1)
    lineStyle = varargin{1};
else
    lineStyle = '-';
end

if isempty(tWindow)
    traj_idx = 1:length(localPlan.time);
else
    traj_idx = find(localPlan.time>= tWindow(1) & localPlan.time<= tWindow(2));
end

localPlan.time = localPlan.time(traj_idx);
localPlan.solveTime = localPlan.solveTime(traj_idx,:);
localPlan.elementTimes = localPlan.elementTimes(traj_idx,:);
localPlan.cost = localPlan.cost(traj_idx,:);
localPlan.iterations = localPlan.iterations(traj_idx,:);
localPlan.horizonLength = localPlan.horizonLength(traj_idx,:);
localPlan.complexitySchedule = localPlan.complexitySchedule(traj_idx,:);

localPlanColorVector = {[166,25,46]/255, [0,45,114]/255, [0,132,61]/255, [242,169,0]/255};

%% Plot Solve Data

solveDataFig = figure(solveDataFig);
solveDataFig.Name = "solve_data";
subplot(3,1,1)
hold on;
scale = 1000; % s to ms
plot(localPlan.time, scale*localPlan.solveTime, ...
    'Color', localPlanColorVector{1}, 'LineWidth', 2, 'LineStyle', lineStyle);
if titles
    title('Solve Data')
end
ylabel('Solve Time (ms)');
axis([min(localPlan.time), max(localPlan.time), 0, 100])

subplot(3,1,2)
hold on;
plot(localPlan.time, localPlan.iterations, ...
    'Color', localPlanColorVector{1}, 'LineWidth', 2, 'LineStyle', lineStyle);
ylabel('Iterations');
axis([min(localPlan.time), max(localPlan.time), 0, 20])

subplot(3,1,3)
hold on;
plot(localPlan.time, localPlan.cost, ...
    'Color', localPlanColorVector{1}, 'LineWidth', 2, 'LineStyle', lineStyle);
xlabel('Current Time (s)');
ylabel('Cost');
axis([min(localPlan.time), max(localPlan.time), 0, max(localPlan.cost)])
set(solveDataFig, 'Position', [100 100 1200 900])

align_Ylabels(solveDataFig);

%% Plot Horizon Evolution

predictionHorizonFig = figure(predictionHorizonFig);
predictionHorizonFig.Name = "prediction_horizon";
hold on;

% Declare sizes
markerSizeSimple = 3;
markerSizeComplex = 6;

% Extract times into vectors
elementTimesVec = [];
complexityScheduleVec = [];
trajTimesVec = [];
for i = 1:length(localPlan.elementTimes)
    horizonLength = localPlan.horizonLength(i);
    trajTimesVec = [trajTimesVec; localPlan.time(i)*ones(horizonLength,1)];
    elementTimesVec = [elementTimesVec; localPlan.elementTimes{i}'];
    complexityScheduleVec = [complexityScheduleVec; localPlan.complexitySchedule{i}'];
end

% Sort into simple and complex sets
simpleIdx = find(complexityScheduleVec==0);
complexIdx = find(complexityScheduleVec==1);
trajTimesVecSimple = trajTimesVec(simpleIdx);
elementTimesVecSimple = elementTimesVec(simpleIdx);
trajTimesVecComplex = trajTimesVec(complexIdx);
elementTimesVecComplex = elementTimesVec(complexIdx);

scatter(trajTimesVecSimple, elementTimesVecSimple, markerSizeSimple, localPlanColorVector{2}, 'filled');

if ~isempty(trajTimesVecComplex)
    scatter(trajTimesVecComplex, elementTimesVecComplex, markerSizeComplex, localPlanColorVector{1});
    legend('Simple', 'Complex', 'Location', 'Southeast')
end

if titles
    title('Prediction Horizon')
end
xlabel('Current Time, $i$ (s)');
ylabel('Predicted Time, $k$ (s)');
axis equal
axis([0, max(localPlan.time), 0, max(localPlan.time, [], 'all')])
set(predictionHorizonFig, 'Position', [100 100 600 600])

%% Export

if nargout >0
    varargout{1} = [solveDataFig, predictionHorizonFig];
end
