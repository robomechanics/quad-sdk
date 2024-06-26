function processLog(varargin)
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
% if ~endsWith(pwd, '/quad_logger/scripts')
%     error('This script must be run from quad_logger/scripts/');
% end

%% Select rosbag to parse

% If a trial name is provided, use that to save everything
if nargin>0
    trialName = varargin{1};
    namespace = varargin{2};
else
    trialName = ''; % Set to '' to load via GUI
    namespace = 'robot_1'; % Namespace of the robot bag, set to '' if none
end

%% Set parameters

bSave = true;                       % Save the figures/videos
bAnimate = false;                   % Animate the trajectory (no translation)
bTitles = true;                     % Turn on figure titles
bPlotLocalPlanInfo = true;          % Turn on to plot local plan information
tWindowStates = [];                 % Specify time window for state (use [] for no clipping)
tWindowControl = [];                % Specify time window for control (use [] for no clipping)
tWindowLocalPlan = [];              % Specify time window for local plan (use [] for no clipping)

    %% Create a directory for saving plots
if bSave
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    plotDir = fullfile('..', 'plots', [trialName '_' timestamp]);
    if ~exist(plotDir, 'dir')
        mkdir(plotDir);
    end
end

%% Load the data

% Load the data
[data, trialName] = parseQuadBag(trialName, namespace);
stateEstimate = data.stateEstimate;
stateGroundTruth = data.stateGroundTruth;
% stateTrajectory = data.stateTrajectory;
% stateGRFs = data.stateGRFs;
% controlGRFs = data.controlGRFs;
% localPlan = data.localPlan;
% ind = find(stateGroundTruth.time == stateEstimate.time(1), 1)
% t_ind_se = find(stateEstimate.time > 0, 1);
% t_ind_gt = length(stateGroundTruth.time) - (length(stateEstimate.time)) + (t_ind_se);
% tstart_gt = stateGroundTruth.time(t_ind_gt);
% tstart_se = stateEstimate.time(t_ind_se);
% 
% t_state = stateGroundTruth.time(t_ind_gt: end);
% state_estimate_pos_x = stateEstimate.position(:,1);
% state_estimate_vel_x = stateEstimate.velocity(:,1);
% ground_truth_pos_x = stateGroundTruth.position(:,1);
% ground_truth_vel_x = stateGroundTruth.velocity(:,1);
% 
% state_estimate_pos_x = state_estimate_pos_x(t_ind_se: end);
% state_estimate_vel_x = state_estimate_vel_x(t_ind_se: end);
% ground_truth_pos_x = ground_truth_pos_x(t_ind_gt: end);
% ground_truth_vel_x = ground_truth_vel_x(t_ind_gt: end);
figure(1)
t = tiledlayout(3,1);
nexttile
% subplot(3,1,1);
hold on
plot(stateGroundTruth.time, stateGroundTruth.position(:,1), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.position(:,1), 'r-', 'LineWidth',2)
% xlim([2 7])
% ylim([-3 3])
title("X position")
legend("Ground Truth", "State Estimate")
grid on
nexttile
% subplot(3,1,2);
hold on
plot(stateGroundTruth.time, stateGroundTruth.position(:,2), 'b', 'LineWidth',2 )
plot(stateEstimate.time, stateEstimate.position(:,2), 'r-' , 'LineWidth',2)
% xlim([2 7])
% ylim([-1.5 1.5])
title("Y position")
legend("Ground Truth", "State Estimate")
grid on
nexttile
% subplot(3,1,3);
hold on
plot(stateGroundTruth.time, stateGroundTruth.position(:,3), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.position(:,3), 'r-','LineWidth',2)
% xlim([1 12])
% ylim([-1.5 1.5])
title("Z position")
legend("Ground Truth", "State Estimate")
grid on

xlabel(t, "Time (s)")
ylabel(t, 'Body position (m/s)')

if bSave
    saveas(gcf, fullfile(plotDir, 'Position.png'));
end

figure(2)
t = tiledlayout(3,1);
nexttile
% subplot(3,1,1);
hold on
plot(stateGroundTruth.time, stateGroundTruth.velocity(:,1), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.velocity(:,1), 'r-', 'LineWidth',2)
xlim([1 12])
% ylim([-3 3])
title("X Velocity")
legend("Ground Truth", "State Estimate")
grid on
nexttile
% subplot(3,1,2);
hold on
plot(stateGroundTruth.time, stateGroundTruth.velocity(:,2), 'b', 'LineWidth',2 )
plot(stateEstimate.time, stateEstimate.velocity(:,2), 'r-' , 'LineWidth',2)
xlim([1 12])
% ylim([-1.5 1.5])
title("Y Velocity")
legend("Ground Truth", "State Estimate")
grid on
nexttile
% subplot(3,1,3);
hold on
plot(stateGroundTruth.time, stateGroundTruth.velocity(:,3), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.velocity(:,3), 'r-','LineWidth',2)
xlim([1 12])
% ylim([-1.5 1.5])
title("Z Velocity")
legend("Ground Truth", "State Estimate")
grid on

xlabel(t, "Time (s)")
ylabel(t, 'Body Velocity (m/s)')

if bSave
    saveas(gcf, fullfile(plotDir, 'Velocity.png'));
end

figure(3)
t = tiledlayout(4,1);
nexttile
% subplot(3,1,1);
hold on
plot(stateGroundTruth.time, stateGroundTruth.orientationQuat(:,1), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.orientationQuat(:,1), 'r-', 'LineWidth',2)
% xlim([2 7])
% ylim([-3 3])
title("W Orientation")
legend("Ground Truth", "State Estimate")
grid on
nexttile
% subplot(3,1,2);
hold on
plot(stateGroundTruth.time, stateGroundTruth.orientationQuat(:,2), 'b', 'LineWidth',2 )
plot(stateEstimate.time, stateEstimate.orientationQuat(:,2), 'r-' , 'LineWidth',2)
% xlim([2 7])
% ylim([-1.5 1.5])
title("X Orientation")
legend("Ground Truth", "State Estimate")
grid on
nexttile
% subplot(3,1,3);
hold on
plot(stateGroundTruth.time, stateGroundTruth.orientationQuat(:,3), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.orientationQuat(:,3), 'r-','LineWidth',2)
% xlim([2 7])
% ylim([-1.5 1.5])
title("Y Orientation")
legend("Ground Truth", "State Estimate")
grid on
nexttile

hold on
plot(stateGroundTruth.time, stateGroundTruth.orientationQuat(:,4), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.orientationQuat(:,4), 'r-','LineWidth',2)
% xlim([2 7])
% ylim([-1.5 1.5])
title("Z Orientation")
legend("Ground Truth", "State Estimate")
grid on

xlabel(t, "Time (s)")

if bSave
    saveas(gcf, fullfile(plotDir, 'Orientation.png'));
end

figure(4)
t = tiledlayout(3,1);
nexttile
% subplot(3,1,1);
hold on
plot(stateGroundTruth.time, stateGroundTruth.orientationRPY(:,1), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.orientationRPY(:,1), 'r-', 'LineWidth',2)
% xlim([2 7])
% ylim([-3 3])
title("Roll")
legend("Ground Truth", "State Estimate")
grid on
nexttile
% subplot(3,1,2);
hold on
plot(stateGroundTruth.time, stateGroundTruth.orientationRPY(:,2), 'b', 'LineWidth',2 )
plot(stateEstimate.time, stateEstimate.orientationRPY(:,2), 'r-' , 'LineWidth',2)
% xlim([2 7])
% ylim([-1.5 1.5])
title("Pitch")
legend("Ground Truth", "State Estimate")
grid on
nexttile
% subplot(3,1,3);
hold on
plot(stateGroundTruth.time, stateGroundTruth.orientationRPY(:,3), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.orientationRPY(:,3), 'r-','LineWidth',2)
% xlim([2 7])
% ylim([-1.5 1.5])
title("Yaw")
legend("Ground Truth", "State Estimate")
grid on

xlabel(t, "Time (s)")

if bSave
    saveas(gcf, fullfile(plotDir, 'RPY.png'));
end

figure(5)
t = tiledlayout(3,1);
nexttile
% subplot(3,1,1);
hold on
plot(stateGroundTruth.time, stateGroundTruth.angularVelocity(:,1), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.angularVelocity(:,1), 'r-', 'LineWidth',2)
% xlim([2 7])
% ylim([-3 3])
title("Angular Vel X")
legend("Ground Truth", "State Estimate")
grid on
nexttile
% subplot(3,1,2);
hold on
plot(stateGroundTruth.time, stateGroundTruth.angularVelocity(:,2), 'b', 'LineWidth',2 )
plot(stateEstimate.time, stateEstimate.angularVelocity(:,2), 'r-' , 'LineWidth',2)
% xlim([2 7])
% ylim([-1.5 1.5])
title("Angular Vel Y")
legend("Ground Truth", "State Estimate")
grid on
nexttile
% subplot(3,1,3);
hold on
plot(stateGroundTruth.time, stateGroundTruth.angularVelocity(:,3), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.angularVelocity(:,3), 'r-','LineWidth',2)
% xlim([2 7])
% ylim([-1.5 1.5])
title("Angular Vel Z")
legend("Ground Truth", "State Estimate")
grid on

xlabel(t, "Time (s)")

if bSave
    saveas(gcf, fullfile(plotDir, 'AngVel.png'));
end



%% Plot the data

% Plot the state
% stateFigs = [];
% if ~isempty(stateGroundTruth)
%     [stateFigs] = plotState(stateGroundTruth,tWindowStates,'-', bTitles, stateFigs);
% end
% if ~isempty(stateTrajectory)
%     [stateFigs] = plotState(stateTrajectory,tWindowStates, ':', bTitles, stateFigs);
% end
% if ~isempty(stateEstimate)
%     [stateFigs] = plotState(stateEstimate,tWindowStates, '--', bTitles, stateFigs);
% end

% Plot the control
% controlFigs = [];
% if ~isempty(stateGRFs)
%     controlFigs = plotControl(stateGRFs,tWindowControl,'-', bTitles, controlFigs);
% end
% if ~isempty(controlGRFs)
%     controlFigs = plotControl(controlGRFs,tWindowControl,':', bTitles,controlFigs);
% end

% Plot local plan information if desired
% localPlanFigs = [];
% if bPlotLocalPlanInfo && ~isempty(localPlan)
%     localPlanFigs = plotLocalPlan(localPlan,tWindowLocalPlan,'-', bTitles,localPlanFigs);
% end

% Add figures to array
% figArray = [stateFigs, controlFigs, localPlanFigs];

%% Save the logs and figures in one directory
% logDir = [];
% if bSave
%     logDir = saveLog(trialName, figArray);
% end

%% Animate and save

% if bAnimate
%     robot_path = '../../quad_simulator/spirit_description/urdf/spirit.urdf';
%     robot = importrobot(robot_path);
%     videosDir = fullfile(logDir,'videos/');
%     animateData(robot,stateGroundTruth, fullfile(videosDir, trialName), bSave);
% end
