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

bSave = false;                       % Save the figures/videos
bAnimate = false;                   % Animate the trajectory (no translation)
bTitles = true;                     % Turn on figure titles
bPlotLocalPlanInfo = true;          % Turn on to plot local plan information
tWindowStates = [];                 % Specify time window for state (use [] for no clipping)
tWindowControl = [];                % Specify time window for control (use [] for no clipping)
tWindowLocalPlan = [];              % Specify time window for local plan (use [] for no clipping)

%% Load the data

% Load the data
[data, trialName] = parseQuadBag(trialName, namespace);
stateEstimate = data.stateEstimate;
stateGroundTruth = data.stateGroundTruth;

% Remove Repeat Values
index = find(stateEstimate.time >= 0, 1);
t_start = stateEstimate.time(index);
[t, ind] = min(abs(stateGroundTruth.time - t_start));

est_time = stateEstimate.time(index:end);
gt_time = stateGroundTruth.time(ind:end);

gt_x = stateGroundTruth.position(index:end,1)
gt_y = stateGroundTruth.position(index:end,2)
gt_z = stateGroundTruth.position(index:end,3)
gt_vel_x = stateGroundTruth.velocity(index:end,1)
gt_vel_y = stateGroundTruth.velocity(index:end,2)
gt_vel_z = stateGroundTruth.velocity(index:end,3)

est_x = stateEstimate.position(index:end,1)
est_y = stateEstimate.position(index:end,2)
est_z = stateEstimate.position(index:end,3)
est_vel_x = stateEstimate.velocity(index:end,1)
est_vel_y = stateEstimate.velocity(index:end,2)
est_vel_z = stateEstimate.velocity(index:end,3)

% Interpolate the Shorter Vector
interp_est_time = interp1(groundTruth.time, groundTruth.position, stateEstimate.time);
interp_est_time = interp1(groundTruth.time, groundTruth.position, stateEstimate.time);
interp_est_time = interp1(groundTruth.time, groundTruth.position, stateEstimate.time);
interp_est_time = interp1(groundTruth.time, groundTruth.position, stateEstimate.time);
interp_est_time = interp1(groundTruth.time, groundTruth.position, stateEstimate.time);
interp_est_time = interp1(groundTruth.time, groundTruth.position, stateEstimate.time);
interp_est_time = interp1(groundTruth.time, groundTruth.position, stateEstimate.time);


% Compute the RMSE
rmse_x = sqrt(mean((gt_x - interp_est_pose_x));
rmse_y = sqrt(mean((gt_y - interp_est_pos_y).^2));
rmse_z = sqrt(mean((gt_z - interp_est_pos_x).^2));
rmse_vel_x = sqrt(mean((gt_vel_x - interp_est_vel_x).^2));
rmse_vel_y = sqrt(mean((gt_vel_y - interp_est_vel_y).^2));
rmse_vel_z = sqrt(mean((gt_vel_z - interp_est_vel_z).^2));




length(est_time)
length(gt_time)

% n = length(stateEstimate.time)
% m = length(stateGroundTruth.time)
% gt_time = stateGroundTruth.time(end-(n+1):end);
% est_time = stateEstimate.time;
% 
% length(gt_time)
% length(est_time)
% gt_time(1)
% gt_time(2)
% est_time(1)
% est_time(2)

% figure(1)
% hold on
% plot(stateGroundTruth.time, stateGroundTruth.position(:,1), 'b-','LineWidth',2)
% plot(stateEstimate.time, stateEstimate.position(:,1), 'r-', 'LineWidth',2)

% figure(2)
% hold on
% plot(stateGroundTruth.time(end-n:end), stateGroundTruth.position(end-n:end,1), 'b-','LineWidth',2)
% plot(stateEstimate.time, stateEstimate.position(:,1), 'r-', 'LineWidth',2)
% figure(1)
% t = tiledlayout(3,1);
% nexttile
% % subplot(3,1,1);
% hold on
% plot(stateGroundTruth.time, stateGroundTruth.position(:,1), 'b-','LineWidth',2)
% plot(stateEstimate.time, stateEstimate.position(:,1), 'r-', 'LineWidth',2)
% % xlim([2 7])
% % ylim([-3 3])
% title("X position")
% legend("Ground Truth", "State Estimate")
% grid on
% nexttile
% % subplot(3,1,2);
% hold on
% plot(stateGroundTruth.time, stateGroundTruth.position(:,2), 'b', 'LineWidth',2 )
% plot(stateEstimate.time, stateEstimate.position(:,2), 'r-' , 'LineWidth',2)
% % xlim([2 7])
% % ylim([-1.5 1.5])
% title("Y position")
% legend("Ground Truth", "State Estimate")
% grid on
% nexttile
% % subplot(3,1,3);
% hold on
% plot(stateGroundTruth.time, stateGroundTruth.position(:,3), 'b-','LineWidth',2)
% plot(stateEstimate.time, stateEstimate.position(:,3), 'r-','LineWidth',2)
% % xlim([2 7])
% % ylim([-1.5 1.5])
% title("Z position")
% legend("Ground Truth", "State Estimate")
% grid on
% 
% xlabel(t, "Time (s)")
% ylabel(t, 'Body position (m/s)')

% figure(2)
% t = tiledlayout(3,1);
% nexttile
% % subplot(3,1,1);
% hold on
% plot(stateGroundTruth.time, stateGroundTruth.velocity(:,1), 'b-','LineWidth',2)
% plot(stateEstimate.time, stateEstimate.velocity(:,1), 'r-', 'LineWidth',2)
% % xlim([2 7])
% % ylim([-3 3])
% title("X Velocity")
% legend("Ground Truth", "State Estimate")
% grid on
% nexttile
% % subplot(3,1,2);
% hold on
% plot(stateGroundTruth.time, stateGroundTruth.velocity(:,2), 'b', 'LineWidth',2 )
% plot(stateEstimate.time, stateEstimate.velocity(:,2), 'r-' , 'LineWidth',2)
% % xlim([2 7])
% % ylim([-1.5 1.5])
% title("Y Velocity")
% legend("Ground Truth", "State Estimate")
% grid on
% nexttile
% % subplot(3,1,3);
% hold on
% plot(stateGroundTruth.time, stateGroundTruth.velocity(:,3), 'b-','LineWidth',2)
% plot(stateEstimate.time, stateEstimate.velocity(:,3), 'r-','LineWidth',2)
% % xlim([2 7])
% % ylim([-1.5 1.5])
% title("Z Velocity")
% legend("Ground Truth", "State Estimate")
% grid on
% 
% xlabel(t, "Time (s)")
% ylabel(t, 'Body Velocity (m/s)')


% Add figures to array
% figArray = [stateFigs, controlFigs, localPlanFigs];

%% Save the logs and figures in one directory
% logDir = [];
% if bSave
%     logDir = saveLog(trialName, figArray);
% end

