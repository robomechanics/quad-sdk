function processVelLog(varargin)
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

%% Load the data

% Load the data
[data, trialName] = parseQuadBag(trialName, namespace);
stateEstimate = data.stateEstimate;
stateGroundTruth = data.stateGroundTruth;
stateGroundTruthBodyFrame = data.stateGroundTruthBodyFrame;
stateTrajectory = data.stateTrajectory;
stateGRFs = data.stateGRFs;
controlGRFs = data.controlGRFs;
localPlan = data.localPlan;
cmdVels = data.cmdVels;

%% Plot the data
% Plot the state
alpha = 0.10;
scale = 1;
t_cmd = cmdVels.time(:); 
N = numel(cmdVels.time);
applied_cmd = zeros(N,3);

% init from zeros (like the C++)
applied_cmd(1,1) = alpha*scale*cmdVels.velocity(1,1);          % x
applied_cmd(1,2) = alpha*scale*cmdVels.velocity(1,2);          % y
applied_cmd(1,3) = alpha*scale*cmdVels.angularVelocity(1,3);   % yaw

for k = 2:N
    applied_cmd(k,1) = (1-alpha)*applied_cmd(k-1,1) + alpha*scale*cmdVels.velocity(k,1);
    applied_cmd(k,2) = (1-alpha)*applied_cmd(k-1,2) + alpha*scale*cmdVels.velocity(k,2);
    applied_cmd(k,3) = (1-alpha)*applied_cmd(k-1,3) + alpha*scale*cmdVels.angularVelocity(k,3);
end
t_cmd(1:5)
[unique_times, ia] = unique(stateGroundTruthBodyFrame.time, 'stable');  
unique_vel = stateGroundTruthBodyFrame.velocity(ia, :);
unique_angvel = stateGroundTruthBodyFrame.angularVelocity(ia, :);
V_gt_i = interp1(unique_times,unique_vel, t_cmd, 'linear', 'extrap');
W_gt_i = interp1(unique_times,unique_angvel, t_cmd, 'linear', 'extrap');

tol = 1e-6;
idx0 = find(any(abs(applied_cmd) > tol, 2), 1, 'first');  if isempty(idx0), idx0 = 1; end
t0   = t_cmd(idx0);
t1   = t0 + 10;
idx1 = find(t_cmd >= t1, 1, 'first');                     if isempty(idx1), idx1 = N; end
win  = idx0:idx1;

%% Quick sanity print
fig1 = figure('Name','vel_tracking'); clf(fig1); tl1 = tiledlayout(fig1, 3,1);
sgtitle(tl1, 'Velocity Tracking Performance', 'FontSize',25, 'Interpreter','latex')
xlabel(tl1,'time [s]',  'FontSize',18, 'Interpreter', 'latex');
ylabel(tl1, 'Body Velocity (m/s), (rad/s)', 'FontSize',18, 'Interpreter', 'latex')
nexttile(tl1); hold on; grid on; subtitle('$V_x$', 'Interpreter','latex')
plot(t_cmd(win), V_gt_i(win,1), 'b-', 'LineWidth', 2);
plot(t_cmd(win), applied_cmd(win,1), 'r-', 'LineWidth', 2);

nexttile(tl1); hold on; grid on; subtitle('$V_y$', 'Interpreter','latex')
plot(t_cmd(win), V_gt_i(win,2), 'b-', 'LineWidth', 2);
plot(t_cmd(win), applied_cmd(win,2), 'r-', 'LineWidth', 2);

nexttile(tl1); hold on; grid on; subtitle('$\dot{\psi}$', 'Interpreter','latex')
plot(t_cmd(win), W_gt_i(win,3), 'b-', 'LineWidth', 2);
plot(t_cmd(win), applied_cmd(win,3), 'r-', 'LineWidth', 2);
legend({'Ground Truth','Commanded'}, 'Interpreter','latex');

% RMS Error on Velocity Tracking
err_x = V_gt_i(win,1) - applied_cmd(win,1);    % vx error
err_y = V_gt_i(win,2) - applied_cmd(win,2);    % vy error
err_w = W_gt_i(win,3) - applied_cmd(win,3);    % yaw rate error

rmse_x = sqrt(mean(err_x.^2))
rmse_y = sqrt(mean(err_y.^2))
rmse_w = sqrt(mean(err_w.^2))

fig2 = figure('Name','tracking_error'); clf(fig2); tl2= tiledlayout(fig2, 3,1);
sgtitle(tl2, 'Velocity Tracking Error', 'FontSize', 25, 'Interpreter', 'latex')
xlabel(tl2,'time [s]',  'FontSize',18, 'Interpreter', 'latex');
ylabel(tl2, 'Tracking Error (m/s), (rad/s)', 'FontSize',18, 'Interpreter', 'latex')
nexttile(tl2); hold on; grid on; subtitle('$V_x$', 'Interpreter','latex')
plot(t_cmd(win), err_x, 'b-', 'LineWidth', 2);
nexttile(tl2); hold on; grid on; subtitle('$V_y$', 'Interpreter','latex')
plot(t_cmd(win), err_y, 'b-', 'LineWidth', 2);
nexttile(tl2); hold on; grid on; subtitle('$\dot{\psi}$', 'Interpreter','latex')
plot(t_cmd(win), err_w, 'b-', 'LineWidth', 2);

% Sum of the Squared Norms of Torque
unique_effort = stateGroundTruthBodyFrame.jointEffort(ia, :);
tau_i = interp1(unique_times, unique_effort, t_cmd, 'linear', 'extrap');
tau_norm_sq = sum(tau_i.^2, 2);
fig3 = figure('Name','torque_sq'); clf;
plot(t_cmd(win), tau_norm_sq(win), 'b-', 'LineWidth', 2);
grid on;
title('Per-timestep Squared Torque Norm, $\|\tau\|^2$', 'Interpreter','latex');
xlabel('time [s]', 'Interpreter','latex');
ylabel('$(\mathrm{N\cdot m})^2$', 'Interpreter','latex');
dt = t_cmd(2) - t_cmd(1);
J = sum(tau_norm_sq(win)) * dt

% Cost of Transport
m = 16.247; g = 9.81;
unique_jointVel = stateGroundTruthBodyFrame.jointVelocity(ia, :);
qd_i = interp1(unique_times, unique_jointVel, t_cmd, 'linear', 'extrap');
P_joint = tau_i .* qd_i;
P_inst  = sum(abs(P_joint), 2); 
E = sum(P_inst) * dt; 

vx = V_gt_i(:,1); vy = V_gt_i(:,2);           % world-frame linear velocities
speed_xy = hypot(vx, vy);                     % horizontal speed
d = sum(speed_xy) * dt; 
CoT = E / (m * g * d)
disp(['Cmd Vel [vx vy wz] = ' num2str([median(cmdVels.velocity(win,1)) median(cmdVels.velocity(win,2)) median(cmdVels.angularVelocity(win,3))],'%.3f ')])

figArray = [fig1 fig2 fig3];

%% Save the logs and figures in one directory
logDir = [];
if endsWith(trialName, "_0")
    trialName = extractBefore(trialName, strlength(trialName) - 1);
end
if bSave
    logDir = saveLog(trialName, figArray);
end

%% Animate and save

% if bAnimate
%     robot_path = '../../quad_simulator/spirit_description/models/spirit/urdf/spirit.urdf';
%     robot = importrobot(robot_path);
%     videosDir = fullfile(logDir,'videos/');
%     animateData(robot,stateGroundTruth, fullfile(videosDir, trialName), bSave);
% end
