function processLog(varargin)

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
[data1, trialName1] = parseQuadBag(trialName, namespace);
[data2, trialName2] = parseQuadBag(trialName, namespace);
[data3, trialName3] = parseQuadBag(trialName, namespace);

%% Extract Data Stuct From Each File Type
stateEstimate_flat = data1.stateEstimate;
stateGroundTruth_flat = data1.stateGroundTruth;

stateEstimate_rough25cm = data2.stateEstimate;
stateGroundTruth_rough25cm = data2.stateGroundTruth;

stateEstimate_rough40cm = data3.stateEstimate;
stateGroundTruth_rough40cm = data3.stateGroundTruth;

%% Flat Ground Data
%Section and Isolate Time and Velocity Arrays
est_ind = find(stateEstimate_flat.time > 0, 1);
ind = find(stateEstimate_flat.time(est_ind) - stateGroundTruth_flat.time < 0.001, 1);

t_flat_gt = stateGroundTruth_flat.time(ind:end);
t_flat_est = stateEstimate_flat.time(est_ind:end);

vx_flat_gt = stateGroundTruth_flat.velocity(:,1);
vx_flat_est = stateEstimate_flat.velocity(:,1);
vy_flat_gt = stateGroundTruth_flat.velocity(:,2);
vy_flat_est = stateEstimate_flat.velocity(:,2);
vz_flat_gt = stateGroundTruth_flat.velocity(:,3);
vz_flat_est = stateEstimate_flat.velocity(:,3);


vx_flat_gt = vx_flat_gt(ind:end);
vx_flat_est = vx_flat_est(est_ind:end);
vy_flat_gt = vy_flat_gt(ind:end);
vy_flat_est = vy_flat_est(est_ind:end);
vz_flat_gt = vz_flat_gt(ind:end);
vz_flat_est = vz_flat_est(est_ind:end);

% Visualize the Ground Truth and Estimation Vectors
[t1,y1] = removeRepeat(t_flat_est,vx_flat_est);
[t2,y2] = removeRepeat(t_flat_est,vy_flat_est);
[t3,y3] = removeRepeat(t_flat_est,vz_flat_est);

vx_flat_est_interp = interp1(t1, y1, t_flat_gt);
vy_flat_est_interp = interp1(t2, y2, t_flat_gt);
vz_flat_est_interp = interp1(t3, y3, t_flat_gt);


error_x_flat = abs(vx_flat_gt - vx_flat_est_interp);
error_y_flat = abs(vy_flat_gt - vy_flat_est_interp);
error_z_flat = abs(vz_flat_gt - vz_flat_est_interp);
[t_flat_gt, error_x_flat, error_y_flat, error_z_flat] = removeNaN(t_flat_gt, error_x_flat, error_y_flat, error_z_flat);

%% Rough25cm Data
est_ind = find(stateEstimate_rough25cm.time > 0, 1);
ind = find(stateEstimate_rough25cm.time(est_ind) - stateGroundTruth_rough25cm.time < 0.001, 1);

t_rough25cm_gt = stateGroundTruth_rough25cm.time(ind:end);
t_rough25cm_est = stateEstimate_rough25cm.time(est_ind:end);

vx_rough25cm_gt = stateGroundTruth_rough25cm.velocity(:,1);
vx_rough25cm_est = stateEstimate_rough25cm.velocity(:,1);
vy_rough25cm_gt = stateGroundTruth_rough25cm.velocity(:,2);
vy_rough25cm_est = stateEstimate_rough25cm.velocity(:,2);
vz_rough25cm_gt = stateGroundTruth_rough25cm.velocity(:,3);
vz_rough25cm_est = stateEstimate_rough25cm.velocity(:,3);


vx_rough25cm_gt = vx_rough25cm_gt(ind:end);
vx_rough25cm_est = vx_rough25cm_est(est_ind:end);
vy_rough25cm_gt = vy_rough25cm_gt(ind:end);
vy_rough25cm_est = vy_rough25cm_est(est_ind:end);
vz_rough25cm_gt = vz_rough25cm_gt(ind:end);
vz_rough25cm_est = vz_rough25cm_est(est_ind:end);

% Visualize the Ground Truth and Estimation Vectors
[t4,y4] = removeRepeat(t_rough25cm_est,vx_rough25cm_est);
[t5,y5] = removeRepeat(t_rough25cm_est,vy_rough25cm_est);
[t6,y6] = removeRepeat(t_rough25cm_est,vz_rough25cm_est);

vx_rough25cm_est_interp = interp1(t4, y4, t_rough25cm_gt);
vy_rough25cm_est_interp = interp1(t5, y5, t_rough25cm_gt);
vz_rough25cm_est_interp = interp1(t6, y6, t_rough25cm_gt);


error_x_rough25cm = abs(vx_rough25cm_gt - vx_rough25cm_est_interp);
error_y_rough25cm = abs(vy_rough25cm_gt - vy_rough25cm_est_interp);
error_z_rough25cm = abs(vz_rough25cm_gt - vz_rough25cm_est_interp);

[t_rough25cm_gt, error_x_rough25cm, error_y_rough25cm, error_z_rough25cm] = removeNaN(t_rough25cm_gt, error_x_rough25cm, error_y_rough25cm, error_z_rough25cm);


%% Rough40cm Data
est_ind = find(stateEstimate_rough40cm.time > 0, 1);
ind = find(stateEstimate_rough40cm.time(est_ind) - stateGroundTruth_rough40cm.time < 0.001, 1);

t_rough40cm_gt = stateGroundTruth_rough40cm.time(ind:end);
t_rough40cm_est = stateEstimate_rough40cm.time(est_ind:end);

vx_rough40cm_gt = stateGroundTruth_rough40cm.velocity(:,1);
vx_rough40cm_est = stateEstimate_rough40cm.velocity(:,1);
vy_rough40cm_gt = stateGroundTruth_rough40cm.velocity(:,2);
vy_rough40cm_est = stateEstimate_rough40cm.velocity(:,2);
vz_rough40cm_gt = stateGroundTruth_rough40cm.velocity(:,3);
vz_rough40cm_est = stateEstimate_rough40cm.velocity(:,3);


vx_rough40cm_gt = vx_rough40cm_gt(ind:end);
vx_rough40cm_est = vx_rough40cm_est(est_ind:end);
vy_rough40cm_gt = vy_rough40cm_gt(ind:end);
vy_rough40cm_est = vy_rough40cm_est(est_ind:end);
vz_rough40cm_gt = vz_rough40cm_gt(ind:end);
vz_rough40cm_est = vz_rough40cm_est(est_ind:end);

% Visualize the Ground Truth and Estimation Vectors
[t7,y7] = removeRepeat(t_rough40cm_est,vx_rough40cm_est);
[t8,y8] = removeRepeat(t_rough40cm_est,vy_rough40cm_est);
[t9,y9] = removeRepeat(t_rough40cm_est,vz_rough40cm_est);

vx_rough40cm_est_interp = interp1(t7, y7, t_rough40cm_gt);
vy_rough40cm_est_interp = interp1(t8, y8, t_rough40cm_gt);
vz_rough40cm_est_interp = interp1(t9, y9, t_rough40cm_gt);


error_x_rough40cm = abs(vx_rough40cm_gt - vx_rough40cm_est_interp);
error_y_rough40cm = abs(vy_rough40cm_gt - vy_rough40cm_est_interp);
error_z_rough40cm = abs(vz_rough40cm_gt - vz_rough40cm_est_interp);

[t_rough40cm_gt, error_x_rough40cm, error_y_rough40cm, error_z_rough40cm] = removeNaN(t_rough40cm_gt, error_x_rough40cm, error_y_rough40cm, error_z_rough40cm);

%% Visualize the Data for All Three Maps
%Visualize in X
figure(1)
plot(t_flat_gt, error_x_flat, 'r-')
hold on
plot(t_rough25cm_gt, error_x_rough25cm, 'g-')
plot(t_rough40cm_gt, error_x_rough40cm, 'b-')
title("Velocity Estimation Error in X")
legend("flat", "rough25cm", "rough40cm")
xlabel("Time (s)")
ylabel("Body Velocity (m/s)")

%Visualize in Y
figure(2)
plot(t_flat_gt, error_y_flat, 'r-')
hold on
plot(t_rough25cm_gt, error_y_rough25cm, 'g-')
plot(t_rough40cm_gt, error_y_rough40cm, 'b-')
title("Velocity Estimation Error in Y")
legend("flat", "rough25cm", "rough40cm")
xlabel("Time (s)")
ylabel("Body Velocity (m/s)")

%Visualize in Z
figure(3)
plot(t_flat_gt, error_z_flat, 'r-')
hold on
plot(t_rough25cm_gt, error_z_rough25cm, 'g-')
plot(t_rough40cm_gt, error_z_rough40cm, 'b-')
title("Velocity Estimation Error in Z")
legend("flat", "rough25cm", "rough40cm")
xlabel("Time (s)")
ylabel("Body Velocity (m/s)")

%% Generate iROS Figure
avg_error_x_flat = sum(error_x_flat)/length(t_flat_gt);
avg_error_x_rough25cm = sum(error_x_rough25cm)/length(t_rough25cm_gt);
avg_error_x_rough40cm = sum(error_x_rough40cm)/(length(t_rough40cm_gt));
avg_error_y_flat = sum(error_y_flat)/length(t_flat_gt);
avg_error_y_rough25cm = sum(error_y_rough25cm)/length(t_rough25cm_gt);
avg_error_y_rough40cm = sum(error_y_rough40cm)/(length(t_rough40cm_gt));
avg_error_z_flat = sum(error_z_flat)/length(t_flat_gt);
avg_error_z_rough25cm = sum(error_z_rough25cm)/length(t_rough25cm_gt);
avg_error_z_rough40cm = sum(error_z_rough40cm)/(length(t_rough40cm_gt));

vals = [avg_error_x_flat avg_error_x_rough25cm avg_error_x_rough40cm;
        avg_error_y_flat avg_error_y_rough25cm avg_error_y_rough40cm;
        avg_error_z_flat avg_error_z_rough25cm avg_error_z_rough40cm;
        ];
labels = categorical({'X','Y','Z'});
labels = reordercats(labels,{'X','Y','Z'});
figure(4)
bar(labels, vals)
title("Average Estimation Error on Different Terrain")
legend("flat", "rough25cm", "rough40cm")
xlabel("Time (s)")
ylabel("Average Body Velocity Error (m/s)")
end

function [t, y] = removeRepeat(A, B)
    % Given Input Pair, Time Vector A, and Data Vector B
    % Return corresponding t,y vectors with no repeats
    [C, ia, ic] = unique(A);
    t = A(ia);
    y = B(ia);
end

function [t, y1, y2, y3] = removeNaN(A, B, C, D)
    %Given Input Pair, Time Vector A, and Data Vector B
    % Remove Nan Values to ensure a valid sum\
    ind = isnan(B);
    t = A(~ind);
    y1 = B(~ind);
    y2 = C(~ind);
    y3 = D(~ind);
end
