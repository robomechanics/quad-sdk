function [t_rel, x_max, y_max, z_max, J_max, tau_vec] = processForceLogCore(bagDir, namespace)
if ~endsWith(pwd, '/quad_logger/scripts')
    error('This script must be run from quad_logger/scripts/');
end
if nargin < 2, namespace = 'robot_1'; end

% bagDir = "/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/robot_1_quad_log_go2_20250830_1601/robot_1_quad_log_go2_20250830_1601_0.mcap"
% Load the bag by directory path
[data] = parseQuadBagVel(bagDir, namespace);
stateGroundTruth = data.stateGroundTruth;
stateGroundTruthBodyFrame = data.stateGroundTruthBodyFrame;


% Window
t0 = stateGroundTruth.time(1);
T  = 25;                                           % horizon (s)
dt = median(diff(stateGroundTruth.time));          % ~0.2s typical
t_rel = (0:dt:T).';                                % relative time grid
t_abs = t0 + t_rel;  

% ----- interpolate positions to t_abs and pick farthest in XY
stance_pose = [0.0 0.0];                           % XY only
[uniq_t_pos, ia_pos] = unique(stateGroundTruth.time, 'stable');
pos_i = interp1(uniq_t_pos, stateGroundTruth.position(ia_pos,:), ...
                t_abs, 'linear', 'extrap');        % (#t_rel)x3

pos_xy = pos_i(:,1:2);
dists  = vecnorm(pos_xy - stance_pose, 2, 2);      % XY distance
[~, j] = max(dists);

x_max = pos_i(j,1);
y_max = pos_i(j,2);
z_max = pos_i(j,3);

% ----- efforts → tau_vec on t_rel
[uniq_t_bf, ia_bf] = unique(stateGroundTruth.time, 'stable');
unique_effort = stateGroundTruth.jointEffort(ia_bf, :);
tau_i = interp1(uniq_t_bf, unique_effort, t_abs, 'linear', 'extrap');   % (#t_rel)xJ
tau_norm_sq = sum(tau_i.^2, 2);                                         % (#t_rel)x1
tau_vec = tau_norm_sq;

% ----- torque cost over the horizon
J_max = sum(tau_norm_sq) * dt;

figure('Name','Squared Torque Norm','Color','w');
plot(t_rel, tau_vec, 'b-', 'LineWidth', 2);
grid on; box on;
xlabel('Time [s]', 'Interpreter','latex');
ylabel('$\|\tau\|_2^2 \; \mathrm{(N \cdot m)^2}$', 'Interpreter','latex');
title('Per-timestep Squared Torque Norm', 'Interpreter','latex');

end