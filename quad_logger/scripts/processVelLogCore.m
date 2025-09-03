function [t_rel, err_x, err_y, err_w, tau_vec] = processVelLogCore(bagDir, namespace)
if ~endsWith(pwd, '/quad_logger/scripts')
    error('This script must be run from quad_logger/scripts/');
end
if nargin < 2, namespace = 'robot_1'; end

% Load the bag by directory path
[data] = parseQuadBagVel(bagDir, namespace);
stateGroundTruthBodyFrame = data.stateGroundTruthBodyFrame;
cmdVels = data.cmdVels;

% Applied cmd (EMA)
alpha = 0.10; scale = 1;
t_cmd = cmdVels.time(:); N = numel(t_cmd);
applied_cmd = zeros(N,3);
applied_cmd(1,1) = alpha*scale*cmdVels.velocity(1,1);
applied_cmd(1,2) = alpha*scale*cmdVels.velocity(1,2);
applied_cmd(1,3) = alpha*scale*cmdVels.angularVelocity(1,3);
for k = 2:N
    applied_cmd(k,:) = (1-alpha)*applied_cmd(k-1,:) + ...
                       alpha*scale*[cmdVels.velocity(k,1), cmdVels.velocity(k,2), cmdVels.angularVelocity(k,3)];
end

% Interp GT
[uniq_t, ia] = unique(stateGroundTruthBodyFrame.time, 'stable');
V_gt_i = interp1(uniq_t, stateGroundTruthBodyFrame.velocity(ia,:), t_cmd, 'linear', 'extrap');
W_gt_i = interp1(uniq_t, stateGroundTruthBodyFrame.angularVelocity(ia,:), t_cmd, 'linear', 'extrap');

% Window
tol = 1e-6;
idx0 = find(any(abs(applied_cmd) > tol, 2), 1, 'first'); if isempty(idx0), idx0 = 1; end
t0   = t_cmd(idx0);
T    = 10; dt = median(diff(t_cmd));
t_rel_full = t_cmd - t0;
win  = (t_rel_full >= 0 & t_rel_full <= T);

err_x_raw = abs(V_gt_i(:,1) - applied_cmd(:,1));
err_y_raw = abs(V_gt_i(:,2) - applied_cmd(:,2));
err_w_raw = abs(W_gt_i(:,3) - applied_cmd(:,3));

unique_effort = stateGroundTruthBodyFrame.jointEffort(ia, :);
tau_i = interp1(uniq_t, unique_effort, t_cmd, 'linear', 'extrap');
tau_norm_sq = sum(tau_i.^2, 2);

% Resample
t_rel = (0:dt:T).';
[t_u, ia] = unique(t_rel_full, 'stable');

err_x = interp1(t_u, err_x_raw(ia), t_rel, 'linear', 'extrap');
err_y = interp1(t_u, err_y_raw(ia), t_rel, 'linear', 'extrap');
err_w = interp1(t_u, err_w_raw(ia), t_rel, 'linear', 'extrap');

tau_vec = interp1(t_u, tau_norm_sq(ia), t_rel, 'linear', 'extrap');
end
