function aggregateForceLogsCompare(namespace)
% Compare two sets of bag runs (two directories) on the same plots.
% Plots mean ± std as shaded regions for err_x, err_y, err_w.

if nargin < 3, namespace = 'robot_1'; end
if ~endsWith(pwd, '/quad_logger/scripts')
    error('Run from quad_logger/scripts');
end

filesMPC ={
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCForce/robot_1_quad_log_go2_20250830_1445/robot_1_quad_log_go2_20250830_1445_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCForce/robot_1_quad_log_go2_20250830_1500/robot_1_quad_log_go2_20250830_1500_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCForce/robot_1_quad_log_go2_20250830_1540/robot_1_quad_log_go2_20250830_1540_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCForce/robot_1_quad_log_go2_20250830_1552/robot_1_quad_log_go2_20250830_1552_0.mcap'
    }

filesLearned ={
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedForce/robot_1_quad_log_go2_20250830_1306/robot_1_quad_log_go2_20250830_1306_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedForce/robot_1_quad_log_go2_20250830_1422/robot_1_quad_log_go2_20250830_1422_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedForce/robot_1_quad_log_go2_20250830_1425/robot_1_quad_log_go2_20250830_1425_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedForce/robot_1_quad_log_go2_20250830_1432/robot_1_quad_log_go2_20250830_1432_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedForce/robot_1_quad_log_go2_20250830_1601/robot_1_quad_log_go2_20250830_1601_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedForce/robot_1_quad_log_go2_20250830_1611/robot_1_quad_log_go2_20250830_1611_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedForce/robot_1_quad_log_go2_20250830_1624/robot_1_quad_log_go2_20250830_1624_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedForce/robot_1_quad_log_go2_20250830_1626/robot_1_quad_log_go2_20250830_1626_0.mcap'
    }

% --- Helper to load & aggregate one directory of bag dirs
    function [t_ref, x, y, z, J, mtau, sigtau, kept] = aggOneList(fileList, namespace)
        Ex = []; Ey = []; Ew = []; kept = 0; t_ref = []; Tau = [];
        x = []; y = []; z = []; J = [];
        for i = 1:numel(fileList)
            try
                fprintf('Processing MCAP: %s (ns=%s)\n', fileList{i}, namespace);
                [t_rel, x_p, y_p, z_p, J_p, tau_vec] = processForceLogCore(fileList{i}, namespace);

                % Align time bases
                if isempty(t_ref)
                    t_ref = t_rel;
                elseif numel(t_rel) ~= numel(t_ref) || any(abs(t_rel - t_ref) > 1e-6)
                    tau_vec = interp1(t_rel, tau_vec, t_ref, 'linear', 'extrap');
                end

                Tau(:, end+1) = tau_vec;
                x(end+1,1) = x_p; %#ok<AGROW>
                y(end+1,1) = y_p; %#ok<AGROW>
                z(end+1,1) = z_p; %#ok<AGROW>
                J(end+1,1) = J_p; %#ok<AGROW>
                kept = kept + 1;
                fprintf('OK: %s\n', fileList{i});
            catch ME
                fprintf('SKIP: %s (%s)\n', fileList{i}, ME.message);
                disp(getReport(ME, 'extended'));
            end
        end

        if kept == 0, error('No valid runs parsed.'); end

        mtau = mean(Tau,2); sigtau = std(Tau,0,2);
    end
% --- Process both dirs
[t1, x1, y1, z1, J1, mtau1, sigtau1, n1] = aggOneList(filesMPC, namespace);
[t2, x2, y2, z2, J2, mtau2, sigtau2, n2] = aggOneList(filesLearned, namespace);

% --- Plot overlays
disp(x1)
disp(y1)
disp(z1)

figure('Name','Torque Cost (Comparison)','Color','w');
colors = lines(2); % two distinct colors
labels = {sprintf('MPC (%d runs)', n1), ...
          sprintf('Learned (%d runs)', n2)};

hold on;
fill([t1; flipud(t1)], [mtau1-sigtau1; flipud(mtau1+sigtau1)], colors(1,:), ...
     'FaceAlpha',0.2,'EdgeColor','none');
h1 = plot(t1, mtau1, 'Color', colors(1,:), 'LineWidth',1.5);

fill([t2; flipud(t2)], [mtau2-sigtau2; flipud(mtau2+sigtau2)], colors(2,:), ...
     'FaceAlpha',0.2,'EdgeColor','none');
h2 = plot(t2, mtau2, 'Color', colors(2,:), 'LineWidth',1.5);

ylabel('$\|\tau_{\mathrm{sum}}^t\|_{2}$ \,(Nm)', 'Interpreter','latex');
xlabel('Time (s)');
ylim([-2000 5000]);
xlim([7.5 16])
title('Squared L2 Norm of Torques', 'Interpreter','latex');
legend([h1, h2], labels, 'Interpreter','none','Location','best');

% ---- Position Plots
stance_pose = [0.0 0.0];  % x, y only

figure('Name','Farthest XY Positions vs Stance','Color','w'); 
hold on;

colors = lines(2); % reuse for consistency

h_stance  = plot(stance_pose(1), stance_pose(2), 'gp', ...
                 'MarkerFaceColor','g', 'MarkerSize',10);

h_mpc     = scatter(x1, y1, 36, colors(1,:), 'filled');
h_learned = scatter(x2, y2, 36, colors(2,:), 'filled');

xlabel('x (m)'); ylabel('y (m)');
title('Farthest XY Positions from Stance Pose');
legend([h_stance, h_mpc, h_learned], ...
       {'Stance pose', sprintf('MPC (%d runs)', n1), sprintf('Learned (%d runs)', n2)}, ...
       'Location','best');
axis equal


end
