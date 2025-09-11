function aggregateVelLogsCompare(namespace)
% Compare two sets of bag runs (two directories) on the same plots.
% Plots mean ± std as shaded regions for err_x, err_y, err_w.

if nargin < 3, namespace = 'robot_1'; end
if ~endsWith(pwd, '/quad_logger/scripts')
    error('Run from quad_logger/scripts');
end

filesMPC ={
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCVelocity/robot_1_quad_log_go2_20250829_1909/robot_1_quad_log_go2_20250829_1909_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCVelocity/robot_1_quad_log_go2_20250829_1912/robot_1_quad_log_go2_20250829_1912_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCVelocity/robot_1_quad_log_go2_20250829_1914/robot_1_quad_log_go2_20250829_1914_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCVelocity/robot_1_quad_log_go2_20250829_1917/robot_1_quad_log_go2_20250829_1917_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCVelocity/robot_1_quad_log_go2_20250829_1921/robot_1_quad_log_go2_20250829_1921_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCVelocity/robot_1_quad_log_go2_20250829_1923/robot_1_quad_log_go2_20250829_1923_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCVelocity/robot_1_quad_log_go2_20250829_1928/robot_1_quad_log_go2_20250829_1928_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCVelocity/robot_1_quad_log_go2_20250829_1930/robot_1_quad_log_go2_20250829_1930_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCVelocity/robot_1_quad_log_go2_20250829_1932/robot_1_quad_log_go2_20250829_1932_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/MPCVelocity/robot_1_quad_log_go2_20250829_1934/robot_1_quad_log_go2_20250829_1934_0.mcap'
    }

filesLearned ={
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedVelocity/robot_1_quad_log_go2_20250829_1806/robot_1_quad_log_go2_20250829_1806_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedVelocity/robot_1_quad_log_go2_20250829_1810/robot_1_quad_log_go2_20250829_1810_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedVelocity/robot_1_quad_log_go2_20250829_1830/robot_1_quad_log_go2_20250829_1830_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedVelocity/robot_1_quad_log_go2_20250829_1833/robot_1_quad_log_go2_20250829_1833_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedVelocity/robot_1_quad_log_go2_20250829_1837/robot_1_quad_log_go2_20250829_1837_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedVelocity/robot_1_quad_log_go2_20250829_1840/robot_1_quad_log_go2_20250829_1840_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedVelocity/robot_1_quad_log_go2_20250829_1846/robot_1_quad_log_go2_20250829_1846_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedVelocity/robot_1_quad_log_go2_20250829_1859/robot_1_quad_log_go2_20250829_1859_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedVelocity/robot_1_quad_log_go2_20250829_1901/robot_1_quad_log_go2_20250829_1901_0.mcap'
    '/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags/LearnedVelocity/robot_1_quad_log_go2_20250829_1904/robot_1_quad_log_go2_20250829_1904_0.mcap'
    }

% --- Helper to load & aggregate one directory of bag dirs
    function [t_ref, mux, sigx, muy, sigy, muw, sigw, mtau, sigtau, kept] = aggOneList(fileList, namespace)
        Ex = []; Ey = []; Ew = []; kept = 0; t_ref = []; Tau = [];
        for i = 1:numel(fileList)
            try
                fprintf('Processing MCAP: %s (ns=%s)\n', fileList{i}, namespace);
                [t_rel, ex, ey, ew, tau_vec] = processVelLogCore(fileList{i}, namespace);

                % Align time bases
                if isempty(t_ref)
                    t_ref = t_rel;
                elseif numel(t_rel) ~= numel(t_ref) || any(abs(t_rel - t_ref) > 1e-6)
                    ex = interp1(t_rel, ex, t_ref, 'linear', 'extrap');
                    ey = interp1(t_rel, ey, t_ref, 'linear', 'extrap');
                    ew = interp1(t_rel, ew, t_ref, 'linear', 'extrap');
                    tau_vec = interp1(t_rel, tau_vec, t_ref, 'linear', 'extrap');
                end

                Ex(:,end+1) = ex; %#ok<AGROW>
                Ey(:,end+1) = ey; %#ok<AGROW>
                Ew(:,end+1) = ew; %#ok<AGROW>
                Tau(:, end+1) = tau_vec;
                kept = kept + 1;
                fprintf('OK: %s\n', fileList{i});
            catch ME
                fprintf('SKIP: %s (%s)\n', fileList{i}, ME.message);
                disp(getReport(ME, 'extended'));
            end
        end

        if kept == 0, error('No valid runs parsed.'); end

        mux = mean(Ex,2); sigx = std(Ex,0,2);
        muy = mean(Ey,2); sigy = std(Ey,0,2);
        muw = mean(Ew,2); sigw = std(Ew,0,2);
        mtau = mean(Tau,2); sigtau = std(Tau,0,2);
    end
% --- Process both dirs
[t1, mux1, sigx1, muy1, sigy1, muw1, sigw1, mtau1, sigtau1, n1] = aggOneList(filesMPC, namespace);
[t2, mux2, sigx2, muy2, sigy2, muw2, sigw2, mtau2, sigtau2, n2] = aggOneList(filesLearned, namespace);

% --- Plot overlays
figure('Name','Velocity Tracking Errors (Comparison)','Color','w');
colors = lines(2); % two distinct colors
labels = {sprintf('MPC (%d runs)', n1), ...
          sprintf('Learned (%d runs)', n2)};

subplot(3,1,1); hold on;
fill([t1; flipud(t1)], [mux1-sigx1; flipud(mux1+sigx1)], colors(1,:), ...
     'FaceAlpha',0.2,'EdgeColor','none');
h1 = plot(t1, mux1, 'Color', colors(1,:), 'LineWidth',1.5);
fill([t2; flipud(t2)], [mux2-sigx2; flipud(mux2+sigx2)], colors(2,:), ...
     'FaceAlpha',0.2,'EdgeColor','none');
h2 = plot(t2, mux2, 'Color', colors(2,:), 'LineWidth',1.5);
ylabel('$V_{x,\mathrm{err}}$ \, (m/s)', 'Interpreter','latex');
title('Average Velocity Tracking Errors', 'Interpreter','latex');
legend([h1, h2], labels, 'Interpreter','none','Location','best');

subplot(3,1,2); hold on; 
fill([t1; flipud(t1)], [muy1-sigy1; flipud(muy1+sigy1)], colors(1,:), ...
     'FaceAlpha',0.2,'EdgeColor','none');
plot(t1, muy1, 'Color', colors(1,:), 'LineWidth',1.5);
fill([t2; flipud(t2)], [muy2-sigy2; flipud(muy2+sigy2)], colors(2,:), ...
     'FaceAlpha',0.2,'EdgeColor','none');
plot(t2, muy2, 'Color', colors(2,:), 'LineWidth',1.5);
ylabel('$V_{y,\mathrm{err}}$ \,(m/s)', 'Interpreter','latex');

subplot(3,1,3); hold on;
fill([t1; flipud(t1)], [muw1-sigw1; flipud(muw1+sigw1)], colors(1,:), ...
     'FaceAlpha',0.2,'EdgeColor','none');
plot(t1, muw1, 'Color', colors(1,:), 'LineWidth',1.5);
fill([t2; flipud(t2)], [muw2-sigw2; flipud(muw2+sigw2)], colors(2,:), ...
     'FaceAlpha',0.2,'EdgeColor','none');
plot(t2, muw2, 'Color', colors(2,:), 'LineWidth',1.5);
ylabel('$V_{\omega,\mathrm{err}}$ \,(rad/s)', 'Interpreter', 'latex');
xlabel('Time (s)');

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
ylim([0 600]);
title('Squared L2 Norm of Torques', 'Interpreter','latex');
legend([h1, h2], labels, 'Interpreter','none','Location','best');

end
