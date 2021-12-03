function varargout = plotControl(control_traj,varargin)

if (length(varargin)>=2)
    GRFVectorsFig = varargin{2}(1);
    GRFPointsFig = varargin{2}(2);
    GRFContactStatesFig = varargin{2}(3);
else
    GRFVectorsFig = 1;
    GRFPointsFig = 2;
    GRFContactStatesFig = 3;
end

if (length(varargin)>=1)
    lineStyle = varargin{1};
else
    lineStyle = '-';
end

controlColorVector = {[166,25,46]/255, [0,45,114]/255, [0,132,61]/255, [242,169,0]/255};

num_feet = 4;

%% Plot GRF Vectors

GRFVectorsFig = figure(GRFVectorsFig);
subplot(3,1,1)
hold on;
for i = 1:num_feet
   plot(control_traj.time, control_traj.vectors{i}(:,1), ...
       'Color', controlColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('X (N)')
title('GRF Vectors')
axis([min(control_traj.time), max(control_traj.time), -200, 200])

subplot(3,1,2)
hold on;
for i = 1:num_feet
   plot(control_traj.time, control_traj.vectors{i}(:,2), ...
       'Color', controlColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Y (N)')
axis([min(control_traj.time), max(control_traj.time), -200, 200])

subplot(3,1,3)
hold on;
for i = 1:num_feet
   plot(control_traj.time, control_traj.vectors{i}(:,3), ...
       'Color', controlColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Z (N)')
xlabel('Time (s)')
axis([min(control_traj.time), max(control_traj.time), -200, 200])
legend('FL','BL','FR','BR','location','east')
set(GRFVectorsFig, 'Position', [100 100 800 600])

%% Export

if nargout >0
    varargout{1} = [GRFVectorsFig, GRFPointsFig, GRFContactStatesFig];
end
