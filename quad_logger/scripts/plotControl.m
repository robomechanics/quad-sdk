function varargout = plotControl(controlTraj,tTindow,varargin)
% plotControl Plot control data for a trajectory.
%   plotControl(controlTraj, tWindow) generates plots for each component of
%   the control GRF message for each time t = [tWindow(1), tWindow(2)]. If 
%   tWindow = [], all data points are displayed.
%
%   plotControl(controlTraj, tWindow, lineStyle) plots all data with the line
%   style specified by lineStyle.
% 
%   plotControl(controlTraj, tWindow, lineStyle, titles) adds figure titles if
%   titles == true.
%
%   plotControl(controlTraj, tWindow, lineStyle, titles, figArray) plots data
%   from controlTraj on existing figure handles specified by figArray.
%
%   figArray = plotControl(___) returns the handles of each figure contained
%   within figArray

if (length(varargin)>=3) && ~isempty(varargin{3})
    GRFVectorsFig = varargin{3}(1);
else
    h =  findobj('type','figure');
    n = length(h);
    
    GRFVectorsFig = n+1;
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

if isempty(tTindow)
    traj_idx = 1:length(controlTraj.time);
else
    traj_idx = find(controlTraj.time>= tTindow(1) & controlTraj.time<= tTindow(2));
end

controlTraj.time = controlTraj.time(traj_idx);
for i = 1:length(controlTraj.vectors)
    controlTraj.vectors{i} = controlTraj.vectors{i}(traj_idx,:);
    controlTraj.points{i} = controlTraj.points{i}(traj_idx,:);
    controlTraj.contactStates{i} = controlTraj.contactStates{i}(traj_idx,:);
end

controlColorVector = {[166,25,46]/255, [0,45,114]/255, [0,132,61]/255, [242,169,0]/255};

%% Plot GRF Vectors (By XYZ components, all legs)

% num_feet = 4;
% GRFVectorsFig = figure(GRFVectorsFig);
% subplot(3,1,1)
% hold on;
% for i = 1:num_feet
%    plot(t_window, control_traj.vectors{i}(:,1), ...
%        'Color', controlColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
% end
% yl = ylabel('X (N)')
% if titles
%     title('GRF Vectors')
% end
% axis([min(t_window), max(t_window), -50, 50])
% 
% subplot(3,1,2)
% hold on;
% for i = 1:num_feet
%    plot(t_window, control_traj.vectors{i}(:,2), ...
%        'Color', controlColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
% end
% yl = ylabel('Y (N)')
% axis([min(t_window), max(t_window), -50, 50])
% 
% subplot(3,1,3)
% hold on;
% for i = 1:num_feet
%    plot(t_window, control_traj.vectors{i}(:,3), ...
%        'Color', controlColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
% end
% yl = ylabel('Z (N)')
% xlabel('Time (s)')
% axis([min(t_window), max(t_window), -100, 100])
% legend('FL','BL','FR','BR','location','east')
% set(GRFVectorsFig, 'Position', [100 100 800 600])

%% Plot GRF Vectors (One leg, all XYZ components)

leg_idx = 4;

GRFVectorsFig = figure(GRFVectorsFig);
GRFVectorsFig.Name = "grfs";

hold on;
plot(controlTraj.time, controlTraj.vectors{leg_idx}(:,1), ...
    'Color', controlColorVector{1}, 'LineWidth', 2, 'LineStyle', lineStyle);
hold on;
plot(controlTraj.time, controlTraj.vectors{leg_idx}(:,2), ...
    'Color', controlColorVector{2}, 'LineWidth', 2, 'LineStyle', lineStyle);
hold on;
plot(controlTraj.time, controlTraj.vectors{leg_idx}(:,3), ...
    'Color', controlColorVector{3}, 'LineWidth', 2, 'LineStyle', lineStyle);

if titles
    title('GRF Vectors')
end
xlabel('Time (s)');
yl = ylabel('GRF (N)');
axis([min(controlTraj.time), max(controlTraj.time), -200, 200])
legend('X','Y','Z','location','east')
set(GRFVectorsFig, 'Position', [100 100 1200 400])

%% Export

if nargout >0
    varargout{1} = GRFVectorsFig;
end
