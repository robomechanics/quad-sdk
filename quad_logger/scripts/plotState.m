function varargout = plotState(stateTraj,tWindow,varargin)
% plotState Plot state data for a trajectory.
%   plotState(stateTraj, tWindow) generates plots for each component of
%   the state message -- COM position, velocity, orientation, angular
%   velocity, joint states, and foot states -- for each time t =
%   [tWindow(1), tWindow(2)]. If tWindow = [], all data points are
%   displayed.
%
%   plotState(stateTraj, tWindow, lineStyle) plots all data with the line
%   style specified by lineStyle.
% 
%   plotState(stateTraj, tWindow, lineStyle, titles) adds figure titles if
%   titles == true.
%
%   plotState(stateTraj, tWindow, lineStyle, titles, figArray) plots data
%   from stateTraj on existing figure handles specified by figArray.
%
%   figArray = plotState(___) returns the handles of each figure contained
%   within figArray

% Use existing figure handles if requested
if (length(varargin)>=3) && ~isempty(varargin{3})
    COMTrajFig = varargin{3}(1);
    linearStateFig = varargin{3}(2);
    angularStateFig = varargin{3}(3);
    jointPositionFig = varargin{3}(4);
    jointVelocityFig = varargin{3}(5);
    jointEffortFig = varargin{3}(6);
    footPositionFig = varargin{3}(7);
    footVelocityFig = varargin{3}(8);
else
    h =  findobj('type','figure');
    n = length(h);
    
    COMTrajFig = n+1;
    linearStateFig = n+2;
    angularStateFig = n+3;
    jointPositionFig = n+4;
    jointVelocityFig = n+5;
    jointEffortFig = n+6;
    footPositionFig = n+7;
    footVelocityFig = n+8;
end

% Turn off the plot titles if requested
if (length(varargin)>=2)
    titles = varargin{2};
else
    titles = true;
end

% Set the line style if specified
if (length(varargin)>=1)
    lineStyle = varargin{1};
else
    lineStyle = '-';
end

% Set the time window if specified
if isempty(tWindow)
    traj_idx = 1:length(stateTraj.time);
else
    traj_idx = find(stateTraj.time>= tWindow(1) & stateTraj.time<= tWindow(2));
end

% Trim the data based on the requested window
stateTraj.time = stateTraj.time(traj_idx);
stateTraj.position = stateTraj.position(traj_idx,:);
stateTraj.velocity = stateTraj.velocity(traj_idx,:);
stateTraj.orientationRPY = stateTraj.orientationRPY(traj_idx,:);
stateTraj.orientationQuat = stateTraj.orientationQuat(traj_idx,:);
stateTraj.angularVelocity = stateTraj.angularVelocity(traj_idx,:);
stateTraj.jointPosition = stateTraj.jointPosition(traj_idx,:);
stateTraj.jointVelocity = stateTraj.jointVelocity(traj_idx,:);
stateTraj.jointEffort = stateTraj.jointEffort(traj_idx,:);
for i = 1:length(stateTraj.footPosition)
    stateTraj.footPosition{i} = stateTraj.footPosition{i}(traj_idx,:);
    stateTraj.footVelocity{i} = stateTraj.footVelocity{i}(traj_idx,:);
end

footColorVector = {[166,25,46]/255, [0,45,114]/255, [0,132,61]/255, [242,169,0]/255};
jointColorVector = footColorVector;
bodyColorVector = footColorVector;

num_feet = 4;

%% Plot spatial trajectory
COMTrajFig = figure(COMTrajFig);
COMTrajFig.Name = "com_trajectory";

hold on; view(3);
plot3(stateTraj.position(:,1), stateTraj.position(:,2), stateTraj.position(:,3), ...
    'Color', cmuColor('red-web'), 'LineStyle', lineStyle);
for i = 1:num_feet
   plot3(stateTraj.footPosition{i}(:,1), stateTraj.footPosition{i}(:,2), ...
       stateTraj.footPosition{i}(:,3), 'Color', footColorVector{i}, ...
       'LineStyle', lineStyle);
end
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
if titles
    title('Body and Foot Trajectories')
end
axis equal

%% Plot linear state data
linearStateFig = figure(linearStateFig);
linearStateFig.Name = "linear_states";

subplot(2,1,1); hold on
for i = 1:3
    plot(stateTraj.time, stateTraj.position(:,i), 'Color', bodyColorVector{i},...
        'LineStyle', lineStyle);
end
ylabel('Position (m)')
if titles
    title('Linear States')
end
legend('X','Y','Z','location','East')
axis tight

subplot(2,1,2); hold on
for i = 1:3
    plot(stateTraj.time, stateTraj.velocity(:,i), 'Color', bodyColorVector{i},...
        'LineStyle', lineStyle);
end
ylabel('Velocity (m/s)')
xlabel('Time (s)')
axis tight
set(linearStateFig, 'Position', [100 100 1200 600])

align_Ylabels(linearStateFig);

%% Plot angular state data
angularStateFig = figure(angularStateFig);
angularStateFig.Name = "angular_states";

subplot(2,1,1); hold on
for i = 1:3
    plot(stateTraj.time, stateTraj.orientationRPY(:,i), 'Color', bodyColorVector{i},...
        'LineStyle', lineStyle);
end
ylabel('Ang. Pos. (rad)')
if titles
    title('Angular States')
end
legend('Roll','Pitch','Yaw','location','East')
axis([min(stateTraj.time), max(stateTraj.time), -pi, pi])

subplot(2,1,2); hold on
for i = 1:3
    plot(stateTraj.time, stateTraj.angularVelocity(:,i), 'Color', bodyColorVector{i},...
        'LineStyle', lineStyle);
end
ylabel('Ang. Vel. (rad/s)')
xlabel('Time (s)')
axis tight
set(angularStateFig, 'Position', [100 100 1200 600])

align_Ylabels(angularStateFig);

%% Plot joint positions

% Specify indices for leg joints
abIndex = [1:3:12];
hipIndex = [2:3:12];
kneeIndex = [3:3:12];
abadLim = [-1, 1];
hipLim = [-pi/2, pi];
kneeLim = [0, pi];

% Plot joint positions
jointPositionFig = figure(jointPositionFig);
jointPositionFig.Name = "joint_positions";

subplot(3,1,1); hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.jointPosition(:,abIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Ab/Ad (rad)')
if titles
    title('Joint Angles')
end
axis([min(stateTraj.time), max(stateTraj.time), abadLim(1), abadLim(2)])

subplot(3,1,2); hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.jointPosition(:,hipIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Hip (rad)')
legend('FL','BL','FR','BR','location','East')
axis([min(stateTraj.time), max(stateTraj.time), hipLim(1), hipLim(2)])

subplot(3,1,3); hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.jointPosition(:,kneeIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Knee (rad)')
xlabel('Time (s)')
axis([min(stateTraj.time), max(stateTraj.time), kneeLim(1), kneeLim(2)])
set(jointPositionFig, 'Position', [100 100 1200 900])

align_Ylabels(jointPositionFig);

%% Plot joint velocities
abadVelLim = [-37.7, 37.7];
hipVelLim = [-37.7, 37.7];
kneeVelLim = [-25, 25];

jointVelocityFig = figure(jointVelocityFig);
jointVelocityFig.Name = "joint_velocities";

subplot(3,1,1); hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.jointVelocity(:,abIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Ab/Ad (rad/s)')
if titles
    title('Joint Velocities')
end
axis([min(stateTraj.time), max(stateTraj.time), abadVelLim(1), abadVelLim(2)])

subplot(3,1,2); hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.jointVelocity(:,hipIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Hip (rad/s)')
legend('FL','BL','FR','BR','location','East')
axis([min(stateTraj.time), max(stateTraj.time), hipVelLim(1), hipVelLim(2)])

subplot(3,1,3); hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.jointVelocity(:,kneeIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Knee (rad/s)')
xlabel('Time (s)')
axis([min(stateTraj.time), max(stateTraj.time), kneeVelLim(1), kneeVelLim(2)])
set(jointVelocityFig, 'Position', [100 100 1200 900])

align_Ylabels(jointVelocityFig);

%% Plot estimate joint effort
abadEffLim = [-21, 21];
hipEffLim = [-21, 21];
kneeEffLim = [-32, 32];

jointEffortFig = figure(jointEffortFig);
jointEffortFig.Name = "joint_effort";

subplot(3,1,1); hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.jointEffort(:,abIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Ab/Ad (A)')
if titles
    title('Joint Effort')
end
axis([min(stateTraj.time), max(stateTraj.time), abadEffLim(1), abadEffLim(2)])

subplot(3,1,2); hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.jointEffort(:,hipIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Hip (A)')
legend('FL','BL','FR','BR','location','East')
axis([min(stateTraj.time), max(stateTraj.time), hipEffLim(1), hipEffLim(2)])

subplot(3,1,3); hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.jointEffort(:,kneeIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Knee (A)')
xlabel('Time (s)')
axis([min(stateTraj.time), max(stateTraj.time), kneeEffLim(1), kneeEffLim(2)])
set(jointEffortFig, 'Position', [100 100 1200 900])

align_Ylabels(jointEffortFig);

%% Plot foot positions
footPositionFig = figure(footPositionFig);
footPositionFig.Name = "foot_positions";

subplot(3,1,1)
hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.footPosition{i}(:,1), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('X (m)')
if titles
    title('Foot Positions')
end
% axis([min(stateTraj.time), max(stateTraj.time), -1, 4])
axis tight

subplot(3,1,2)
hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.footPosition{i}(:,2), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Y (m)')
% axis([min(stateTraj.time), max(stateTraj.time), -2 2])
axis tight
legend('FL','BL','FR','BR','location','East')

subplot(3,1,3)
hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.footPosition{i}(:,3), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Z (m)')
xlabel('Time (s)')
% axis([min(stateTraj.time), max(stateTraj.time), 0, 0.3])
axis tight
set(footPositionFig, 'Position', [100 100 1200 900])

align_Ylabels(footPositionFig);

%% Plot foot velocities
footVelocityFig = figure(footVelocityFig);
footVelocityFig.Name = "foot_velocities";

vel_axis = max(abs(cell2mat(stateTraj.footVelocity)), [], 'all');
if ~isempty(footVelocityFig.CurrentAxes)
    vel_axis = max(vel_axis, max(abs(footVelocityFig.CurrentAxes.YLim)));
end
subplot(3,1,1)
hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.footVelocity{i}(:,1), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('X (m/s)')
if titles
    title('Foot Velocities')
end
axis([min(stateTraj.time), max(stateTraj.time), -vel_axis, vel_axis])

subplot(3,1,2)
hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.footVelocity{i}(:,2), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Y (m/s)')
axis([min(stateTraj.time), max(stateTraj.time), -vel_axis, vel_axis])
legend('FL','BL','FR','BR','location','East')

subplot(3,1,3)
hold on;
for i = 1:num_feet
   plot(stateTraj.time, stateTraj.footVelocity{i}(:,3), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Z (m/s)')
xlabel('Time (s)')
axis([min(stateTraj.time), max(stateTraj.time), -vel_axis, vel_axis])
set(footVelocityFig, 'Position', [100 100 1200 900])

align_Ylabels(footVelocityFig);

%% Export

if nargout >0
    varargout{1} = [COMTrajFig, linearStateFig, angularStateFig, jointPositionFig, jointVelocityFig, ...
        jointEffortFig, footPositionFig, footVelocityFig];
end
