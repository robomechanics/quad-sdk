function varargout = plotState(state_traj,varargin)

if (length(varargin)>=2)
    COMTrajFig = varargin{2}(1);
    linearStateFig = varargin{2}(2);
    angularStateFig = varargin{2}(3);
    jointPositionFig = varargin{2}(4);
    jointVelocityFig = varargin{2}(5);
    jointEffortFig = varargin{2}(6);
    footPositionFig = varargin{2}(7);
    footVelocityFig = varargin{2}(8);
else
    COMTrajFig = 1;
    linearStateFig = 2;
    angularStateFig = 3;
    jointPositionFig = 4;
    jointVelocityFig = 5;
    jointEffortFig = 6;
    footPositionFig = 7;
    footVelocityFig = 8;
end

if (length(varargin)>=1)
    lineStyle = varargin{1};
else
    lineStyle = '-';
end

footColorVector = {[166,25,46]/255, [0,45,114]/255, [0,132,61]/255, [242,169,0]/255};
jointColorVector = footColorVector;
bodyColorVector = footColorVector;

num_feet = 4;

%% Plot spatial trajectory
COMTrajFig = figure(COMTrajFig);
hold on; view(3);
plot3(state_traj.position(:,1), state_traj.position(:,2), state_traj.position(:,3), ...
    'Color', cmuColor('red-web'), 'LineWidth', 4, 'LineStyle', lineStyle);
for i = 1:num_feet
   plot3(state_traj.footPosition{i}(:,1), state_traj.footPosition{i}(:,2), ...
       state_traj.footPosition{i}(:,3), 'Color', footColorVector{i}, 'LineWidth', 4, ...
       'LineStyle', lineStyle);
end
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Body and Foot Trajectories')
axis equal

%% Plot linear state data
linearStateFig = figure(linearStateFig);
subplot(2,1,1); hold on
for i = 1:3
    plot(state_traj.time, state_traj.position(:,i), 'Color', bodyColorVector{i},...
        'LineStyle', lineStyle);
end
ylabel('Position (m)')
title('Linear States')
legend('X','Y','Z','location','best')
axis tight

subplot(2,1,2); hold on
for i = 1:3
    plot(state_traj.time, state_traj.velocity(:,i), 'Color', bodyColorVector{i},...
        'LineStyle', lineStyle);
end
ylabel('Velocity (m/s)')
xlabel('Time (s)')
axis tight
set(linearStateFig, 'Position', [100 100 800 600])

%% Plot angular state data
angularStateFig = figure(angularStateFig);
subplot(2,1,1); hold on
for i = 1:3
    plot(state_traj.time, state_traj.orientationRPY(:,i), 'Color', bodyColorVector{i},...
        'LineStyle', lineStyle);
end
ylabel('Ang. Pos. (rad)')
title('Angular States')
legend('Roll','Pitch','Yaw','location','best')
axis([min(state_traj.time), max(state_traj.time), -pi, pi])

subplot(2,1,2); hold on
for i = 1:3
    plot(state_traj.time, state_traj.angularVelocity(:,i), 'Color', bodyColorVector{i},...
        'LineStyle', lineStyle);
end
ylabel('Ang. Vel. (rad/s)')
xlabel('Time (s)')
axis tight
set(angularStateFig, 'Position', [100 100 800 600])

%% Plot joint positions

% Specify indices for leg joints
abIndex = [1:3:12];
hipIndex = [2:3:12];
kneeIndex = [3:3:12];

% Plot joint positions
jointPositionFig = figure(jointPositionFig);
subplot(3,1,1); hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.jointPosition(:,abIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Ab/Ad (rad)')
title('Joint Angles')
axis([min(state_traj.time), max(state_traj.time), -1, 1])

subplot(3,1,2); hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.jointPosition(:,hipIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Hip (rad)')
legend('FL','BL','FR','BR','location','best')
axis([min(state_traj.time), max(state_traj.time), -pi/2, pi])

subplot(3,1,3); hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.jointPosition(:,kneeIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Knee (rad)')
xlabel('Time (s)')
axis([min(state_traj.time), max(state_traj.time), 0, pi])
set(jointPositionFig, 'Position', [100 100 800 600])

%% Plot joint velocities
jointVelocityFig = figure(jointVelocityFig);
subplot(3,1,1); hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.jointVelocity(:,abIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Ab/Ad (rad/s)')
title('Joint Velocities')
axis([min(state_traj.time), max(state_traj.time), -38, 38])

subplot(3,1,2); hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.jointVelocity(:,hipIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Hip (rad/s)')
legend('FL','BL','FR','BR','location','best')
axis([min(state_traj.time), max(state_traj.time), -38, 38])

subplot(3,1,3); hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.jointVelocity(:,kneeIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Knee (rad/s)')
xlabel('Time (s)')
axis([min(state_traj.time), max(state_traj.time), -25, 25])
set(jointVelocityFig, 'Position', [100 100 800 600])

%% Plot estimate joint effort
jointEffortFig = figure(jointEffortFig);
subplot(3,1,1); hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.jointEffort(:,abIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Ab/Ad (A)')
title('Joint Effort')
axis([min(state_traj.time), max(state_traj.time), -21, 21])

subplot(3,1,2); hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.jointEffort(:,hipIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Hip (A)')
legend('FL','BL','FR','BR','location','best')
axis([min(state_traj.time), max(state_traj.time), -21, 21])

subplot(3,1,3); hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.jointEffort(:,kneeIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Knee (A)')
xlabel('Time (s)')
axis([min(state_traj.time), max(state_traj.time), -32, 32])
set(jointEffortFig, 'Position', [100 100 800 600])


%% Plot foot positions
footPositionFig = figure(footPositionFig);
subplot(3,1,1)
hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.footPosition{i}(:,1), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('X (m)')
title('Foot Positions')
axis([min(state_traj.time), max(state_traj.time), -1, 4])

subplot(3,1,2)
hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.footPosition{i}(:,2), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Y (m)')
axis([min(state_traj.time), max(state_traj.time), -2 2])

subplot(3,1,3)
hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.footPosition{i}(:,3), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Z (m)')
xlabel('Time (s)')
axis([min(state_traj.time), max(state_traj.time), 0, 0.3])
legend('FL','BL','FR','BR','location','east')
set(footPositionFig, 'Position', [100 100 800 600])

%% Plot foot velocities
footVelocityFig = figure(footVelocityFig);
vel_axis = 2;
subplot(3,1,1)
hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.footVelocity{i}(:,1), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('X (m/s)')
title('Foot Velocities')
axis([min(state_traj.time), max(state_traj.time), -vel_axis, vel_axis])

subplot(3,1,2)
hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.footVelocity{i}(:,2), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Y (m/s)')
axis([min(state_traj.time), max(state_traj.time), -vel_axis, vel_axis])

subplot(3,1,3)
hold on;
for i = 1:num_feet
   plot(state_traj.time, state_traj.footVelocity{i}(:,3), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', lineStyle);
end
ylabel('Z (m/s)')
xlabel('Time (s)')
axis([min(state_traj.time), max(state_traj.time), -vel_axis, vel_axis])
legend('FL','BL','FR','BR','location','east')
set(footPositionFig, 'Position', [100 100 800 600])


%% Export

if nargout >0
    varargout{1} = [COMTrajFig, linearStateFig, angularStateFig, jointPositionFig, jointVelocityFig, ...
        jointEffortFig, footPositionFig, footVelocityFig];
end
