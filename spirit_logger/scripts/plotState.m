function varargout = plotState(state_traj)

footColorVector = {[242,169,0]/255, [0,132,61]/255, [166,25,46]/255, [0,45,114]/255};
footStyleVector = {'-','-','--','--'};
jointColorVector = footColorVector;
jointStyleVector = footStyleVector;

%% Plot spatial trajectory
COMTrajFig = figure;
plot3(state_traj.position(:,1), state_traj.position(:,2), state_traj.position(:,3), ...
    'Color', cmuColor('red-web'), 'LineWidth', 4);
hold on
for i = 1:4
   plot3(state_traj.footPosition{i}(:,1), state_traj.footPosition{i}(:,2), ...
       state_traj.footPosition{i}(:,3), 'Color', footColorVector{i}, 'LineWidth', 4);
end
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Body and Foot Trajectories')
axis equal

%% Plot linear state data
linearStateFig = figure;
subplot(2,1,1)
plot(state_traj.time, state_traj.position);
ylabel('Position (m)')
title('Linear States')
legend('X','Y','Z','location','best')
axis tight

subplot(2,1,2)
plot(state_traj.time, state_traj.velocity);
ylabel('Velocity (m/s)')
xlabel('Time (s)')
axis tight
set(linearStateFig, 'Position', [100 100 800 600])

%% Plot angular state data
angularStateFig = figure;
subplot(2,1,1)
plot(state_traj.time, state_traj.orientationRPY);
ylabel('Ang. Pos. (rad)')
title('Angular States')
legend('Roll','Pitch','Yaw','location','best')
axis([min(state_traj.time), max(state_traj.time), -pi, pi])

subplot(2,1,2)
plot(state_traj.time, state_traj.angularVelocity);
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
jointPositionFig = figure;
subplot(3,1,1); hold on;
for i = 1:4
   plot(state_traj.time, state_traj.jointPosition(:,abIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', jointStyleVector{i});
end
ylabel('Ab/Ad (rad)')
title('Joint Angles')
axis([min(state_traj.time), max(state_traj.time), -1, 1])

subplot(3,1,2); hold on;
for i = 1:4
   plot(state_traj.time, state_traj.jointPosition(:,hipIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', jointStyleVector{i});
end
ylabel('Hip (rad)')
legend('FL','BL','FR','BR','location','best')
axis([min(state_traj.time), max(state_traj.time), -pi/2, pi])

subplot(3,1,3); hold on;
for i = 1:4
   plot(state_traj.time, state_traj.jointPosition(:,kneeIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', jointStyleVector{i});
end
ylabel('Knee (rad)')
xlabel('Time (s)')
axis([min(state_traj.time), max(state_traj.time), 0, pi])
set(jointPositionFig, 'Position', [100 100 800 600])

%% Plot joint velocities
jointVelocityFig = figure;
subplot(3,1,1); hold on;
for i = 1:4
   plot(state_traj.time, state_traj.jointVelocity(:,abIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', jointStyleVector{i});
end
ylabel('Ab/Ad (rad/s)')
title('Joint Velocities')
axis([min(state_traj.time), max(state_traj.time), -38, 38])

subplot(3,1,2); hold on;
for i = 1:4
   plot(state_traj.time, state_traj.jointVelocity(:,hipIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', jointStyleVector{i});
end
ylabel('Hip (rad/s)')
legend('FL','BL','FR','BR','location','best')
axis([min(state_traj.time), max(state_traj.time), -38, 38])

subplot(3,1,3); hold on;
for i = 1:4
   plot(state_traj.time, state_traj.jointVelocity(:,kneeIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', jointStyleVector{i});
end
ylabel('Knee (rad/s)')
xlabel('Time (s)')
axis([min(state_traj.time), max(state_traj.time), -25, 25])
set(jointVelocityFig, 'Position', [100 100 800 600])

%% Plot estimate joint effort
jointEffortFig = figure;
subplot(3,1,1); hold on;
for i = 1:4
   plot(state_traj.time, state_traj.jointEffort(:,abIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', jointStyleVector{i});
end
ylabel('Ab/Ad (A)')
title('Joint Effort')
axis([min(state_traj.time), max(state_traj.time), -21, 21])

subplot(3,1,2); hold on;
for i = 1:4
   plot(state_traj.time, state_traj.jointEffort(:,hipIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', jointStyleVector{i});
end
ylabel('Hip (A)')
legend('FL','BL','FR','BR','location','best')
axis([min(state_traj.time), max(state_traj.time), -21, 21])

subplot(3,1,3); hold on;
for i = 1:4
   plot(state_traj.time, state_traj.jointEffort(:,kneeIndex(i)), ...
       'Color', jointColorVector{i}, 'LineWidth', 2, 'LineStyle', jointStyleVector{i});
end
ylabel('Knee (A)')
xlabel('Time (s)')
axis([min(state_traj.time), max(state_traj.time), -32, 32])
set(jointEffortFig, 'Position', [100 100 800 600])


%% Plot foot positions
footPositionFig = figure;
subplot(3,1,1)
hold on;
for i = 1:4
   plot(state_traj.time, state_traj.footPosition{i}(:,1), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', footStyleVector{i});
end
ylabel('X (m)')
title('Foot Positions')
axis([min(state_traj.time), max(state_traj.time), -1, 4])

subplot(3,1,2)
hold on;
for i = 1:4
   plot(state_traj.time, state_traj.footPosition{i}(:,2), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', footStyleVector{i});
end
ylabel('Y (m)')
axis([min(state_traj.time), max(state_traj.time), -2 2])

subplot(3,1,3)
hold on;
for i = 1:4
   plot(state_traj.time, state_traj.footPosition{i}(:,3), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', footStyleVector{i});
end
ylabel('Z (m)')
xlabel('Time (s)')
axis([min(state_traj.time), max(state_traj.time), 0, 0.3])
legend('FL','BL','FR','BR','location','east')
set(footPositionFig, 'Position', [100 100 800 600])

%% Plot foot velocities
footVelocityFig = figure;
vel_axis = 2;
subplot(3,1,1)
hold on;
for i = 1:4
   plot(state_traj.time, state_traj.footVelocity{i}(:,1), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', footStyleVector{i});
end
ylabel('X (m/s)')
title('Foot Velocities')
axis([min(state_traj.time), max(state_traj.time), -vel_axis, vel_axis])

subplot(3,1,2)
hold on;
for i = 1:4
   plot(state_traj.time, state_traj.footVelocity{i}(:,2), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', footStyleVector{i});
end
ylabel('Y (m/s)')
axis([min(state_traj.time), max(state_traj.time), -vel_axis, vel_axis])

subplot(3,1,3)
hold on;
for i = 1:4
   plot(state_traj.time, state_traj.footVelocity{i}(:,3), ...
       'Color', footColorVector{i}, 'LineWidth', 2, 'LineStyle', footStyleVector{i});
end
ylabel('Z (m/s)')
xlabel('Time (s)')
axis([min(state_traj.time), max(state_traj.time), -vel_axis, vel_axis])
legend('FL','BL','FR','BR','location','east')
set(footPositionFig, 'Position', [100 100 800 600])


%% Export

figArray = [COMTrajFig, linearStateFig, angularStateFig, jointPositionFig, jointVelocityFig, jointEffortFig];

for i = 1:nargout
    varargout{i} = figArray(i);
end