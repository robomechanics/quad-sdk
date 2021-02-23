function varargout = plotState(state)

% Plot spatial trajectory
COMTrajFig = figure;
plot3(state.position(:,1), state.position(:,2), state.position(:,3), 'Color', cmuColor('red-web'));
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('COM Trajectory')
axis equal

% Plot linear state data
linearStateFig = figure;
subplot(2,1,1)
plot(state.time, state.position);
ylabel('Position (m)')
title('Linear States')
legend('X','Y','Z','location','best')
axis tight

subplot(2,1,2)
plot(state.time, state.velocity);
ylabel('Velocity (m/s)')
xlabel('Time (s)')
axis tight
set(linearStateFig, 'Position', [100 100 800 600])

% Plot angular state data
angularStateFig = figure;
subplot(2,1,1)
plot(state.time, state.orientationRPY);
ylabel('Ang. Pos. (rad)')
title('Angular States')
legend('Roll','Pitch','Yaw','location','best')
axis([min(state.time), max(state.time), -pi, pi])

subplot(2,1,2)
plot(state.time, state.angularVelocity);
ylabel('Ang. Vel. (rad/s)')
xlabel('Time (s)')
axis tight
set(angularStateFig, 'Position', [100 100 800 600])

% Specify indices for leg joints
abIndex = [1:3:12];
hipIndex = [2:3:12];
kneeIndex = [3:3:12];

% Plot joint positions
jointPositionFig = figure;
subplot(3,1,1)
plot(state.time, state.jointPosition(:,abIndex));
ylabel('Ab/Ad (rad)')
title('Joint Angles')
axis([min(state.time), max(state.time), -1, 1])

subplot(3,1,2)
plot(state.time, state.jointPosition(:,hipIndex));
ylabel('Hip (rad)')
legend('FL','BL','FR','BR','location','best')
axis([min(state.time), max(state.time), -pi/2, pi])

subplot(3,1,3)
plot(state.time, state.jointPosition(:,kneeIndex));
ylabel('Knee (rad)')
xlabel('Time (s)')
axis([min(state.time), max(state.time), 0, pi])
set(jointPositionFig, 'Position', [100 100 800 600])

% Plot joint velocities
jointVelocityFig = figure;
subplot(3,1,1)
plot(state.time, state.jointVelocity(:,abIndex));
ylabel('Ab/Ad (rad/s)')
title('Joint Velocities')
axis([min(state.time), max(state.time), -38, 38])

subplot(3,1,2)
plot(state.time, state.jointVelocity(:,hipIndex));
ylabel('Hip (rad/s)')
legend('FL','BL','FR','BR','location','best')
axis([min(state.time), max(state.time), -38, 38])

subplot(3,1,3)
plot(state.time, state.jointVelocity(:,kneeIndex));
ylabel('Knee (rad/s)')
xlabel('Time (s)')
axis([min(state.time), max(state.time), -25, 25])
set(jointVelocityFig, 'Position', [100 100 800 600])

% Plot estimate joint effort
jointEffortFig = figure;
subplot(3,1,1)
plot(state.time, state.jointEffort(:,abIndex));
ylabel('Ab/Ad (A)')
title('Joint Effort')
axis([min(state.time), max(state.time), -21, 21])

subplot(3,1,2)
plot(state.time, state.jointEffort(:,hipIndex));
ylabel('Hip (A)')
legend('FL','BL','FR','BR','location','best')
axis([min(state.time), max(state.time), -21, 21])

subplot(3,1,3)
plot(state.time, state.jointEffort(:,kneeIndex));
ylabel('Knee (A)')
xlabel('Time (s)')
axis([min(state.time), max(state.time), -32, 32])
set(jointEffortFig, 'Position', [100 100 800 600])

figArray = [COMTrajFig, linearStateFig, angularStateFig, jointPositionFig, jointVelocityFig, jointEffortFig];

for i = 1:nargout
    varargout{i} = figArray(i);
end