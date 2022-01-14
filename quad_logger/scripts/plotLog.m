close all, clear, clc

%% Setting
bagPath = {'Tail\', 'Leg\'};
fig_title = {'MPC Tail', 'Tail\'};

% Loop all folders
for i = 1:size(bagPath, 2)
    
    % Load bag list
    bagList = dir(strcat(bagPath{i}, '*.bag'));
    
    %% Loop all bags
    for j = 1:size(bagList, 1)
        
        % Load bag files
        bag = rosbag(strcat(bagPath{i}, bagList(j).name));
        
        %% Load ground truth
        groundTruth = loadGroundTruth(bag);
        
        % Load local plan
        localPlan = loadLocalPlan(bag);
        
        % Load contact info
        grf = loadGRF(bag);
        
        % Load tail control
        tailPlan = loadTailPlan(bag);
        
        % Load tail control
        tailCommand = loadTailCommand(bag);
        
        % Localize time stamp
        startTime = groundTruth.time(1);
        
        groundTruth.time = groundTruth.time - startTime;
        localPlan.time = localPlan.time - startTime;
        grf.time = grf.time - startTime;
        if (isfield(tailPlan, 'time'))
            tailPlan.time = tailPlan.time - startTime;
        end
        if (isfield(tailCommand, 'time'))
            tailCommand.time = tailCommand.time - startTime;
        end
        
        %% Record
        groundTruthRecord{j, i} = groundTruth;
        localPlanRecord{j, i} = localPlan;
        grfRecord{j, i} = grf;
        tailPlanRecord{j, i} = tailPlan;
        tailCommandRecord{j, i} = tailCommand;
        
        %%
%         groundTruth=groundTruthRecord{9};
%         localPlan=localPlanRecord{9};
%         grf=grfRecord{9};
%         tailPlan=tailPlanRecord{9};
%         tailCommand=tailCommandRecord{9};
        
%         %% Create figure
%         figure()
% %         sgtitle(fig_title{i}, 'FontSize', 24);
%         
%         % Plot angles
%         subplot(2, 1, 1);
%         plotAngle(groundTruth, tailPlan);
%         
%         % Plot contact sequence
%         subplot(2, 1, 2);
%         if (isfield(tailPlan, 'time'))
%             plotContactSequence(localPlan, grf, groundTruth, tailPlan);
%         else
%             plotContactSequence(localPlan, grf, groundTruth);
%         end
%         
% %%
%         figure()
%         hold on
%         stairs(tailCommand.time, min(max(tailCommand.effort(:, 1), -10), 10), 'DisplayName', 'Tail roll torque')
%     for i = 1:size(tailPlan.time, 1)
%         time(1) = tailPlan.time(i);
%         time(2) = tailPlan.time(i) + tailPlan.dtFirstStep(i);
%         for j = 1:10
%             time(j+2) = tailPlan.time(i) + tailPlan.dtFirstStep(i) + 0.03*j;
%         end
%         h = stairs(time, tailPlan.torqueFf{i}(1, :), 'b:');
%         h.Annotation.LegendInformation.IconDisplayStyle = 'off';
%         h = scatter(time(1), tailPlan.torqueFf{i}(1, 1), 'b*');
%         h.Annotation.LegendInformation.IconDisplayStyle = 'off';
% %         h = plot(time, tailPlan.torqueFf{i}(2, :), 'r:');
% %         h.Annotation.LegendInformation.IconDisplayStyle = 'off';
%     end
%     
% ylabel('Torque (Nm)')
% xlabel('Time (s)')
% xlim([groundTruth.time(1), groundTruth.time(end)]);
% legend('Location','southeast');
%     
%         %%
%         figure()
%         subplot(3, 1, 1);
%         hold on
%         stairs(tailCommand.time, tailCommand.effort(:, 1))
% %         stairs(tailCommand.time, min(max(tailCommand.torqueFf(:, 1)+min(max(tailCommand.fbComponent(:, 1), -10), 10), -10), 10))
%         ylim([-10, 10])
%         xlim([0, 5])
%         
%         subplot(3, 1, 2);
%         hold on
%         stairs(tailCommand.time, tailCommand.torqueFf(:, 1))
% %         stairs(tailCommand.time, tailCommand.torqueFf(:, 2))
%         ylim([-10, 10])
%         xlim([0, 5])
%         
%         subplot(3, 1, 3);
%         hold on
%         stairs(tailCommand.time, tailCommand.fbComponent(:, 1))
% %         stairs(tailCommand.time, tailCommand.torqueFf(:, 2))
%         ylim([-10, 10])
%         xlim([0, 5])
%         
%         %%
%         figure()
%         subplot(2, 1, 1);
%         hold on
%         stairs(tailCommand.time, tailCommand.posComponent(:, 1)/5)
% %         stairs(tailCommand.time, tailCommand.torqueFf(:, 2))
%         ylim([-10, 10])
%         xlim([0, 5])
%         
%         subplot(2, 1, 2);
%         hold on
%         stairs(tailCommand.time, tailCommand.velComponent(:, 1)/10)
% %         stairs(tailCommand.time, tailCommand.torqueFf(:, 2))
%         ylim([-10, 10])
%         xlim([0, 5])
    end
end

%% Analyse the orientation error
for i = 1:size(bagPath, 2)
    for j = 1:size(bagList, 1)
        maxError(j, i) = max(abs(groundTruthRecord{j, i}.axisAngleRP(:, 4)));
    end
end

mean(maxError(maxError(:, 1)<2, 1))
mean(maxError(maxError(:, 2)<2, 2))
mean(maxError(maxError(:, 3)<2, 3))
sum(maxError(:, 1)>2, 1)
sum(maxError(:, 2)>2, 1)
sum(maxError(:, 3)>2, 1)

%%
function h = stairs2(x, y, lineSpec, lineWidth)

for i=1:length(x)-1
    A(:,i) = [x(i) x(i+1)];
    B(:,i) = [y(i) y(i)];
end
h = plot(A, B, lineSpec, 'LineWidth', lineWidth);

end

%%
function groundTruth = loadGroundTruth(bag)

groundTruthData = readMessages(select(bag,'Topic','/state/ground_truth'),'DataFormat','struct');
groundTruth = struct;

groundTruth.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, groundTruthData, 'UniformOutput', 0));
groundTruth.orientationQuat = cell2mat(cellfun(@(m) ...
    [m.Body.Pose.Orientation.W, m.Body.Pose.Orientation.X, m.Body.Pose.Orientation.Y, m.Body.Pose.Orientation.Z], groundTruthData, 'UniformOutput', 0));
groundTruth.orientationRPY = fliplr(quat2eul(groundTruth.orientationQuat));
groundTruth.axisAngleRP = rotm2axang(eul2rotm([zeros(size(groundTruth.orientationRPY, 1), 1), fliplr(groundTruth.orientationRPY(:, 1:2))]));
if (~isempty(groundTruthData{1}.TailJoints.Position))
    groundTruth.tailJointPosition = cell2mat(cellfun(@(m) ...
        [m.TailJoints.Position(1), m.TailJoints.Position(2)], groundTruthData, 'UniformOutput', 0));
    groundTruth.tailJointVelocity = cell2mat(cellfun(@(m) ...
        [m.TailJoints.Velocity(1), m.TailJoints.Velocity(2)], groundTruthData, 'UniformOutput', 0));
else
    groundTruth.tailJointPosition = [];
end

end

%%
function localPlan = loadLocalPlan(bag)

localPlanData = readMessages(select(bag,'Topic','/local_plan'),'DataFormat','struct');
localPlan = struct;

localPlan.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, localPlanData, 'UniformOutput', 0));
localPlan.initialTime = cell2mat(cellfun(@(m) double(m.GlobalPlanTimestamp.Sec) + double(m.GlobalPlanTimestamp.Nsec)*1E-9, localPlanData, 'UniformOutput', 0));
localPlan.grf = struct2cell(cell2mat(cellfun(@(m) m.Grfs.Vectors, localPlanData, 'UniformOutput', 0)));
localPlan.grf = cell2mat(localPlan.grf(2:end, :, :));

%% Compute nominal contact schedule
localPlan.contactStates = zeros(size(localPlan.time, 1), 4);
localPlan.contactStates(floor(mod(localPlan.time - localPlan.initialTime, 0.36)/0.03)<6, :) = repmat([1, 0, 0, 1], sum(floor(mod(localPlan.time - localPlan.initialTime, 0.36)/0.03)<6), 1);
localPlan.contactStates(floor(mod(localPlan.time - localPlan.initialTime, 0.36)/0.03)>=6, :) = repmat([0, 1, 1, 0], sum(floor(mod(localPlan.time - localPlan.initialTime, 0.36)/0.03)>=6), 1);

end

%%
function grf = loadGRF(bag)

grfData = readMessages(select(bag,'Topic','/state/grfs'),'DataFormat','struct');
grf = struct;

grf.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, grfData, 'UniformOutput', 0));
grf.contactStates = cell2mat(cellfun(@(m) m.ContactStates', grfData, 'UniformOutput', 0));

end

%%
function tailPlan = loadTailPlan(bag)

tailPlanData = readMessages(select(bag,'Topic','/control/tail_plan'),'DataFormat','struct');
tailPlan = struct;

if (~isempty(tailPlanData))
    tailPlan.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, tailPlanData, 'UniformOutput', 0));
    tailPlan.missContact = cell2mat(cellfun(@(m) ...
        m.MissContact', tailPlanData, 'UniformOutput', 0));
    tailPlan.plan = cellfun(@(m) ...
        [m(1:2:end).PosSetpoint; m(2:2:end).PosSetpoint], ...
        cellfun(@(m) ...
        [m(:).MotorCommands], ...
        cellfun(@(m) ...
        m.LegCommands, ...
        tailPlanData, 'UniformOutput', 0), 'UniformOutput', 0), 'UniformOutput', 0);
    tailPlan.torqueFf = cellfun(@(m) ...
        [m(1:2:end).TorqueFf; m(2:2:end).TorqueFf], ...
        cellfun(@(m) ...
        [m(:).MotorCommands], ...
        cellfun(@(m) ...
        m.LegCommands, ...
        tailPlanData, 'UniformOutput', 0), 'UniformOutput', 0), 'UniformOutput', 0);
end

end

%%
function tailCommand = loadTailCommand(bag)

tailCommandData = readMessages(select(bag,'Topic','/control/tail_command'),'DataFormat','struct');
tailCommand = struct;

if (~isempty(tailCommandData))
    tailCommand.time = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1E-9, tailCommandData, 'UniformOutput', 0));
    tailCommand.posSetpoint = cell2mat(cellfun(@(m) ...
        [m.MotorCommands(1).PosSetpoint, m.MotorCommands(2).PosSetpoint], tailCommandData, 'UniformOutput', 0));
    tailCommand.velSetpoint = cell2mat(cellfun(@(m) ...
        [m.MotorCommands(1).VelSetpoint, m.MotorCommands(2).VelSetpoint], tailCommandData, 'UniformOutput', 0));
    tailCommand.kp = cell2mat(cellfun(@(m) ...
        [m.MotorCommands(1).Kp, m.MotorCommands(2).Kp], tailCommandData, 'UniformOutput', 0));
    tailCommand.kd = cell2mat(cellfun(@(m) ...
        [m.MotorCommands(1).Kd, m.MotorCommands(2).Kd], tailCommandData, 'UniformOutput', 0));
    tailCommand.posComponent = cell2mat(cellfun(@(m) ...
        [m.MotorCommands(1).PosComponent, m.MotorCommands(2).PosComponent], tailCommandData, 'UniformOutput', 0));
    tailCommand.velComponent = cell2mat(cellfun(@(m) ...
        [m.MotorCommands(1).VelComponent, m.MotorCommands(2).VelComponent], tailCommandData, 'UniformOutput', 0));
    tailCommand.torqueFf = cell2mat(cellfun(@(m) ...
        [m.MotorCommands(1).TorqueFf, m.MotorCommands(2).TorqueFf], tailCommandData, 'UniformOutput', 0));
    tailCommand.fbComponent = cell2mat(cellfun(@(m) ...
        [m.MotorCommands(1).FbComponent, m.MotorCommands(2).FbComponent], tailCommandData, 'UniformOutput', 0));
    tailCommand.effort = cell2mat(cellfun(@(m) ...
        [m.MotorCommands(1).Effort, m.MotorCommands(2).Effort], tailCommandData, 'UniformOutput', 0));
end

end

%%
function tailTorque = computeTailTorque(groundTruth, tailCommand)

for i = 1:size(tailCommand.time, 1)
    [~, idx] = min(abs(groundTruth.time - tailCommand.time(i)));
    
    tailTorque(i, 1) = min(max(tailCommand.torqueFfRoll(i) + ...
        tailCommand.kpRoll(i) * (-groundTruth.tailJointPosition(idx, 1) + tailCommand.posRoll(i)) + ...
        tailCommand.kdRoll(i) * (-groundTruth.tailJointVelocity(idx, 1) + tailCommand.velRoll(i)), -10), 10);
    tailTorque(i, 2) =  min(max(tailCommand.torqueFfPitch(i) + ...
        tailCommand.kpRoll(i) * (-groundTruth.tailJointPosition(idx, 2) + tailCommand.posPitch(i)) + ...
        tailCommand.kdRoll(i) * (-groundTruth.tailJointVelocity(idx, 2) + tailCommand.velPitch(i)), -10), 10);
end

end

%%
function plotAngle(groundTruth, varargin)

if (~isempty(varargin))
    tailPlan = varargin{1};
end

ax = gca;
hold on
ylabel('Body orientation (deg)')
plot(groundTruth.time, rad2deg(groundTruth.orientationRPY(:, 1)), 'b-', 'DisplayName', 'Body roll');
plot(groundTruth.time, rad2deg(groundTruth.orientationRPY(:, 2)), 'r-', 'DisplayName', 'Body pitch');
ax.YAxis(1).Color = 'k';
if (~isempty(groundTruth.tailJointPosition))
    yyaxis right
    ylabel('Tail orientation (deg)')
    plot(groundTruth.time, rad2deg(groundTruth.tailJointPosition(:, 1)), 'b--', 'DisplayName', 'Tail roll');
    plot(groundTruth.time, rad2deg(groundTruth.tailJointPosition(:, 2)), 'r--', 'DisplayName', 'Tail pitch');
    ax.YAxis(2).Color = 'k';
end

if (~isempty(varargin))
    for i = 1:size(tailPlan.time, 1)
        time(1) = tailPlan.time(i);
        for j = 1:11
            time(j+2) = tailPlan.time(i) + 0.03*j;
        end
        h = plot(time, rad2deg(tailPlan.plan{i}(1, :)), 'b:');
        h.Annotation.LegendInformation.IconDisplayStyle = 'off';
        h = scatter(time(1), rad2deg(tailPlan.plan{i}(1, 1)), 'b*');
        h.Annotation.LegendInformation.IconDisplayStyle = 'off';
        h = plot(time, rad2deg(tailPlan.plan{i}(2, :)), 'r:');
        h.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
end

xlabel('Time (s)')
xlim([groundTruth.time(1), groundTruth.time(end)]);
legend('Location','southeast');

end

%%
function plotContactSequence(localPlan, grf, groundTruth, varargin)

hold on

if (~isempty(varargin))
    tailPlan = varargin{1};
end

if (~isempty(varargin))
    stairs2(tailPlan.time, 4*tailPlan.missContact(:, 1), 'g-', 12);
    stairs2(tailPlan.time, 3*tailPlan.missContact(:, 2), 'g-', 12);
    stairs2(tailPlan.time, 2*tailPlan.missContact(:, 3), 'g-', 12);
    handleMissContact = stairs2(tailPlan.time, 1*tailPlan.missContact(:, 4), 'g-', 12);
end

stairs2(localPlan.time, 4*localPlan.contactStates(:, 1), 'r-', 8);
stairs2(localPlan.time, 3*localPlan.contactStates(:, 2), 'r-', 8);
stairs2(localPlan.time, 2*localPlan.contactStates(:, 3), 'r-', 8);
handleNominal = stairs2(localPlan.time, 1*localPlan.contactStates(:, 4), 'r-', 8);

stairs2(grf.time, 4*grf.contactStates(:, 1), 'b-', 4);
stairs2(grf.time, 3*grf.contactStates(:, 2), 'b-', 4);
stairs2(grf.time, 2*grf.contactStates(:, 3), 'b-', 4);
handleGroundTruth = stairs2(grf.time, 1*grf.contactStates(:, 4), 'b-', 4);

xlim([groundTruth.time(1), groundTruth.time(end)]);
xlabel('Time (s)')
ylim([0.5, 4.5]);
yticks([1, 2, 3, 4]);
yticklabels({'RR', 'FR', 'RL', 'FL'});
ylabel('Contact sequence')
if (~isempty(varargin))
    legend([handleNominal(1), handleGroundTruth(1), handleMissContact(1)], {'Nominal', 'Ground truth', 'MissContactEst'}, 'Location','southeast');
else
    legend([handleNominal(1), handleGroundTruth(1)], {'Nominal', 'Ground truth'}, 'Location','southeast');
end

end