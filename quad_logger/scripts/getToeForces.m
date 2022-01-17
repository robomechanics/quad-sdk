function toeForces = getToeForces(robot, state)

% homeConfig = homeConfiguration(robot);
% jointOrder = str2double({homeConfig.JointName})+1;
% 
% orderedJointEffort = state.jointEffort(:,jointOrder);
% ktau = [0.546; 0.546; 1.092]; % N m/A estimate for Ab/Ad, Hip, Knee
% 
% toeForces = zeros(length(state.time),12);
% for ii = 1:length(state.time)
%     config = homeConfig;
%     for jj = 1:12 % todo remove magic number
%         config(jj).JointPosition = state.jointPosition(ii,str2double(config(jj).JointName)+1);
%     end
%     geoJac = cell(1,4);
%     for jj = 1:4
%         geoJac{jj} = geometricJacobian(robot,config,robot.BodyNames{jj*4});
%         toeForces(ii,(jj-1)*3 + (1:3)) = geoJac{jj}(4:6,(jj-1)*3 + (1:3)).'\(-ktau.*orderedJointEffort(ii,(jj-1)*3 + (1:3)).');
%     end
% end