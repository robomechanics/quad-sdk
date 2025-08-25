function [data, info] = robotPlanDiagnostics
%RobotPlanDiagnostics gives an empty data for quad_msgs/RobotPlanDiagnostics
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'quad_msgs/RobotPlanDiagnostics';
[data.compute_time, info.compute_time] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.cost, info.cost] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.iterations, info.iterations] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.horizon_length, info.horizon_length] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.complexity_schedule, info.complexity_schedule] = ros.internal.ros2.messages.ros2.default_type('uint32',NaN,0);
[data.element_times, info.element_times] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
info.MessageType = 'quad_msgs/RobotPlanDiagnostics';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'compute_time';
info.MatPath{2} = 'cost';
info.MatPath{3} = 'iterations';
info.MatPath{4} = 'horizon_length';
info.MatPath{5} = 'complexity_schedule';
info.MatPath{6} = 'element_times';
