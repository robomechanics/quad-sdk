clear all; clc; close all;

% addpath('utils/')
addpath('./utils')

%% Physics parameter
parameter.physics.gravitational_constant=9.81; % Gravity

% parameter.physics.sim2real_scale_factor=(13.3-11.6620+5.75)/5.75; % Real spirit
% parameter.physics.sim2real_scale_factor=1; % Sim spirit or A1
% parameter.physics.sim2real_scale_factor=1; % Sim go2
parameter.physics.sim2real_scale_factor=1; % Sim a2

% parameter.physics.mass_body_body=parameter.physics.sim2real_scale_factor*5.75; % Only body weight of spirit
% parameter.physics.mass_body_body=parameter.physics.sim2real_scale_factor*6.0; % Only body weight of A1
% parameter.physics.mass_body_body=parameter.physics.sim2real_scale_factor*7.279; % Only body weight of Go2
% parameter.physics.mass_body_body = parameter.physics.sim2real_scale_factor*32.86; % Only body weight of Spot
% parameter.physics.mass_body_body = parameter.physics.sim2real_scale_factor*5.204; % Only body weight of Go1
parameter.physics.mass_body_body = parameter.physics.sim2real_scale_factor*19.651; % Only body weight of A2

% parameter.physics.mass_body_leg=1.478; % Each leg weight of spirit
% parameter.physics.mass_body_leg=1.935; % Each leg weight of A1
% parameter.physics.mass_body_leg=2.242; % Each leg weight of GO2
% parameter.physics.mass_body_leg = 4.385; % Each leg weight of Spot
% parameter.physics.mass_body_leg = 1.974; % Each leg weight of Go1
parameter.physics.mass_body_leg = 5.106; % Each leg weight of A2

parameter.physics.mass_body=parameter.physics.mass_body_body+...
    4*parameter.physics.mass_body_leg; % Total body weight

% parameter.physics.hip_offset=[0.2263; 0.098; 0]; % Absolute hip offset from body COM of spirit
% parameter.physics.hip_offset=[0.1805; 0.047; 0]; % Absolute hip offset from body COM of A1
% parameter.physics.hip_offset=[0.2263; 0.07; 0]; % Absolute hip offset from body COM of Go2
% parameter.physics.hip_offset = [0.29785; 0.055; 0]; % Absolute hip offset from body COM of Spot
% parameter.physics.hip_offset = [0.1881; 0.04675; 0]; % Absolute hip offset from body COM of Go1
parameter.physics.hip_offset = [0.25944; 0.075113; 0]; % Absolute hip offset from body COM of A2

% parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
%     diag([0.05; 0.1; 0.1]); % Body inertia of spirit
% parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
%     [0.0158533, -3.66e-5, -6.11e-5;
%     -3.66e-5, 0.0377999, -2.75e-5;
%     -6.11e-5, -2.75e-5, 0.0456542]; % Body inertia of A1
% parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
%     [0.02573761894851248, 0.00012166, 0.001534235398809762;
%     0.00012166, 0.1031209538931058, -3.1199999999999999e-05;
%     0.001534235398809762, -3.1199999999999999e-05, 0.1128090234445933]; % Body inertia of Go2
% parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
%     [0.13143999874591827, 0.0, 0.0;
%      0.0, 0.13143999874591827, 0.0;
%      0.0, 0.0, 0.13143999874591827]; % Body Intertia of Spot
% parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
%      [0.0168128557, -0.0002296769, -0.0002945293;
%     -0.0002296769, 0.063009565, -4.18731e-05;
%     -0.0002945293, -4.18731e-05, 0.0716547275]; % Body Intertia of Go1
parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
     [0.818570, 0.000680, -0.104550;
      0.000680, 2.082600, 0.000040;
     -0.104550, 0.000040, 2.412490]; % Full composite inertia of A2 about COM at nominal stance

% parameter.physics.inertia_body=parameter.physics.inertia_body+...
%     4*parameter.physics.mass_body_leg*...
%     diag([parameter.physics.hip_offset(2)^2+parameter.physics.hip_offset(3)^2;
%     parameter.physics.hip_offset(1)^2+parameter.physics.hip_offset(3)^2;
%     parameter.physics.hip_offset(1)^2+parameter.physics.hip_offset(2)^2]); % Robot inertia (assume leg mass concentrated at hip)

parameter.name = "a2"; % Model name
parameter.n = 12; % State dimension
parameter.m = 12; % Input dimension

%% Generate Dynamics Model
dynamicsModel(parameter);
