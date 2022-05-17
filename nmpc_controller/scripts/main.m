clear all; clc; close all;

addpath('./utils')

%% Physics parameter
parameter.physics.gravitational_constant=9.81; % Gravity

parameter.physics.sim2real_scale_factor=(13.3-11.6620+5.75)/5.75; % Real spirit
% parameter.physics.sim2real_scale_factor=1; % Sim spirit or A1

parameter.physics.mass_body_body=parameter.physics.sim2real_scale_factor*5.75; % Only body weight of spirit
% parameter.physics.mass_body_body=parameter.physics.sim2real_scale_factor*6.0; % Only body weight of A1

parameter.physics.mass_body_leg=1.478; % Each leg weight of spirit
% parameter.physics.mass_body_leg=1.935; % Each leg weight of A1

parameter.physics.mass_body=parameter.physics.mass_body_body+...
    4*parameter.physics.mass_body_leg; % Total body weight

parameter.physics.hip_offset=[0.2263; 0.098; 0]; % Absolute hip offset from body COM of spirit
% parameter.physics.hip_offset=[0.1805; 0.047; 0]; % Absolute hip offset from body COM of A1

parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
    diag([0.05; 0.1; 0.1]); % Body inertia of spirit
% parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
%     [0.0158533, -3.66e-5, -6.11e-5;
%     -3.66e-5, 0.0377999, -2.75e-5;
%     -6.11e-5, -2.75e-5, 0.0456542]; % Body inertia of A1

parameter.physics.inertia_body=parameter.physics.inertia_body+...
    4*parameter.physics.mass_body_leg*...
    diag([parameter.physics.hip_offset(2)^2+parameter.physics.hip_offset(3)^2;
    parameter.physics.hip_offset(1)^2+parameter.physics.hip_offset(3)^2;
    parameter.physics.hip_offset(1)^2+parameter.physics.hip_offset(2)^2]); % Robot inertia (assume leg mass concentrated at hip)

parameter.name = "spirit"; % Model name
parameter.n = 12; % State dimension
parameter.m = 12; % Input dimension

%% Generate Dynamics Model
dynamicsModel(parameter);
