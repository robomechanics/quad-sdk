if ispc
    addpath(['~/Documents/MATLAB/RMLscripts/MATLAB']);
else
    addpath([fullfile(getenv('HOME'), 'Documents', 'MATLAB', 'RMLscripts', 'MATLAB')]);
end

set(0, 'DefaultAxesFontSize', 24);
set(0, 'DefaultLineMarkerSize', 10);
set(0, 'DefaultLineLineWidth', 2);
set(0,'defaultfigurecolor',[1 1 1]);
set(0,'DefaultAxesXGrid','off','DefaultAxesYGrid','off')
set(0,'defaulttextinterpreter','latex');
set(0, 'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');
set(0,'defaultfigureposition',[400 250 900 750])
disp('Using RML Scripts');
% set(0,'DefaultAxesPosition', [0.1, 0.1, 0.9, 0.9]) % [0.1300    0.1100  0.7750    0.8150] is MATLAB default
