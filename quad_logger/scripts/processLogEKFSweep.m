function processLogEKFSweep(varargin)
% processLog Process a quad data log file to generate figures
%   processLog uses the default 'quad_log_current' file name to
%   yield a data structure containing select topic data. If this bag does
%   not exist, the user can select the bag via a UI.
%
%   processLog(FILENAME) uses the data in a bag with the specified
%   file name, looking in '../bags/'.

%% Prepare the environment
close all;clc;

if ~exist(directoryPath, 'dir')
    error(['Directory not found)')
end

files = dir(fullfile(directoryPath, '*.bag'));

for i = 1:length(files)

    P = [0.5 1 2 3 4 5 6 7 8 9 10]
    na = [0.0001 0.0005 0.001 0.005 0.01 0.05 0.1 0.5 1 5 10]
    nf = [0.0001 0.0005 0.001 0.005 0.01 0.05 0.1 0.5 1 5 10]
    
    trialName = ''; % Set to '' to load via GUI
    namespace = 'robot_1'; % Namespace of the robot bag, set to '' if none

%% Set parameters

    bSave = true;                       % Save the figures/videos
    bAnimate = false;                   % Animate the trajectory (no translation)
    bTitles = true;                     % Turn on figure titles
    bPlotLocalPlanInfo = true;          % Turn on to plot local plan information
    tWindowStates = [];                 % Specify time window for state (use [] for no clipping)
    tWindowControl = [];                % Specify time window for control (use [] for no clipping)
    tWindowLocalPlan = [];              % Specify time window for local plan (use [] for no clipping)

    %% Create a directory for saving plots
    if bSave
        timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
        plotDir = fullfile('..', 'plots', [trialName '_' timestamp]);
        if ~exist(plotDir, 'dir')
            mkdir(plotDir);
        end
    end
    
    %% Load the data
    
    % Load the data
    [data, trialName] = parseQuadBag(trialName, namespace);
    stateEstimate = data.stateEstimate;
    stateGroundTruth = data.stateGroundTruth;
    
    figure(1)
    t = tiledlayout(3,1);
    nexttile
    hold on
    plot(stateGroundTruth.time, stateGroundTruth.position(:,1), 'b-','LineWidth',2)
    plot(stateEstimate.time, stateEstimate.position(:,1), 'r-', 'LineWidth',2)
    title("X position")
    legend("Ground Truth", "State Estimate")
    grid on
    nexttile
    hold on
    plot(stateGroundTruth.time, stateGroundTruth.position(:,2), 'b', 'LineWidth',2 )
    title("Y position")
    legend("Ground Truth", "State Estimate")
    grid on
    nexttile
    hold on
    plot(stateGroundTruth.time, stateGroundTruth.position(:,3), 'b-','LineWidth',2)
    plot(stateEstimate.time, stateEstimate.position(:,3), 'r-','LineWidth',2)
    title("Z position")
    legend("Ground Truth", "State Estimate")
    grid on
    xlabel(t, "Time (s)")
    ylabel(t, 'Body position (m/s)')
    
    if bSave
        saveas(gcf, fullfile(plotDir, 'Position.png'));
    end
    
    figure(2)
    t = tiledlayout(3,1);
    nexttile
    hold on
    plot(stateGroundTruth.time, stateGroundTruth.velocity(:,1), 'b-','LineWidth',2)
    plot(stateEstimate.time, stateEstimate.velocity(:,1), 'r-', 'LineWidth',2)
    xlim([1 12])
    title("X Velocity")
    legend("Ground Truth", "State Estimate")
    grid on
    nexttile
    hold on
    plot(stateGroundTruth.time, stateGroundTruth.velocity(:,2), 'b', 'LineWidth',2 )
    plot(stateEstimate.time, stateEstimate.velocity(:,2), 'r-' , 'LineWidth',2)
    xlim([1 12])
    title("Y Velocity")
    legend("Ground Truth", "State Estimate")
    grid on
    nexttile
    hold on
    plot(stateGroundTruth.time, stateGroundTruth.velocity(:,3), 'b-','LineWidth',2)
    plot(stateEstimate.time, stateEstimate.velocity(:,3), 'r-','LineWidth',2)
    xlim([1 12])
    title("Z Velocity")
    legend("Ground Truth", "State Estimate")
    grid on
    xlabel(t, "Time (s)")
    ylabel(t, 'Body Velocity (m/s)')
    
    if bSave
        saveas(gcf, fullfile(plotDir, 'Velocity.png'));
    end

end
