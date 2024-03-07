function [torqueInputs,statesMatrix, scheduleMatrix] = beamStructToVectors(dataStruct)

    % loading data struct save for testing. 
    % This can be commented once it's tested and returns vectors

    % dataStruct = load("dataStructSample_1.mat");

    %% Getting Inputs vector

    torqueInputs = [dataStruct.jointTorques.time,dataStruct.jointTorques.torques]';
    % 13xn -> [time, 12 joint torques]

    n_time = length(dataStruct.jointTorques.time);

    %% Extracting 37Xn matrix with:
    % [Body Pos, B quat, joint ang, Body vel, Body ang vel, joint Vel]

    
    states = [dataStruct.stateGroundTruth.time, dataStruct.stateGroundTruth.position,...
                    dataStruct.stateGroundTruth.orientationQuat, dataStruct.stateGroundTruth.jointPosition,...
                    dataStruct.stateGroundTruth.velocity, dataStruct.stateGroundTruth.angularVelocity,...
                    dataStruct.stateGroundTruth.jointVelocity]';


    statesMatrix = [];

    j = 1:2:2*n_time;

    statesMatrix = states(:,j);
    
    %Checking size is the same
    if(length(statesMatrix(1,:)) > n_time)

        statesMatrix(:,end) = [];

    end

   %% Getting Contact Schedule

   schedule = [dataStruct.stateGRFs.time, dataStruct.stateGRFs.contactStates{1}(:,1),...
                dataStruct.stateGRFs.contactStates{2}(:,1), dataStruct.stateGRFs.contactStates{3}(:,1),...
                dataStruct.stateGRFs.contactStates{4}(:,1)]';
    

   scheduleMatrix = schedule(:,j);

   %Checking size is the same
    if(length(scheduleMatrix(1,:)) > n_time)

        scheduleMatrix(:,end) = [];

    end


    %% Saving vectors as .CSV
    
    % Uncomment this to save the data as CSV files with the sample names
    % csvwrite('torqueInputsSample', torqueInputs);
    % csvwrite('statesMatrixSample', statesMatrix);
    % csvwrite('contactMatrix', scheduleMatrix);


end