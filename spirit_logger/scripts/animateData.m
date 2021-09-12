function animateData(robot, state, fileName, bRecord)

homeConfig = homeConfiguration(robot);

timer = tic;
timeScale = 1;
frameRate = 30;
tLast = -1/frameRate;

if bRecord
    vid = VideoWriter([fileName, '_animation']);

    vid.FrameRate = frameRate;
    vid.Quality = 98;
    open(vid);
end

joint_indices = [2,3,5,6,8,9,11,12,1,4,7,10];
urdf_joint_position = state.jointPosition(:,joint_indices);

h_ani = figure(101);
clf
ax = show(robot,homeConfig,'frames','off');
config = homeConfig;
for jj = 1:12 % todo remove magic number
    config(jj).JointPosition = urdf_joint_position(1,str2double(config(jj).JointName)+1);
end
show(robot,config,'frames','off','parent',ax);
ax.NextPlot = 'replaceChildren'; % This makes show slightly faster

set(gca,'xlim',[-0.5,0.5],'ylim',[-0.4,0.4],'zlim',[-0.5,0.3])

%jointPosition(:,9:12) = 0.5; %+: feet left
%jointPosition(:,1:2:7) = 1.5; %+: down
%jointPosition(:,2:2:8) = 1.5; %+: down


for ii = 1:length(state.time)
    if state.time(ii) - tLast < 1/(timeScale*frameRate)
        continue
    end
    
    config = homeConfig;
    for jj = 1:12 % todo remove magic number
        config(jj).JointPosition = urdf_joint_position(ii,str2double(config(jj).JointName)+1);
    end
    figure(h_ani)
    show(robot,config,'frames','off','parent',ax,'preserveplot',true);
    title(['Index ',num2str(ii),', Time ', num2str(state.time(ii),'%06.3f'), ' s'])
    drawnow
    
    %pause(timeScale*(jointTime(ii)-tLast) - toc(timer))
    timer = tic;
    tLast = state.time(ii);
    
    if bRecord
        frame = getframe(h_ani);
        writeVideo(vid, frame.cdata);
    end
end

if bRecord
    close(vid)
end