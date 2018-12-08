clear all;
close all;
sim = 0;

if sim
    [data, robotPose, landmarkMap] = simulator();
    figure(1); hold on;
    title('Simulator Plot');
    % Draw simulated landmarks
    LsG  = line(...
    'linestyle','none',...
    'marker','+',...
    'color','r',...
    'xdata',landmarkMap(:,1),...
    'ydata',landmarkMap(:,2));
    % Draw simulated robot pose
    RsG = line(...
    'linestyle','-',...
    'marker','none',...
    'color','b',...
    'xdata',robotPose(:,1),...
    'ydata',robotPose(:,2));
    figure(1); hold off;

else
    % Draw groundtruth
    load('data/dataSala1.mat');
end

%% Static Variables
nTimestamps = length(data);
plotx = zeros(nTimestamps,1);
ploty = zeros(nTimestamps,1);
ReG = zeros(1, nTimestamps);
nLandmarksTotal = 15;
wheeldistance = 0.21;
rNoise = [0.05; 0.0175; 0.05; 0.0175];
%rNoise = [0.2; 0.1; 0.2; 0.1];
Rn = diag(rNoise.^2);   %probably wrong
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Dynamic Variables
stateMean = zeros(3, 1);
%stateMean(2) = 0.5;
stateCov = zeros(3,3);
%StateCov(2,2) = 0.1;
%TODO --> Avoid overwrite before matching step

q = [0.1;0.03];
lQ = diag(q.^2); % Landmark noise

Jr = zeros(2,3);
Jl = zeros(2);

last_odom = zeros(1, 4);
last_time = data(1).time;

nLandmarksSeen = 0;
nLandmarksCurrent = 0;
landmarkRaw = zeros(3, 1);
landmarkXY = zeros(2, 1);
landmarkList = -ones(nLandmarksTotal, 1);

runtime=zeros(nTimestamps,1);
error=zeros(nTimestamps,1);
errorc=zeros(nTimestamps,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%   3. Graphics
mapFig = figure(17);
cla;    %clear axis
axis([-10 10 -10 10])
axis square
if sim
    WG = line('parent', gca, ...    %landmarks truth
        'linestyle', 'none', ...
        'marker', 'o', ...
        'color', 'r', ...
        'xdata', landmarkMap(:,1), ...
        'ydata', landmarkMap(:,2));
    RG = line('parent',gca, ...     %simulator robot
        'marker', '*', ...
        'color', 'r', ...
        'xdata', robotPose(1,1), ...
        'ydata', robotPose(1,2));

    rG = line('parent',gca, ...     %estimator robot
    'marker', '*', ...
    'color', 'b', ...
    'xdata', robotPose(1,1), ...
    'ydata', robotPose(1,2));
end


relipG = line('parent',gca, ...     %estimator robot elipses
    'color', 'b', ...
    'xdata', [], ...
    'ydata', []);


if sim
    lG = zeros(1,size(landmarkMap,1));        %this is for the landmarks
else
    lG = zeros(1,nLandmarksTotal);
end
for i = 1:nLandmarksTotal
    lG(i)= line('parent',gca, ...     %landmark estimator
        'linestyle', 'none', ...
        'marker', '+', ...
        'color', 'b', ...
        'xdata', [], ...
        'ydata', []);
end
if sim
    eG = zeros(1,size(landmarkMap,1));        %this is for the landmarks elipses
else
    eG = zeros(1,nLandmarksTotal);
end
for i = 1:numel(eG)
    eG(i) = line(...
        'parent', gca, ...
        'color', 'g', ...
        'xdata', [], ...
        'ydata', []);
end

estPose = size(nTimestamps,2);
index = 0;
for t = 1:nTimestamps
     %% Prediction step
    if sim == 1
        noise = rNoise .* randn(4,1);
    else
        noise = zeros(4,1);
    end

    %%%
%    couve = ...
%        movement_model(stateMean(1:3)', [last_odom last_time], noise, data(t).time, ...
%        wheeldistance, nLandmarksCurrent, Rn);
%    errorc(t)=norm(abs(couve(1:2))-abs(robotPose(floor(t/2)+rem(t,2),1:2)))^2;
%    if errorc(t)>2
%        disp('here couve')
%    end
    %%%%

    tic
    [stateMean(1:3), rJacob, nJacob] = ...
        movement_model(stateMean(1:3)', [last_odom last_time], noise, data(t).time, ...
        wheeldistance, nLandmarksCurrent, Rn);
    last_time = data(t).time;

    plotx(t,1) = stateMean(1);
    ploty(t,1) = stateMean(2);
    stateCov = rJacob*stateCov*rJacob' + nJacob;

	%% Correction step
    if data(t).option == 1
        nLandmarksSeen = data(t).landmarksSeen;
        if nLandmarksSeen > 0
            for i = 1:nLandmarksSeen
                landmarkRaw = data(t).landmark(i, :); % Get [landmarkID, landmarkDist, landmarkAngle]
                %lQ = [var(landmarkRaw(2)) 0; 0 var(landmarkRaw(3))];
                if ~ismember(landmarkRaw(1), landmarkList) % If never seen before, add new Landmark
                    [landmarkXY, Jr, Jl] = ...
                        new_landmark([landmarkRaw(3) landmarkRaw(2)], stateMean(1:3)); % Get [landmarkX, landmarkY]
                    landmarkList(nLandmarksCurrent+1) = landmarkRaw(1); % Add ID to list of landmarks
                    stateMean(3+nLandmarksCurrent*2+1) = landmarkXY(1); % Add X to state mean
                    stateMean(3+nLandmarksCurrent*2+2) = landmarkXY(2); % Add Y to state mean
                    P_lx = Jr*stateCov(1:3,:);
                    P_ll = Jr*stateCov(1:3,1:3)*Jr' + Jl*lQ*Jl';
                    stateCov = [stateCov P_lx']; %Add 2 columns to state cov
                    stateCov = [stateCov; P_lx P_ll]; %Add 2 rows to state cov
                    nLandmarksCurrent = nLandmarksCurrent + 1;
                    location = nLandmarksCurrent;
                else
                    location = find(landmarkList==landmarkRaw(1));
                end
                [z, H] = observation_model(stateMean(1:3), ...
                    [stateMean(3+location*2-1) stateMean(3+location*2)], ...
                    location, nLandmarksCurrent); % Get z = [landmarkDist, landmarkAngle] and jacobian

                if ([landmarkRaw(3) landmarkRaw(2)] - z)<10
                    K = (stateCov*(H'))/(H*stateCov*(H')+lQ);
                    stateMean = stateMean + K*([landmarkRaw(3) landmarkRaw(2)]' - z');
                    aux = K*H;
                    stateCov = (eye(size(aux))-aux)*stateCov;
                end
            end
        end
    else
        last_odom = data(t).odom; % Save last odometry measurement
        index = index+1;
    end
    runtime(t)=toc;

    %error(t)=norm(abs(stateMean(1:2)')-abs(robotPose(floor(t/2)+rem(t,2),1:2)))^2;
    %%%%
    %if error(t)>2
    %    disp('here')
    %end
    %%%%

    %   3. Graphics
    if sim
        set(RG, 'xdata', robotPose(index,1), ...
            'ydata', robotPose(index,2)); %simulator robot
        set(rG, 'xdata', stateMean(1), 'ydata', stateMean(2)); %estimator robot
    end
    el = [stateMean(1); stateMean(2)];
    EL = stateCov(1:2, 1:2);
    [X,Y] = cov2elli(el,EL,3,16);
    set(relipG, 'xdata', Y, 'ydata', X);
    lids = nLandmarksCurrent;
    if lids > 0
        for lid = 1:lids
            lx = stateMean(3+(lid*2)-1,1);  %landmarks x coordinate
            ly = stateMean(3+lid*2,1);  %landmarks y coordinate
            set(lG(lid), 'xdata', ly, 'ydata', lx);
            el = [stateMean(3+(lid*2)-1,1); stateMean(3+(lid*2),1)];
            EL = stateCov([3+(lid*2)-1 3+(lid*2)],[3+(lid*2)-1 3+(lid*2)]);
            [X,Y] = cov2elli(el,EL,3,16);
            set(eG(lid), 'xdata', Y, 'ydata', X);
        end
    end
    estPose(t,1) = stateMean(1,1);
    estPose(t,2) = stateMean(2,1);
    drawnow;
    %pause(0.1);
end

%Error calculations
maxErrorSquared=max(error)
avgErrorSquared=mean(error)

%TimeCalculations
maxTime=max(runtime)
avgTime=mean(runtime)


% Draw estimated robot pose
figure(2); hold on;
ReGtotal = line(...
        'linestyle','-',...
        'marker','none',...
        'color','b',...
        'xdata',estPose(:,2),...
        'ydata',estPose(:,1));

% Draw estimated landmarks
for i=1:nLandmarksCurrent
    figure(2);
    LeG = line(...
    'linestyle','none',...
    'marker','+',...
    'color','r',...
    'xdata',stateMean(3+2*i),...
    'ydata',stateMean(3+2*i-1));
end
