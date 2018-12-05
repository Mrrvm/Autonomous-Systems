clear all;
close all;
sim = 1;

if sim
    [data, robotPose, landmarkMap] = simulator();
    figure(1); hold on;
    title('Simulator Plot');
    % Draw simulated landmarks
    LsG = line(...
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
    load('data.mat');
end


%% Static Variables
nTimestamps = length(data);
ReG = zeros(1, nTimestamps);
nLandmarksTotal = 12;
wheeldistance = 0.21;
rNoise = [0.1; 0.0175; 0.1; 0.0175];
Rn = diag(rNoise.^2);   %probably wrong
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Dynamic Variables
stateMean = zeros(3, 1);
stateCov = zeros(3,3);
%TODO --> Avoid overwrite before matching step

q = [0.1;0.0175];
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:nTimestamps
     %% Prediction step
    %TODO --> Calculate Rn
    if sim == 1
        noise = rNoise .* randn(4,1);
    else
        noise = zeros(4,1);
    end
    
    [stateMean(1:3), rJacob, nJacob] = ...
        movement_model(stateMean(1:3)', [last_odom last_time], noise, data(t).time, ...
        wheeldistance, nLandmarksCurrent, Rn);
    last_time = data(t).time;

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
                 K = (stateCov*(H'))/(H*stateCov*(H')+lQ);
                 stateMean = stateMean + K*([landmarkRaw(3) landmarkRaw(2)]' - z');
                 aux = K*H;
                 stateCov = (eye(size(aux))-aux)*stateCov;
            end
        end
    else
        last_odom = data(t).odom; % Save last odometry measurement
    end
    
    %% PLOT
    if t == 1
        figure(2); hold on;
        title('EKF Plot');
        plot(stateMean(1),stateMean(2),'og')
        figure(3); hold on;
        title('EKF Plot with Landmark Covariance');
        plot(stateMean(1),stateMean(2),'og')
        figure(4); hold on;
        title('EKF Plot with Robot Covariance');
        plot(stateMean(1),stateMean(2),'og')
    else
        estPose(t,1) = stateMean(1);
        estPose(t,2) = stateMean(2);
        % Draw estimated robot pose
        figure(3);
        ReG(t) = line(...
        'linestyle','none',...
        'marker','^',...
        'color','r',...
        'xdata',stateMean(1),...
        'ydata',stateMean(2));
        figure(4);
        ReG(t) = line(...
        'linestyle','none',...
        'marker','^',...
        'color','r',...
        'xdata',stateMean(1),...
        'ydata',stateMean(2));
        % Draw robot covariance
        ReCov = stateCov(1:2,1:2);
        [xx,yy] = cov2elli([stateMean(1) stateMean(2)],ReCov,3,16);
        eLG = line(...
        'linestyle','-',...
        'marker','none',...
        'color','m',...
        'xdata',xx,...
        'ydata',yy);
        % Draw landmark covariance
        for i=1:nLandmarksCurrent
            figure(3);
            LeCov = stateCov((3+2*i-1):(3+2*i),(3+2*i-1):(3+2*i));
            [xx,yy] = cov2elli([stateMean(3+2*i-1) stateMean(3+2*i)],LeCov,3,16);
            eLG = line(...
            'linestyle','-',...
            'marker','none',...
            'color','g',...
            'xdata',xx,...
            'ydata',yy);
            figure(3);
            LeG = line(...
            'linestyle','none',...
            'marker','+',...
            'color','r',...
            'xdata',stateMean(3+2*i-1),...
            'ydata',stateMean(3+2*i)); 
        end
    end
end

% Draw estimated robot pose
figure(2); hold on;
ReGtotal = line(...
        'linestyle','-',...
        'marker','none',...
        'color','b',...
        'xdata',estPose(:,1),...
        'ydata',estPose(:,2));

% Draw estimated landmarks
for i=1:nLandmarksCurrent
    figure(2);
    LeG = line(...
    'linestyle','none',...
    'marker','+',...
    'color','r',...
    'xdata',stateMean(3+2*i-1),...
    'ydata',stateMean(3+2*i));
    figure(4);
    LeG = line(...
    'linestyle','none',...
    'marker','+',...
    'color','r',...
    'xdata',stateMean(3+2*i-1),...
    'ydata',stateMean(3+2*i));
end