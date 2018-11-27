% Data vector contains
% data.odom is a matrix [fwAngle, fwVelocity, bwAngle, bwVelocity] * nTimestamps
% % fw is front wheels, bw is back wheels
% data.landmark is a vector of structures
% data.landmark(t) contains {nLandmarksSeen, landmarkSeen[]}
% data.landmark(t).landmarkSeen(i) = [landmarkID, landmarkDist, landmarkAngle]
% nLandmarksSeen is the number of landmarks observed in one image
data = load(data.mat);

%% Static Variables
nTimestamps = data.nTimestamps;
nLandmarksTotal = data.nLandmarks;
wheeldistance = 0.21;
rNoise = zeros(4, 1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Dynamic Variables
stateMean = zeros(3+2*nLandmarksTotal, 1);
stateCov = zeros(3,3);
%TODO --> Avoid overwrite before matching step

Rn = zeros(3,3); % Robot noise
lQ = zeros(2,2); % Landmark noise

nLandmarksSeen = 0;
nLandmarksCurrent = 0;
landmarkRaw = zeros(3, 1);
landmarkXY = zeros(2, 1);
landmarkList = zeros(nLandmarksTotal, 1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T = length(data.odom,2);

for t = 1:nTimestamps
	%% Prediction step
    %TODO --> Calculate Rn
    if online == 0
        %should ensure t+1 does not break
        if t+1 < T
            time = data.odom(5, t+1);
        else
            time= data.odom(5, t)+1;
        end
    else
        time = clock;
    end
	[stateMean(1:3), rJacob, nJacob] = ...
		movement_model(stateMean(1:3), data.odom(1:5, t), rNoise(:), time, ...
        wheeldistance, nLandmarksCurrent, Rn);
    
    stateCov = rJacob*stateCov*rJacob' + nJacob;
	%% Correction step
    nLandmarksSeen = data.landmark(t).nLandmarksSeen;
    if nLandmarksSeen > 0
        for i = 1:nLandmarksSeen
            landmarkRaw = data.landmark(t).landmarkSeen(i); % Get [landmarkID, landmarkDist, landmarkAngle]
            %lQ = [var(landmarkRaw(2)) 0; 0 var(landmarkRaw(3))];
            if ~ismember(landmarkRaw(1), landmarkList) % If never seen before, add new Landmark
                landmarkXY = new_landmark(landmarkRaw(2:3), stateMean(1:3)); % Get [landmarkX, landmarkY]
                landmarkList(nLandmarksCurrent) = landmarkRaw(1); % Add ID to list of landmarks
                stateMean(3+nLandmarksCurrent*2+1) = landmarkXY(1); % Add X to state mean
                stateMean(3+nLandmarksCurrent*2+2) = landmarkXY(2); % Add Y to state mean
                stateCov = [stateCov zeros(3+nLandmarksCurrent*2, 2)]; %Add 2 collums to state cov
                stateCov = [stateCov; zeros(2, 3+nLandmarksCurrent*2+2)]; %Add 2 rows to state cov
                nLandmarksCurrent = nLandmarksCurrent + 1;
                location = nLandmarksCurrent;
            else
                location = find(landmarkRaw(1), landmarkList);
            end
            [z, H] = observation_model(stateMean(1:3), ...
                [stateMean(3+location*2-1) stateMean(3+location*2)], ...
                location, nLandmarksCurrent); % Get z = [landmarkDist, landmarkAngle] and jacobian
            K = stateCov*(H')*(H*stateCov*(H')+lQ)';
            stateMean = stateMean + K*(landmarkRaw(2:3)' - z);
            aux = K*H;
            stateCov = (eye(size(aux))-aux)*stateCov;
        end
    end
end
