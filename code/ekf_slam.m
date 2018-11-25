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
wheeldistance = 0; % todo
rNoise = zeros(4, 1); % todo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Dynamic Variables 
stateMean = zeros(3+2*nLandmarksTotal, 1);
stateCov = zeros(3+2*nLandmarksTotal, 3+2*nLandmarksTotal);

rJacobian = zeros(3,3);
rNoiseJacobian = zeros(2,2);
lJacobian = zeros(2,3);
lNoiseJacobian = zeros(2,2);

nLandmarksSeen = 0
nLandmarksCurrent = 0;
landmarkRaw = zeros(3, 1);
landmarkXY = zeros(2, 1);
landmarkList = zeros(nLandmarksTotal, 1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:nTimestamps

	%% Prediction step
	[stateMean(1:3), rJacobian, rNoiseJacobian] = ...
		movement_model(stateMean(1:3), data.odom(1:4, t) , rNoise(:), wheeldistance);
	% Calculate new stateCov
	% todo

	%% Correction step    
    nLandmarksSeen = data.landmark(t).nLandmarksSeen;
    if nLandmarksSeen > 0
        for i = 1:nLandmarksSeen
            landmarkRaw = data.landmark(t).landmarkSeen(i); % Get [landmarkID, landmarkDist, landmarkAngle]
            if ~ismember(landmarkRaw(1), landmarkList) % If never seen before, add new Landmark
                landmarkXY = new_landmark(landmarkRaw(2:3), stateMean(1:3)); % Get [landmarkX, landmarkY]
                landmarkList(nLandmarksCurrent) = landmarkRaw(1); % Add ID to list of landmarks
                stateMean(3+nLandmarksCurrent*2+1) = landmarkXY(1); % Add X to state mean
                stateMean(3+nLandmarksCurrent*2+2) = landmarkXY(2); % Add Y to state mean
                nLandmarksCurrent = nLandmarksCurrent + 1;
                location = nLandmarksCurrent;
            else
                location = find(landmarkRaw(1), landmarkList); 
            end
            [z, H] = observation_model(stateMean(1:3), ...
                [stateMean(3+location*2-1) stateMean(3+location*2)], ...
                location, nLandmarksCurrent); % Get z = [landmarkDist, landmarkAngle] and jacobian 
            K = stateCov*(H')*(H*stateCov*(H')+Q)';
            stateMean = stateMean + K*(landmarkRaw(2:3)' - z);
            aux = K*H;
            stateCov = (eye(size(aux))-aux)*stateCov;
        end
    end
end