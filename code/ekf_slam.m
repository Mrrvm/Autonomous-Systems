% Data vector contains
% [fwAngle, fwVelocity, bwAngle, bwVelocity, landmarkID, landmarkDist, landmarkAngle] * nTimestamps
% fw is front wheels, bw is back wheels
load(data.mat);

%% Static Variables
nTimestamps = 100;
nLandmarksCurrent = 0;
nLandmarksTotal = 10;
wheeldistance = 0.21;
rNoise = zeros(2, 1); % todo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Dynamic Variables
stateMean = zeros(3+2*nLandmarksTotal, 1);
stateCov = zeros(3+2*nLandmarksTotal, 3+2*nLandmarksTotal);

rJacobian = zeros(3,3);
rNoiseJacobian = zeros(2,2);
lJacobian = zeros(2,3);
lNoiseJacobian = zeros(2,2);

landmark = zeros(3, 1);
landmarkList = zeros(nLandmarksTotal, 1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:nTimestamps

	%% Prediction step
	[stateMean(1:3), rJacobian, rNoiseJacobian] = ...
		movement_model(stateMean(1:3), rControl(1:4, t) , rNoise(:), wheeldistance);
	% Calculate new stateCov
	% todo

	%% Landmark Observation
	[landmark, lJacobian, lNoiseJacobian] = observation_model(data(5:7, t), stateMean(1:3));
	if ismember(landmark(3), landmarkList)
		%% Correction step
		location = find(landmark(3), landmarkList);
		% Calculate new stateMean
		% Calculate new stateCov
	else
		%% Add new landmark
		landmarkList(nLandmarksCurrent) = landmark(3);
		stateMean(nLandmarksCurrent*2) = landmark(1);
		stateMean(nLandmarksCurrent*2+1) = landmark(2);
		% Calculate new stateCov ??
		nLandmarksCurrent = nLandmarksCurrent + 1;
	end

end
