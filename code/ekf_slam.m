% Data vector contains
% data[fwAngle, fwVelocity, bwAngle, bwVelocity, landmarkID, landmarkDist, landmarkAngle] * nTimestamps
% fw is front wheels, bw is back wheels
load(data.mat);

%% Static Variables
nTimestamps = 100;
nLandmarksTotal = 10;
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

nLandmarksCurrent = 0;
landmark = zeros(2, 1);
landmarkList = zeros(nLandmarksTotal, 1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 1:nTimestamps

	%% Prediction step
	[stateMean(1:3), rJacobian, rNoiseJacobian] = ...
		movement_model(stateMean(1:3), data(1:4, t) , rNoise(:), wheeldistance);
	% Calculate new stateCov
	% todo

	%% Landmark Observation
    nSeen = 0;      %Number of landmarks observed
    %Q = [var(robotState) 0; 0 var(measurement)];
	%[range, bearing, ID] = aruco();
    if nSeen > 0
        for i = 1:nSeen
            if ~ismember(ID, landmarkList)
                %% Add new landmark
                landmark = new_landmark([range bearing], stateMean(1:3));
                stateMean(3+nLandmarksCurrent*2+1) = landmark(1);
                stateMean(3+nLandmarksCurrent*2+2) = landmark(2);
                landmarkList(nLandmarksCurrent+1) = ID;
                nLandmarksCurrent = nLandmarksCurrent + 1;
            end
            %% Correction step
            location = find(ID, landmarkList);
            [z, H] = observation_model(stateMean(1:3), ...
                [stateMean(3+location*2-1) stateMean(3+location*2)], ...
                location, nLandmarksCurrent);
            K = stateCov*(H')*inv(H*stateCov*(H')+Q);
            stateMean = stateMean + K*([range bearing]' - z);
            aux = K*H;
            stateCov = (eye(size(aux))-aux)*stateCov;
        end
    end

end