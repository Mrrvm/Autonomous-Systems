%%
%Map Generation

%robot is initialized at position [0,0] with orientation 0
initialRobotPose=[0,0,0];  %[x,y,alpha]

%Flag for default or customized landmarks
Lcustom = false;

if Lcustom
    %TODO add user input possibility
else
    %create default landmarks
    nLandmarks=5;
    
    landmarkMap=[2 4; -4 1; -6 -1; -3 -2; -1 -1]; 
end

%%
%Define control signal
samplingtime=1;
timelimit=10;
t=0:samplingtime:timelimit;

%straigth line
ctr=[0,1,0,1];
controlSignal=[repmat(ctr,[length(t),1]) t'];
%


%%
%Robot Pose based on controlsignal through time

robotPose=[initialRobotPose;zeros(timelimit,3)];

for i=2:size(controlSignal,1)
    [robotPose(i,:),~,~]= ...
        movement_model(robotPose(i-1,:),controlSignal(i-1,:),zeros(1,4),t(i), ...
        0.21, 5, zeros(3));
end

%%
%Measurement Calculations
n=1;

for i=1:size(robotPose,1)
    for j=1:size(landmarkMap,1)
        [measurement,~]=observation_model(robotPose(i,:),landmarkMap(j,:),1,1);
        if abs(measurement(2))<90
            measurements(n,:)=[measurement' j i];
            n=n+1;
        end
    end
end
        