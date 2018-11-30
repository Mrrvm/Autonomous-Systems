%%
%Map Generation

%robot is initialized at position [0,0] with orientation 0
initialRobotPose=[0,0,0];  %[x,y,alpha]

%Flag for default or customized landmarks-- CHANGE HERE
Lrand = true;
nRandLandmarks=5;
LandmarkLimits=[-10,10];

odomRand =true;
odomSpeedLimits=[0, 3];
odomAngleLimits=[0, 360];

%%
if Lrand
    %here random landmarks
    nLandmarks=nRandLandmarks;
    
    landmarkMap= randi(LandmarkLimits,nRandLandmarks,2);
else
    %create default landmarks
    nLandmarks=5;
    
    landmarkMap=[2 -1; -4 -1; 6 -1; -3 -2; 1 -1]; 
end

timelimit=10;

%%
%Define control signal
samplingtime_odom=1;
t_odom=0:samplingtime_odom:timelimit; %TODO fancy time

if odomRand
    controlSignal=[randi(odomSpeedLimits,length(t_odom),1),...
        randi(odomAngleLimits,length(t_odom),1), ...
        randi(odomSpeedLimits,length(t_odom),1), ...
        randi(odomAngleLimits,length(t_odom),1), ...
        t_odom'
        ];
else
    %straigth line
    ctr=[2,90,2,-90];
    %ctr=[1,0,1,0];
    controlSignal=[repmat(ctr,[length(t_odom),1]) t_odom'];
    %
end

%%
%Define camera capture instants
%samplingtime_cam=1;
%t_cam=0:samplingtime_cam:timelimit; %TODO fancy time



%%
%Robot Pose based on controlsignal through time

robotPose=[initialRobotPose;zeros(timelimit,3)];

for i=2:size(controlSignal,1)
    [robotPose(i,:),~,~]= ...
        movement_model(robotPose(i-1,:),controlSignal(i-1,:),zeros(1,4)',t_odom(i), ...
        0.21, 5, zeros(3));
end

%%
%Measurement Calculations
n=1;

for i=1:size(robotPose,1)
    seenLand(i,:)=[0 i-1];
    for j=1:size(landmarkMap,1)
        [measurement,~]=observation_model(robotPose(i,:),landmarkMap(j,:),1,1);
        if measurement(1)<90 || measurement(1)>270
            measurements(n,:)=[j measurement(2) measurement(1) i-1];
            seenLand(i,1)=seenLand(i,1)+1;
            n=n+1;
        end
    end
end

time=sort( [controlSignal(:,5); seenLand(:,2)]);
for i=1:(size(seenLand,1)+size(robotPose,1))
    data(i).option=~rem(i,2);
    data(i).odom=controlSignal(floor(i/2)+rem(i,2),1:4);
    data(i).time=time(i);
    data(i).landmarksSeen=seenLand(floor(i/2)+rem(i,2));
    if seenLand(floor(i/2)+rem(i,2))>0 && ~rem(i,2)
        data(i).landmark= ...
            measurements(measurements(:,4)==(floor(i/2)+rem(i,2)-1),1:3);
    end
end