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
    
    landmarkMap=[2 -1; -4 1; -6 -1; -3 -2; -1 -1]; 
end

timelimit=10;

%%
%Define control signal
samplingtime_odom=1;
t_odom=0:samplingtime_odom:timelimit; %TODO fancy time

%straigth line
ctr=[0,1,0,1];
controlSignal=[repmat(ctr,[length(t_odom),1]) t_odom'];
%

%%
%Define camera capture instants
%samplingtime_cam=1;
%t_cam=0:samplingtime_cam:timelimit; %TODO fancy time



%%
%Robot Pose based on controlsignal through time

robotPose=[initialRobotPose;zeros(timelimit,3)];

for i=2:size(controlSignal,1)
    [robotPose(i,:),~,~]= ...
        movement_model(robotPose(i-1,:),controlSignal(i-1,:),zeros(1,4),t_odom(i), ...
        0.21, 5, zeros(3));
end

%%
%Measurement Calculations
n=1;

for i=1:size(robotPose,1)
    seenLand(i)=0;
    for j=1:size(landmarkMap,1)
        [measurement,~]=observation_model(robotPose(i,:),landmarkMap(j,:),1,1);
        if abs(measurement(2))<90
            measurements(n,:)=[measurement' j i];
            seenLand(i)=seenLand(i)+1;
            n=n+1;
        end
    end
end

time=sort( [controlSignal(:,5); measurements(:,4)]);
for i=1:(size(measurements,1)+size(robotPose,1))
    data(i).option=~rem(i,2);
    data(i).odom=controlSignal(floor(i/2)+1);
    data(i).time=time(i);
    data(i).landseen=seenLand(floor(i/2)+1);
    if seenLand(floor(i/2)+1)>0 && ~rem(i,2)
        for j=1:seenLand(floor(i/2)+1)
         data(i).landmarks=measurements(i+j-1,1:3);
        end
    end
end