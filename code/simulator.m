%function [data, robotPose, landmarkMap] = simulator()

  %%
  %Map Generation

  %robot is initialized at position [0,0] with orientation 0
  initialRobotPose=[0,0,0];  %[x,y,alpha]

  %Flag for default or customized landmarks-- CHANGE HERE
  Lrand = true;
  nLandmarks=5;
  LandmarkLimits=[-10,10];

  odomtype = 3;%0-rand;1-straigth;2-rotation;3-circle;
  odomSpeedLimits=[0, 3];
  odomAngleLimits=[0, 360];

  %Measurement Noise
  q = [.01;.1];

  %%
  if Lrand
      landmarkMap= randi(LandmarkLimits,nLandmarks,2);
  else
      landmarkMap =[2 -1; -4 -1; 6 -1; -3 -2; 1 -1];
  end

  timelimit=10;

  %%
  % Define control signal
  samplingtime_odom=1;
  t_odom=0:samplingtime_odom:timelimit; %TODO fancy time

  if odomtype==0
      controlSignal=[randi(odomSpeedLimits,length(t_odom),1),...
          randi(odomAngleLimits,length(t_odom),1), ...
          randi(odomSpeedLimits,length(t_odom),1), ...
          randi(odomAngleLimits,length(t_odom),1), ...
          t_odom'
          ];
  elseif odomtype==1
      %straigth line
      ctr=[1,0,1,0];
      controlSignal=[repmat(ctr,[length(t_odom),1]) t_odom'];
      %
  elseif odomtype==2
      %rotation
      ctr=[2,wrapToPi(degtorad(90)),2,wrapToPi(degtorad(-90))];
      controlSignal=[repmat(ctr,[length(t_odom),1]) t_odom'];
      %
  elseif odomtype==3
      %circle
      ctr=[1,wrapToPi(degtorad(90)),1,0];
      controlSignal=[repmat(ctr,[length(t_odom),1]) t_odom'];
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
          0.21, nLandmarks, zeros(4));
  end

  %%
  %Measurement Calculations
  n=1;

  for i=1:size(robotPose,1)
      seenLand(i,:)=[0 i-1];
      for j=1:size(landmarkMap,1)
          [measurement,~]=observation_model(robotPose(i,:),landmarkMap(j,:),1,1);
          if (measurement(2)<90 || measurement(2)>270) && measurement(2)<2
              measNoise = q.*randn(2,1);
              measurements(n,:)=[j (measurement(2)+measNoise(1)) (measurement(1)+measNoise(2)) i-1];
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

%{
figure(1); hold on;
plot(robotPose(1,1),robotPose(1,2),'og')
plot(robotPose(:,1), robotPose(:,2))
for i=1:length(landmarkMap)
    plot(landmarkMap(i,1),landmarkMap(i,2),'xr')
end
title('Simulator Plot');
hold off;
clearvars -except data robotPose landmarkMap
%}
