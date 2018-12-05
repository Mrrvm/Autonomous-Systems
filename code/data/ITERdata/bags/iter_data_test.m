%clear;
%load('iterdata4.mat');

%%
nTimestamps = length(data);
pose = [0; 0; 0];
last_odom = zeros(1,4);
last_time = data(1).timestamp;
rNoise = zeros(4, 1);
wheeldistance = 0.21;
nLandmarksCurrent = 0;
Rn = zeros(4,4);



%%
for t = 1:nTimestamps
    %% Prediction step
    %TODO --> Calculate Rn
    
    pose = ...
        movement_model(pose, [last_odom last_time], rNoise(:), data(t).timestamp, ...
        wheeldistance, nLandmarksCurrent, Rn);
    last_time = data(t).timestamp;
    last_odom = data(t).odom;
    
    x(t)=pose(1);
    y(t)=pose(2);
end

%%
figure(); hold on;
plot(x,y);