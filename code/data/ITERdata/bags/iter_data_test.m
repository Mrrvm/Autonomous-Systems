%clear;
%load('iterdata4.mat');

%%
nTimestamps = length(data);
pose = [0; 0; 0];
last_odom = zeros(1,4);
last_time = data(1).timestamp;
rNoise = [0 0 0 0];
wheeldistance = 0.21;
nLandmarksCurrent = 0;
Rn = zeros(4,4);
x = zeros(length(data));
y = zeros(length(data));

%%
for t = 1:nTimestamps
    %% Prediction step
    %TODO --> Calculate Rn
    pose = ...
        movement_model(pose, [last_odom last_time], rNoise(:), data(t).timestamp, ...
        wheeldistance, nLandmarksCurrent, Rn);
    last_time = data(t).timestamp;
    last_odom = data(t).odom;
    
    x(t) = pose(1);
    y(t) = pose(2);
    
    %{
    pose = data(t).odom;
    time = data(t).timestamp;
    timediff = time - last_time;
    last_time = time;
    x(t) = x(t-1) + (pose(1)*timediff)*cos(pose(2));
    y(t) = y(t-1) + (pose(3)*timediff)*sin(pose(4));
    %}

end

%%
figure(); hold on;
plot(x,y);

