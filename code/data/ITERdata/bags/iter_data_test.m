clear;
% load(fullfile('90degturn_50rpm','iterdata3.mat'));
load('iterdatacorredorsquare.mat')

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
    data(t,1).odom(1) = data(t).odom(1)/2;
    data(t,1).odom(3) = data(t).odom(3)/2;
    data(t,1).odom(2) = wrapToPi(data(t).odom(2));
    data(t,1).odom(4) = wrapToPi(data(t).odom(4));
    pose = ...
        movement_model(pose, [last_odom last_time], rNoise(:), data(t).timestamp, ...
        wheeldistance, nLandmarksCurrent, Rn);
    last_time = data(t).timestamp;
    last_odom = data(t).odom;
    
    x(t) = pose(1);
    y(t) = pose(2);
    z(t) = pose(3);
    
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

z = rad2deg(z)';

z(end)

