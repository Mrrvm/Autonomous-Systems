clear;
% load(fullfile('90degturn_50rpm','iteriterdata3.mat'));
load('iterdataSala1.mat')

%%
nTimestamps = length(iterdata);
pose = [0; 0; 0];
last_odom = zeros(1,4);
last_time = iterdata(1).timestamp;
rNoise = [0 0 0 0];
wheeldistance = 0.21;
nLandmarksCurrent = 0;
Rn = zeros(4,4);
x = zeros(length(iterdata));
y = zeros(length(iterdata));

%%
for t = 1:nTimestamps
    %% Prediction step
    %TODO --> Calculate Rn
    iterdata(t,1).odom(1) = iterdata(t).odom(1)/2;
    iterdata(t,1).odom(3) = iterdata(t).odom(3)/2;
    iterdata(t,1).odom(2) = wrapToPi(iterdata(t).odom(2));
    iterdata(t,1).odom(4) = wrapToPi(iterdata(t).odom(4));
    pose = ...
        movement_model(pose, [last_odom last_time], rNoise(:), iterdata(t).timestamp, ...
        wheeldistance, nLandmarksCurrent, Rn);
    last_time = iterdata(t).timestamp;
    last_odom = iterdata(t).odom;
    
    x(t) = pose(1);
    y(t) = pose(2);
    z(t) = pose(3);
    
    %{
    pose = iterdata(t).odom;
    time = iterdata(t).timestamp;
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

