function [z, H] = observation_model(robotPose, landmark, i, N)
%  Observation Model returns what we were supposed to see in terms of range
%  and bearing for a specific landmark in a specific robot pose.
%
%  In:
%      robotPose = [x, y, alfa] current robot pose
%      landmark = [x, y]    current x and y for a specific landmark
%      i:   Index of the landmark above
%      N:   Current number of landmarks
%  Out:
%      z = [d, alfa] range and bearing of the landmark from the robot frame
%      H:   Full and ready to use jacobian matrix

    delta = [landmark(1)-robotPose(1); landmark(2)-robotPose(2)];
    q = delta'*delta;
    d = sqrt(q);
    
    z = [d wrapTo360(wrapTo360(atan2d(delta(2),delta(1)))-robotPose(3))];
    
    A = zeros(1,2*(i-1));
    B = zeros(1,2*(N-i));
    F = [1 0 0 A 0 0 B
        0 1 0 A 0 0 B
        0 0 1 A 0 0 B
        0 0 0 A 1 0 B
        0 0 0 A 0 1 B];
    
    H = (1/q)*[-d*delta(1) -d*delta(2) 0 d*delta(1) d*delta(2)
                delta(2) -delta(1) -q -delta(2) delta(1)]*F;
end