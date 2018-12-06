function [landmark, Jr, Jl] = new_landmark(measurement, robotPose)
%  NEW_LANDMARK gives us x and y for a new landmark (one that was not on
%  the map vector yet)
%
%  In:
%      measurement:  point in sensor frame   measurement = [d, alfa]
%      robotPose:   robot frame         robotPose = [x, y, alfa]
%  Out:
%      landmark: point in global frame   landmark = [x, y]
%      Jr:  Jacobian in order to robotPose
%      Jl:  Jacobian in order to landmark

    alfa = robotPose(3) + measurement(2);
    
    landmark(1) = robotPose(1) + measurement(1)*cos(alfa);
    landmark(2) = robotPose(2) + measurement(1)*sin(alfa);
    
    if nargout > 1
        Jr = [1 0 -measurement(1)*sin(alfa); 0 1 measurement(1)*cos(alfa)];

        Jl = [cos(robotPose(3)) -sin(robotPose(3))
            sin(robotPose(3)) cos(robotPose(3))] * ...
            [cos(measurement(2)) -measurement(1)*sin(measurement(2))
            sin(measurement(2)) measurement(1)*cos(measurement(2))];
    end
end