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
    
    landmark(1) = robotPose(1) + measurement(1)*cosd(alfa);
    landmark(2) = robotPose(2) + measurement(1)*sind(alfa);
    
    Jr = [1 0 -measurement(1)*sind(alfa); 0 1 measurement(1)*cosd(alfa)];
    
    Jl = [cosd(robotPose(3)) -sin(robotPose(3))
        sind(robotPose(3)) cosd(robotPose(3))] * ...
        [cosd(measurement(2)) -measurement(1)*sind(measurement(2))
        sind(measurement(2)) measurement(1)*cosd(measurement(2))];
    
end