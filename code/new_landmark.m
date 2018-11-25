function [landmark] = new_landmark(measurement, robotPose)
%  NEW_LANDMARK gives us x and y for a new landmark (one that was not on
%  the map vector yet)
%
%  In:
%      measurement:  point in sensor frame   measurement = [d, alfa]
%      robotPose:   robot frame         robotPose = [x, y, alfa]
%  Out:
%      landmark: point in global frame   landmark = [x, y]

    alfa = robotPose(3) + measurement(2);
    
    landmark(1) = robotPose(1) + measurement(1)*cos(alfa);
    landmark(2) = robotPose(2) + measurement(1)*sin(alfa);
    
end