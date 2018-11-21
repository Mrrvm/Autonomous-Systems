function [landmark] = observation_model(measurement, robotPose)
%  Observation Model transforms an observation in the robot frame, to a point
%  on the global frame
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