function [landmark, j_r, j_l] = observation_model(measurement, robotPose)
%  SENSOR_MEASUREMENT receives the data sent by the sensor and transforms
%  it into usefull information
%
%  In:
%      measurement:  point in sensor frame   measurement = [d, alfa, ID]
%      robotPose:   robot frame         robotPose = [x, y, alfa]
%  Out:
%      landmark: point in global frame   landmark = [x, y, landmark_id]
%      j_r: Jacobian in order to the state dh(x)/dr
%      j_l: Jacobian in order to the landmark dh(x)/dl

    landmark(3) = measurement(3);
    
    alfa = robotPose(3) + measurement(2);
    
    landmark(1) = robotPose(1) + measurement(1)*cos(alfa);
    landmark(2) = robotPose(2) + measurement(1)*sin(alfa);
    
    j_r = [1, 0, -measurement(1)*sin(alfa);
        0, 1, measurement(1)*cos(alfa)];
    
    j_l = [cos(robotPose(3)), -sin(robotPose(3))
        sin(robotPose(3)), cos(robotPose(3))];

end