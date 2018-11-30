function [robotPoseUpdated, G_t, R_t] = ...
    movement_model(robotPose, controlSignal, noise, calcTimeStamp, ...
    wheeldistance, nLandmarks, jrp_n)
%  Model of the movement of the robot for slam.
%
%  In:
%      robotPose= [x ; y ; alpha]
%      controlSignal= [theta_f, V_f, theta_r, V_r]
%      noise = additive to control signal
%      wheeldistance = distance between the two wheels of the ITER
%  Out:
%      robotPoseUpdated: updated robot pose
%      jrp_r: Jacobian d(ro) / d(r) - derivate to robotpose
%      jrp_n: Jacobian d(ro) / d(n) - derivated to noise

  timediff = (calcTimeStamp-controlSignal(5));
  displacement=controlSignal(1:4)+noise';

  d_x = (displacement(1)*cosd(robotPose(3)+displacement(2))+ ...
          displacement(3)*cosd(robotPose(3)+displacement(4)))/2;

  d_y = (displacement(1)*sind(robotPose(3)+displacement(2))+ ...
          displacement(3)*sind(robotPose(3)+displacement(4)))/2;

  d_alpha = (displacement(1)*sind(displacement(2))- ...
          displacement(3)*sind(displacement(4)))/wheeldistance;

  robotPoseUpdated = robotPose + timediff*[d_x, d_y, d_alpha];


  % Calculate Jacobians
     jrp_p=[ 0, 0, (-displacement(1)*sind(robotPose(3)+displacement(2)) ...
                -displacement(3)*sind(robotPose(3)+displacement(4)))/2;
        0, 0, (displacement(1)*cosd(robotPose(3)+displacement(2)) ...
                +displacement(3)*cosd(robotPose(3)+displacement(4)))/2;
        0, 0, 0];

% DISPLACEMENT IS WRONG 
%     jrp_n=[
%         (-displacement(2)*sind(robotPose(3)+displacement(1)))/2, ...
%         cosd(robotPose(3)+displacement(1))/2, ...
%         (-displacement(4)*sind(robotPose(3)+displacement(3)))/2, ...
%         cosd(robotPose(3)+displacement(3))/2;
% 
%         (displacement(2)*cosd(robotPose(3)+displacement(1)))/2, ...
%         sind(robotPose(3)+displacement(1))/2, ...
%         (displacement(4)*cosd(robotPose(3)+displacement(3)))/2, ...
%         sind(robotPose(3)+displacement(3))/2;
% 
%         displacement(2)*cosd(displacement(1))/wheeldistance, ...
%         sind(displacement(1))/wheeldistance, ...
%         -displacement(4)*cosd(displacement(3))/wheeldistance, ...
%         -sind(displacement(3))/wheeldistance
%     ];


  %Massage jacobians for Covariance matrix update
  F=[eye(3), zeros(3,2*nLandmarks)];
  G_t=eye(3+2*nLandmarks)+F'*jrp_p*F;
  R_t=F'*jrp_n*F;
