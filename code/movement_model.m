function [robotPoseUpdated, G_t, R_t] = ...
    movement_model(robotPose, controlSignal, noise, calcTimeStamp, ...
    wheeldistance, nLandmarks, R_n)
%  Model of the movement of the robot for slam.
%
%  In:
%      robotPose= [x ; y ; alpha]
%      controlSignal= [theta_f, V_f, theta_r, V_r, timestamp]
%      noise = additive to control signal
%      wheeldistance = distance between the two wheels of the ITER
%  Out:
%      robotPoseUpdated: updated robot pose
%      jrp_r: Jacobian d(ro) / d(r) - derivate to robotpose
%      jrp_n: Jacobian d(ro) / d(n) - derivated to noise

  timediff = (calcTimeStamp-controlSignal(5));
  displacement=controlSignal(1:4)+noise';

  d_x = (displacement(1)*cos(robotPose(3)+displacement(2))+ ...
          displacement(3)*cos(robotPose(3)+displacement(4)))/2;

  d_y = (displacement(1)*sin(robotPose(3)+displacement(2))+ ...
          displacement(3)*sin(robotPose(3)+displacement(4)))/2;

  d_alpha = (displacement(1)*sin(displacement(2))- ...
          displacement(3)*sin(displacement(4)))/wheeldistance;

  robotPoseUpdated = robotPose + timediff*[d_x, d_y, d_alpha];
  robotPoseUpdated(3) = wrapToPi(robotPoseUpdated(3));
    
  if nargout > 1
    % Calculate Jacobians
    jrp_p= timediff*[ 0, 0, (-displacement(1)*sin(robotPose(3)+displacement(2)) ...
            -displacement(3)*sin(robotPose(3)+displacement(4)))/2;
                   0, 0, (displacement(1)*cos(robotPose(3)+displacement(2)) ...
            +displacement(3)*cos(robotPose(3)+displacement(4)))/2;
                   0, 0, 0];
    % DISPLACEMENT IS WRONG 
    j_wrt_c= timediff*[
     cos(robotPose(3)+displacement(2))/2, ...
     (-noise(1)*sin(robotPose(3)+displacement(2)))/2, ...
     cos(robotPose(3)+displacement(4))/2, ...
     (-noise(3)*sin(robotPose(3)+displacement(4)))/2;

     sin(robotPose(3)+displacement(2))/2, ...
     (noise(1)*cos(robotPose(3)+displacement(2)))/2, ...
     sin(robotPose(3)+displacement(4))/2, ...
     (noise(3)*cos(robotPose(3)+displacement(4)))/2;

     sin(displacement(2))/wheeldistance, ...
     noise(1)*cos(displacement(2))/wheeldistance, ...
     -sin(displacement(4))/wheeldistance, ...
     -noise(3)*cos(displacement(4))/wheeldistance
    ]*timediff;

    jrp_n = j_wrt_c*R_n*j_wrt_c';

     %Massage jacobians for Covariance matrix update
     F=[eye(3), zeros(3,2*nLandmarks)];
     G_t=eye(3+2*nLandmarks)+F'*jrp_p*F;
     R_t=F'*jrp_n*F;
  end
  
end