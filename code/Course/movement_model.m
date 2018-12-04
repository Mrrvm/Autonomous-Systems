function [robotPoseUpdated, RO_r, RO_n] = movement_model(robotPose, controlSignal, noise, calcTimeStamp, ...
    wheeldistance)
%  Model of the movement of the robot for slam.
%
%  In:
%      robotPose= [x ; y ; alpha]
%      controlSignal= [theta_f, V_f, theta_r, V_r, time]
%      noise = additive to control signal
%      wheeldistance = distance between the two wheels of the ITER
%  Out:
%      robotPoseUpdated: updated robot pose
%      jrp_r: Jacobian d(ro) / d(r) - derivate to robotpose
%      jrp_n: Jacobian d(ro) / d(n) - derivated to noise

  timediff = (calcTimeStamp-controlSignal(5));
  displacement=controlSignal(1:4)+noise;

  d_x = (displacement(1)*cos(robotPose(3)+displacement(2))+ ...
          displacement(3)*cos(robotPose(3)+displacement(4)))/2;

  d_y = (displacement(1)*sin(robotPose(3)+displacement(2))+ ...
          displacement(3)*sin(robotPose(3)+displacement(4)))/2;

  d_alpha = (displacement(1)*sin(displacement(2))- ...
          displacement(3)*sin(displacement(4)))/wheeldistance;

  robotPoseUpdated = robotPose + timediff*[d_x; d_y; d_alpha];
  robotPoseUpdated(3) = wrapToPi(robotPoseUpdated(3));

  if nargout > 1
  % Calculate Jacobians
     RO_r=[ 1, 0, (-displacement(1)*sin(robotPose(3)+displacement(2)) ...
                -displacement(3)*sin(robotPose(3)+displacement(4)))/2;
        0, 1, (displacement(1)*cos(robotPose(3)+displacement(2)) ...
                +displacement(3)*cos(robotPose(3)+displacement(4)))/2;
        0, 0, 1]*timediff;
    
    
    
    % IF WE DEFINE THE NOISE IN THE SAME SPACE AS THE STATE IT WOULD BE
    % EASIER
     RO_n=[
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
  end
  
end
  
%%
function f()
%% this is to find the jacobian
syms x y a u1 u2 u3 u4 tu n1 n2 n3 n4 t wd real
r = [x y a]';
u = [u1 u2 u3 u4 tu]';
n = [n1 n2 n3 n4]';
p_r = movement_model(r, u, n, t, wd);
RO_r = jacobian(p_r,r)
end