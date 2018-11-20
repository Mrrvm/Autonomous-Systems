function [robotPoseUpdated,jrp_r,jrp_n] = movement_model(robotPose, controlSignal, noise, wheeldistance)
%  Model of the movement of the robot for slam. The controlSignal is assumed to be
%      treated, i.e. is assumed to be [theta_f, V_f*(t-tod), theta_r, V_r*(t-tod)].
%
%  In:
%      robotPose= [x ; y ; alpha]
%      controlSignal= [theta_f, V_f*(t-tod), theta_r, V_r*(t-tod)]
%      noise= additive to control signal
%      wheeldistance= distance between the two wheels of the ITER
%  Out:
%      robotPoseUpdated: updated robot pose
%      jrp_r: Jacobian d(ro) / d(r) - derivate to robotpose
%      jrp_n: Jacobian d(ro) / d(n) - derivated to noise

  displacement=controlSignal+noise;

  d_x = (displacement(2)*cos(robotPose(3)+displacement(1))+ ...
          displacement(4)*cos(robotPose(3)+displacement(3)))/2;

  d_y = (displacement(2)*sin(robotPose(3)+displacement(1))+ ...
          displacement(4)*sin(robotPose(3)+displacement(3)))/2;

  d_alpha = (displacement(2)*sin(displacement(1))- ...
          displacement(4)*sin(displacement(3)))/wheeldistance;


  robotPoseUpdated = robotPose + [d_x, d_y, d_alpha];


  % Calculate Jacobians
    jrp_r = [ 1, 0, (-displacement(2)*sin(robotPose(3)+displacement(1)) ...
                -displacement(4)*sin(robotPose(3)+displacement(3)))/2;
        0, 1, (displacement(2)*cos(robotPose(3)+displacement(1)) ...
                +displacement(4)*cos(robotPose(3)+displacement(3)))/2;
        0, 0, 1];

    jrp_n=[
        (-displacement(2)*sin(robotPose(3)+displacement(1)))/2,
        cos(robotPose(3)+displacement(1))/2,
        (-displacement(4)*sin(robotPose(3)+displacement(3)))/2,
        cos(robotPose(3)+displacement(3))/2;

        (displacement(2)*cos(robotPose(3)+displacement(1)))/2,
        sin(robotPose(3)+displacement(1))/2,
        (displacement(4)*cos(robotPose(3)+displacement(3)))/2,
        sin(robotPose(3)+displacement(3))/2;

        displacement(2)*cos(displacement(1))/wheeldistance,
        sin(displacement(1))/wheeldistance,
        -displacement(4)*cos(displacement(3))/wheeldistance,
        -sin(displacement(3))/wheeldistance
    ];