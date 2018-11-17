import numpy as np


def Movement(robotPose, controlSignal, noise, wheeldistance):
    """
    Model of the movement of the robot for slam. The controlSignal is assumed to be
        treated, i.e. is assumed to be [theta_f, V_f*(t-tod), theta_r, V_r*(t-tod)].

    In:
        robotPose= [x ; y ; alpha]
        controlSignal= [theta_f, V_f*(t-tod), theta_r, V_r*(t-tod)]
        noise= additive to control signal
        wheeldistance= distance between the two wheels of the ITER
    Out:
        robotPoseUpdated: updated robot pose
        jrp_r: Jacobian d(ro) / d(r) - derivate to robotpose
        jrp_n: Jacobian d(ro) / d(n) - derivated to noise
    """
    displacement=controlSignal+noise;

    d_x=(displacement[1]*np.cos(robotPose[2]+displacement[0])+
            displacement[3]*np.cos(robotPose[2]+displacement[2]))/2;

    d_y=(displacement[1]*np.sin(robotPose[2]+displacement[0])+
            displacement[3]*np.sin(robotPose[2]+displacement[2]))/2;

    d_alpha=(displacement[1]*np.sin(displacement[0])-
            displacement[3]*np.sin(displacement[2]))/wheeldistance;


    robotPoseUpdated=robotPose+np.array([d_x, d_y, d_alpha]);

    # Calculate Jacobians
    jrp_r= np.matrix([
        [1, 0, (-displacement[1]*np.sin(robotPose[2]+displacement[0])
                -displacement[3]*np.sin(robotPose[2]+displacement[2]))/2],
        [0, 1, (displacement[1]*np.cos(robotPose[2]+displacement[0])
                +displacement[3]*np.cos(robotPose[2]+displacement[2]))/2],
        [0, 0, 1]]);

    jrp_n= np.matrix([
        [
        (-displacement[1]*np.sin(robotPose[2]+displacement[0]))/2,
        np.cos(robotPose[2]+displacement[0])/2,
        (-displacement[3]*np.sin(robotPose[2]+displacement[2]))/2,
        np.cos(robotPose[2]+displacement[2])/2
        ],
        [
        (displacement[1]*np.cos(robotPose[2]+displacement[0]))/2,
        np.sin(robotPose[2]+displacement[0])/2,
        (displacement[3]*np.cos(robotPose[2]+displacement[2]))/2,
        np.sin(robotPose[2]+displacement[2])/2
        ],
        [
        displacement[1]*np.cos(displacement[0])/wheeldistance,
        np.sin(displacement[0])/wheeldistance,
        -displacement[3]*np.cos(displacement[2])/wheeldistance,
        -np.sin(displacement[2])/wheeldistance
        ]
    ]);
    return [np.squeeze(np.asarray(robotPoseUpdated)),jrp_r,jrp_n];
