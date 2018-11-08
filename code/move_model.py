import numpy as np


def Movement(robotPose, controlSignal, noise):
    """
    Model of the movement of the robot for slam. The data is assumed to be
        treated. Meaning that alpha is not the angle of the wheels but of the
        the robot in relation to the global frame, dx is the increment to
        x(localFrame)and not the speed of the wheels.
    In:
        robotPose r = [x ; y ; alpha]
        controlSignal u = [dx ; d_alpha]
        noise, additive to control signal
    Out:
        robotPoseUpdated: updated robot pose
        jrp_r: Jacobian d(ro) / d(r) - covariances?
        jrp_n: Jacobian d(ro) / d(n) - covariances?
    """
    alphaEstimate = robotPose[2] + controlSignal[1] + noise[1];
            #calculate estimate of alpha(k+1) as alpha(k) + d_calpha(k)/dalpha
            #   range should be from [-pi:pi] REVIEW
            #  NOTE:if need arises to map to [0:2pi] we can use unwrap

    dRobotpos = np.array([controlSignal[0]+noise[0], 0]);
            #increment to the robots Pose is [dx, dy] where dx is the increment
            #   in the robots frame
            #
            #zero on dy defines that the robot can only move to the front

    [dRobotposGF,frame_j,dRobotposGF_j]=FromLocalFrameToGlobalFrame(robotPose,dRobotpos);
            #convert change in robot pose due to dRobotpos from robot frame
            #   (forward assuming movement is always to the front-- REVIEW HERE)
            #   dRobotpos needs to be the increment to position of the robot

    jrp_r=np.matrix([frame_j,[0,0,1]]);

    jrp_n=np.matrix([[dRobotposGF_j[:,0],np.zeros([2,1])],[0,1]]);

    robotPoseUpdated=np.array([dRobotposGF,alphaEstimate]);

    return [robotPoseUpdated,jrp_r,jrp_n];


def FromLocalFrameToGlobalFrame(localFrame, pointLF):
    """
    convert pointLF from frame localFrame to globalFrame
    In:
        localFrame: reference frame is the robots pose given as [xGF,yGF,alpha]
        pointLF: point in frame F pf = [xLF ; yLF]
    Out:
        pointGF: point in global frame
        frame_j: Jacobian wrt F
        pointGF_j: Jacobian wrt pf
    """
    alpha=localFrame[2];
    R=np.matrix([[np.cos(alpha),-np.sin(alpha)],[np.sin(alpha),np.cos(alpha)]])

    pointGF= R*np.transpose(pointLF[np.newaxis]) + localFrame[0:1];

    #calculate Jacobians

    frame_j=np.matrix([
        [1, 0, -pointLF[1]*np.cos(alpha)-pointLF[0]*np.sin(alpha)],
        [0, 1, pointLF[0]*np.cos(alpha)-pointLF[1]*np.sin(alpha)]]);

    pointGF_j=R;

    return [pointGF,frame_j,pointGF_j];
