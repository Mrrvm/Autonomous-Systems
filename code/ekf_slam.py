import numpy as np
import rospy
from motion import Movement


def predict(self, rPose, rControl, rNoise):
    rPoseJacob = np.zeros(3, 3)
    rNoiseJacob = np.zeros(3, 4)
    [rPose, rPoseJacob, rNoiseJacob] = Movement(rPose, rControl, rNoise)


def correct(self):
    pass


def observe(self, msg):
    # if new landmark
    #   add to state vector
    # else
    #   correct()
    pass

if __name__ == '__main__':
    """
        
        Variables
            nLandmarks: Initial number of landmarks (zero)
            stateMean: State vector's mean (state vector is state[robot's pose, landmarks])
                robot's pose [x, y, alpha]
            stateCov: State vector's covariance matrix
            rNoise: Robot's Noise (todo)
            rNoiseCov: Covariance of noise vector (todo)
            rControl: [fw angle, fw velocity, bw angle, bw velocity]
                fw is front wheels, bw is back wheels
                information is got through ITER 
        
    """

    nActiveLandmarks = 0
    nTotalLandmarks = 10
    stateMean = np.zeros(3+nTotalLandmarks*2)
    stateCov = np.zeros(stateMean.size, stateMean.size)
    rNoise = np.zeros(2)
    rNoiseCov = 0
    rControl = np.zeros(4)

    # Register node in ROS network
    rospy.init_node('ekf-slam', anonymous=False)
    # Print message in terminal
    rospy.loginfo('EKF-SLAM is Live!')

    # Subscribe to camera rosnode
    # rospy.Subscriber("camera_topic", ADD MESSAGE STRUCTURE HERE, observe)

    # Subscribe to control rosnode
    # rospy.Subscriber("control_topic", ADD MESSAGE STRUCTURE HERE, observe)

    while True:

        predict(stateMean[0:3], rControl, rNoise)

        rospy.sleep(0.5) # sleep for 0.5 seconds



