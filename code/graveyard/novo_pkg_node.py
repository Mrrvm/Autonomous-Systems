#!/usr/bin/env python

import math
# ROS python api with lots of handy ROS functions
import rospy

# to be able to subcribe to laser scanner data
from sensor_msgs.msg import LaserScan

# to be able to publish Twist data (and move the robot)
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

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

    jrp_r=np.concatenate((frame_j,np.array([[0,0,1]])),axis=0);

    jrp_n=np.concatenate((np.concatenate(
        (dRobotposGF_j[:,0],np.zeros([2,1])),axis=1),np.array([[0,1]])),axis=0);

    robotPoseUpdated=np.concatenate((dRobotposGF.T,np.array([[alphaEstimate]])), axis=1);

    return [np.squeeze(np.asarray(robotPoseUpdated)),jrp_r,jrp_n];




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

    pointGF= R*np.transpose(pointLF[np.newaxis]) + np.transpose(localFrame[0:2][np.newaxis]);

    #calculate Jacobians

    frame_j=np.matrix([
        [1, 0, -pointLF[1]*np.cos(alpha)-pointLF[0]*np.sin(alpha)],
        [0, 1, pointLF[0]*np.cos(alpha)-pointLF[1]*np.sin(alpha)]]);

    pointGF_j=R;

    return [pointGF,frame_j,pointGF_j];


class reader(object):

    def __init__(self):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''
        # register node in ROS network
        rospy.init_node('reader_behavior', anonymous=False)
        # print message in terminal
        rospy.loginfo('We are Live!')
        
	#informs if it is the first time we check odom
	self.init = 1
	rospy.Subscriber("robot_0/base_pose_ground_truth", Odometry, self.poseCallback)
	#subscribe to odometry topic
	rospy.Subscriber("robot_0/odom", Odometry, self.odomCallback)
	
	self.pub_odom = rospy.Publisher('odom_information', Odometry, queue_size=1)

	self.x=0
	self.y=0
	self.a=0
	self.dx=0
	self.dy=0
	self.da=0

    def poseCallback(self, msg):
	
	if self.init == 1:			#initial robot positions
	    self.x = msg.pose.pose.position.x
	    self.y = msg.pose.pose.position.y
	    w = msg.pose.pose.orientation.w	#cos(a/2)
	    z = msg.pose.pose.orientation.z	#sin(a/2)
	    if z > 0:
	    	self.a = 2*math.acos(w)		#in rad
	    else:
		self.a = -2*math.acos(w)	#in rad
	    self.init = 0;
	    print(self.x, self.y, self.a*180/math.pi)
	    
    def odomCallback(self, msg):
	self.dx = 0.1*msg.twist.twist.linear.x	#0.1s*m/s
	self.dy = 0.1*msg.twist.twist.linear.y	#0.1s*m/s
	self.da = 0.1*msg.twist.twist.angular.z	#0.1s*rad/s
	print(self.dx, self.dy, self.da)

    def publish_odom(self):
	odom_msg = Odometry()

	odom_msg.pose.pose.position.x = self.x
	odom_msg.pose.pose.position.y = self.y
	odom_msg.pose.pose.position.z = self.a

	odom_msg.pose.pose.orientation.x = self.dx
	odom_msg.pose.pose.orientation.y = self.dy
	odom_msg.pose.pose.orientation.z = self.da

	self.pub_odom.publish(odom_msg)

def main():
    my_object = reader()

    rospy.sleep(0.1)
    [robot_pose, jac_r, jac_n] = Movement(np.array([my_object.x, my_object.y, my_object.a]), np.array([my_object.dx, my_object.da]), np.array([0, 0]))
    print(robot_pose)

    while not rospy.is_shutdown():
	my_object.publish_odom()
	rospy.sleep(0.1)
	[robot_pose, jac_r, jac_n] = Movement(robot_pose, np.array([my_object.dx, my_object.da]), np.array([0, 0]))
	print(robot_pose)
