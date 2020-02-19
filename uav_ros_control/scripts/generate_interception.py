#!/usr/bin/env python

import copy, time

# Ros imports
import rospy
import numpy as np
import tf
from math import pi
from nav_msgs.msg import Odometry
from uav_ros_control.srv import GenerateInterception, GenerateInterceptionResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist


class RequestInterceptionTrajectory():

    def __init__(self):
        # Service
        self.generate_search_service = rospy.Service('generate_interception', GenerateInterception, self.generateInterceptionCallback)

        # Subscribers
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)

        # Publishers
        self.trajectory_pub = rospy.Publisher("topp/input/trajectory", MultiDOFJointTrajectory, queue_size=1)

        # # Define msg for points
        # self.trajectoryPoints = MultiDOFJointTrajectory()

        # Odometry msg
        self.odom_msg = Odometry()
        self.odom_flag = False

        # Define height


        # # Takeo Off point
        # self.takeOffPoint = MultiDOFJointTrajectoryPoint()
        # temp_transform = Transform()
        # temp_transform.translation.x = -40.0
        # temp_transform.translation.y = 0.0
        # temp_transform.translation.z = self.z_default
        # temp_transform.rotation.w = 1.0

        # self.takeOffPoint.transforms.append(temp_transform)


    def generateInterceptionCallback(self, req):
        print "Generating interception trajectory."
        res = GenerateInterceptionResponse()

        # Define msg for points
        self.trajectoryPoints = MultiDOFJointTrajectory()

        # fail

        self.interception_point = MultiDOFJointTrajectoryPoint()
        temp_transform = Transform()
        temp_transform.translation.x = req.interception_point.pose.position.x
        temp_transform.translation.y = req.interception_point.pose.position.y
        temp_transform.translation.z = req.interception_point.pose.position.z

        temp_transform.rotation.x = req.interception_point.pose.orientation.x
        temp_transform.rotation.y = req.interception_point.pose.orientation.y
        temp_transform.rotation.z = req.interception_point.pose.orientation.z
        temp_transform.rotation.w = req.interception_point.pose.orientation.w

        self.interception_point.transforms.append(temp_transform)

        if self.odom_flag:
            self.getMyPosition()
            self.trajectoryPoints.points.append(self.starting_point)
            self.trajectoryPoints.points.append(self.interception_point)

            self.trajectory_pub.publish(self.trajectoryPoints)
            self.odom_flag = False
            res.success = True
        else:
            print "GenerateInterception - Odometry unavailable."
            res.success = False

        return res

    def odom_cb(self, msg):
        self.odom_msg = msg
        self.odom_flag = True

    def run(self):
        #  Waiting for service request
        rospy.spin()


    def getMyPosition(self):
        self.starting_point = MultiDOFJointTrajectoryPoint()
        temp_transform = Transform()
        temp_transform.translation.x = self.odom_msg.pose.pose.position.x
        temp_transform.translation.y = self.odom_msg.pose.pose.position.y
        temp_transform.translation.z = self.odom_msg.pose.pose.position.z

        yaw_start = tf.transformations.euler_from_quaternion(
            [self.odom_msg.pose.pose.orientation.x, 
            self.odom_msg.pose.pose.orientation.y,
            self.odom_msg.pose.pose.orientation.z,
            self.odom_msg.pose.pose.orientation.w])[2]

        temp_transform.rotation.x = self.odom_msg.pose.pose.orientation.x
        temp_transform.rotation.y = self.odom_msg.pose.pose.orientation.y
        temp_transform.rotation.z = self.odom_msg.pose.pose.orientation.z
        temp_transform.rotation.w = self.odom_msg.pose.pose.orientation.w

        self.starting_point.transforms.append(temp_transform)

if __name__ == "__main__":
    rospy.init_node("generate_interception")
    trajectory = RequestInterceptionTrajectory()
    # trajectory.start()
    trajectory.run()