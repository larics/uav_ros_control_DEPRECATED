#!/usr/bin/env python

import copy, time

# Ros imports
import rospy
import numpy as np
import tf
from math import pi, sqrt, sin, cos, atan2, floor, ceil, fabs
from nav_msgs.msg import Odometry
from uav_ros_control.srv import GenerateSearch, GenerateSearchResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PointStamped

# from uav_search.msg import *
# from uav_search.srv import *


class RequestSearchTrajectory():

    def __init__(self):
        # Service
        self.generate_search_service = rospy.Service('generate_search', GenerateSearch, self.generateSearchCallback)

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
        self.z_default = 3.0

        # Define arena dimensions
        self.x_size = 100.0
        self.y_size = 40.0

        self.x_offset = 20
        self.y_offset = 10

        # Number of points for trajectory
        self.numPoints = 200

        # # Takeo Off point
        # self.takeOffPoint = MultiDOFJointTrajectoryPoint()
        # temp_transform = Transform()
        # temp_transform.translation.x = -40.0
        # temp_transform.translation.y = 0.0
        # temp_transform.translation.z = self.z_default
        # temp_transform.rotation.w = 1.0

        # self.takeOffPoint.transforms.append(temp_transform)


    def generateSearchCallback(self, req):
        print "Generating search trajectory."
        res = GenerateSearchResponse()

        # Define msg for points
        self.trajectoryPoints = MultiDOFJointTrajectory()

        # fail

        #get params
        self.z_default = req.desired_height
        self.x_size = req.x_size
        self.y_size = req.y_size
        self.x_offset = req.x_offset
        self.y_offset = req.y_offset
        self.x_takeOff = req.x_takeOff
        self.y_takeOff = req.y_takeOff
        self.scan_yaw_offset_deg = req.yaw_offset

        # Takeo Off point
        self.takeOffPoint = MultiDOFJointTrajectoryPoint()
        temp_transform = Transform()
        temp_transform.translation.x = req.x_takeOff
        temp_transform.translation.y = req.y_takeOff
        temp_transform.translation.z = self.z_default

        self.takeOffPoint.transforms.append(temp_transform)

        if self.odom_flag:
            self.getMyPosition()
            self.generateLine()
            self.modifyTrajectory()

            self.trajectory_pub.publish(self.trajectoryPoints)
            self.odom_flag = False
            res.success = True
            print "GenerateSearch - Points generated"
        else:
            print "GenerateSearch - Odometry unavailable."
            res.success = False

        return res

    def odom_cb(self, msg):
        self.odom_msg = msg
        self.odom_flag = True


    # def start(self):
    #     # Starting point = odom
    #     rospy.sleep(0.5)
    #     while not rospy.is_shutdown():
    #         if self.odom_flag:
    #             self.getMyPosition()
    #             self.generateLine()
    #             self.modifyTrajectory()

    #             self.trajectory_pub.publish(self.trajectoryPoints)
    #             return
    #         rospy.sleep(0.01)

    def run(self):
        #  Waiting for service request
        rospy.spin()


    def getMyPosition(self):
        self.starting_point = MultiDOFJointTrajectoryPoint()
        temp_transform = Transform()
        temp_transform.translation.x = self.odom_msg.pose.pose.position.x
        temp_transform.translation.y = self.odom_msg.pose.pose.position.y
        temp_transform.translation.z = self.odom_msg.pose.pose.position.z

        self.yaw_starting = tf.transformations.euler_from_quaternion(
            [self.odom_msg.pose.pose.orientation.x, 
            self.odom_msg.pose.pose.orientation.y,
            self.odom_msg.pose.pose.orientation.z,
            self.odom_msg.pose.pose.orientation.w])[2]

        temp_transform.rotation.x = self.odom_msg.pose.pose.orientation.x
        temp_transform.rotation.y = self.odom_msg.pose.pose.orientation.y
        temp_transform.rotation.z = self.odom_msg.pose.pose.orientation.z
        temp_transform.rotation.w = self.odom_msg.pose.pose.orientation.w

        self.starting_point.transforms.append(temp_transform)

    def getToCenter(self):

        self.x_center = -48.0
        self.y_center = 0.0
        self.z_center = 10.0

        temp_point = MultiDOFJointTrajectoryPoint()
        temp_transform = Transform()
        temp_transform.translation.x = self.x_center
        temp_transform.translation.y = self.y_center
        temp_transform.translation.z = self.z_center

        temp_transform.rotation.w = 1.0
        temp_point.transforms.append(temp_transform)

        self.trajectoryPoints.points.append(temp_point)

    def generateLine(self):
        if (self.x_size > self.y_size):
            temp_x = np.linspace(self.takeOffPoint.transforms[0].translation.x + self.x_offset, self.takeOffPoint.transforms[0].translation.x + (self.x_size-self.x_offset), self.numPoints)
            temp_y = [self.y_takeOff]*len(temp_x)
        elif (self.y_size > self.x_size):
            temp_y = np.linspace(self.takeOffPoint.transforms[0].translation.y + self.y_offset, self.takeOffPoint.transforms[0].translation.y + (self.y_size-self.y_offset), self.numPoints)
            temp_x = [self.x_takeOff]*len(temp_y)

        temp_z = [self.z_default]*len(temp_x)
        temp_yaw = [0]*len(temp_x)

        self.line_array = np.zeros((2*self.numPoints, 4))

        for i in range(0, len(temp_x)):
            # self.line_arrayp[i][0] = temp_x[i]
            self.line_array[i][:]= np.array([temp_x[i], temp_y[i], temp_z[i], temp_yaw[i]])

        temp_yaw = [pi]*len(temp_x)

        # Backwards
        for i in range(0, len(temp_x)):
            self.line_array[i+self.numPoints][:]= np.array([temp_x[-i-1], temp_y[-i-1], temp_z[-i-1], temp_yaw[i]])

    def modifyTrajectory(self):
        dist = []
        temp_yaw = []

        # Align with arena orientation
        theta = np.radians(self.scan_yaw_offset_deg)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))

        for i in range(len(self.line_array)):
            self.line_array[i][3] += self.scan_yaw_offset_deg * pi / 180
            self.line_array[i][:2] = (R.dot(self.line_array[i][:2].reshape(2,1))).flatten()

        # Get current yaw
        yaw_start = tf.transformations.euler_from_quaternion(
            [self.odom_msg.pose.pose.orientation.x, 
            self.odom_msg.pose.pose.orientation.y,
            self.odom_msg.pose.pose.orientation.z,
            self.odom_msg.pose.pose.orientation.w])[2]

        # Compare current position and orientation with points in line_array
        for i in range(0,len(self.line_array)):
            dist.append(np.linalg.norm(self.line_array[i][:2]-[self.starting_point.transforms[0].translation.x, self.starting_point.transforms[0].translation.y]))
            temp_yaw.append(pi-abs(abs(yaw_start -self.line_array[i][3])-pi))

        # Index of two nearest points in line array with opposite orientation
        idx = np.argpartition(dist, 2)
        print(temp_yaw[idx[0]], temp_yaw[idx[1]])

        # Select one point with an orientation closer to the current yaw
        if (temp_yaw[idx[0]] < temp_yaw[idx[1]]):
            self.startInTrajectoryIdx = idx[0]
        else:
            self.startInTrajectoryIdx = idx[1]

        # Add my position
        self.trajectoryPoints.points.append(self.starting_point)


        for i in range(self.startInTrajectoryIdx, 2*self.numPoints):

            if (i == self.startInTrajectoryIdx):
                delta = self.yaw_starting - self.line_array[i][3]
            else:
                delta = self.line_array[i-1][3] - self.line_array[i][3]

            if (delta > pi):
                self.line_array[i][3] += ceil(floor(fabs(delta)/pi)/(2.0))*2.0*pi
            elif (delta < -pi):
                self.line_array[i][3] -= ceil(floor(fabs(delta)/pi)/(2.0))*2.0*pi


            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = self.line_array[i][0]
            temp_transform.translation.y = self.line_array[i][1]
            temp_transform.translation.z = self.line_array[i][2]

            q_backwards = tf.transformations.quaternion_from_euler(0, 0, self.line_array[i][3], 'rxyz')

            temp_transform.rotation.x = q_backwards[0]
            temp_transform.rotation.y = q_backwards[1]
            temp_transform.rotation.z = q_backwards[2]
            temp_transform.rotation.w = q_backwards[3]
            temp_point.transforms.append(temp_transform)

            self.trajectoryPoints.points.append(temp_point)

        for i in range(0, self.startInTrajectoryIdx):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = self.line_array[i][0]
            temp_transform.translation.y = self.line_array[i][1]
            temp_transform.translation.z = self.line_array[i][2]

            q_backwards = tf.transformations.quaternion_from_euler(0, 0, self.line_array[i][3], 'rxyz')

            temp_transform.rotation.x = q_backwards[0]
            temp_transform.rotation.y = q_backwards[1]
            temp_transform.rotation.z = q_backwards[2]
            temp_transform.rotation.w = q_backwards[3]
            temp_point.transforms.append(temp_transform)

            self.trajectoryPoints.points.append(temp_point)


if __name__ == "__main__":
    rospy.init_node("generate_search")
    trajectory = RequestSearchTrajectory()
    # trajectory.start()
    trajectory.run()
