#!/usr/bin/env python

import copy, time

# Ros imports
import rospy
import numpy as np
import tf
from math import pi, sqrt, sin, cos, atan2, floor, ceil, fabs
from nav_msgs.msg import Odometry
from uav_ros_control.srv import GenerateSearch, GenerateSearchResponse
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PointStamped

from dynamic_reconfigure.server import Server
from uav_ros_control.cfg import GenerateSearchParametersConfig

class RequestSearchTrajectory():
    """ 
    This class provides search strategies for MBZIRC 2020 Challenge 1. It is used with Track-and-Follow algorithm.
    There are two types of search: line search and Point of Interest(POI) search.
    All parameters are configurable.
    """

    def __init__(self):
        """ Initialize search parameters, subscribers, publishers and service. """
        # Define initial parameters [service, dynamic, class]
        self.type = "line"
        self.z_default = 10.0

        self.searchAreaX = 100.0
        self.searchAreaY = 40.0
        self.offsetX = 0
        self.offsetY = 0
        self.initialX = 0
        self.initialY = 0
        self.arenaYawOffsetDeg = 0.0

        self.radiusPOI = 4.0
        self.zOffsetPOI = 3.0

        self.pointsNumLine = 20
        self.pointsNumPOI = 20

        self.odomAvailable = False

        # Define msgs
        self.currOdom = Odometry()
        self.currCarrotReference = MultiDOFJointTrajectoryPoint()
        self.estimatedTargetPoint = PointStamped()
        self.trajectoryPoints = MultiDOFJointTrajectory()

        # Subscribers
        self.subOdom = rospy.Subscriber("odom", Odometry, self.odometryCb)
        self.subCarrotReference = rospy.Subscriber("carrot/trajectory", MultiDOFJointTrajectoryPoint, self.carrotReferenceCb)
        self.subEstimatedTargetPoint = rospy.Subscriber("figure8/target/position_estimated", PointStamped, self.estimatedTargetPointCb)

        # Publishers
        self.pubTrajectory = rospy.Publisher("topp/input/trajectory", MultiDOFJointTrajectory, queue_size=1)

        # Service
        self.generate_search_service = rospy.Service('generate_search', GenerateSearch, self.generateSearchCallback)

        # Dynamic reconfigure
        self.srv = Server(GenerateSearchParametersConfig, self.parametersCb)

    # Callbacks

    def parametersCb(self, config, level):
        """  Dynamic reconfigure callback for parameters update.  """
        rospy.loginfo("GenerateSearch - Parameters callback.")
        self.searchAreaX = config["searchAreaX"]
        self.searchAreaY = config["searchAreaY"]
        self.offsetX = config["offsetX"]
        self.offsetY = config["offsetY"]
        self.initialX = config["initialX"]
        self.initialY = config["initialY"]
        self.arenaYawOffsetDeg = config["arenaYawOffsetDeg"]

        self.radiusPOI = config["radius"]
        self.zOffsetPOI = config["offsetZ"]

        self.updateTakeoffPoint()

        return config

    def generateSearchCallback(self, req):
        """ 
        GenerateSearch service provides search waypoints and trajectory for a trajectory publisher.

        :param type:    type of search (line or POI). 
        :param height:  constant height during line search in meters.
        """
        rospy.loginfo("GenerateSearch - Service requested.")
        res = GenerateSearchResponse()

        self.trajectoryPoints = MultiDOFJointTrajectory()

        # Retrieve parameters
        self.type = req.type
        self.z_default = req.height

        if (self.type == "line"):

            if self.odomAvailable:
                # Generate waypoints and trajectory
                self.generateLine()
                self.generateLineTrajectory()

                # Publish trajectory
                self.pubTrajectory.publish(self.trajectoryPoints)
                rospy.loginfo("GenerateSearch - Points for LINE search generated.")

                # Set flag and response
                self.odomAvailable = False
                res.success = True
            else:
                rospy.loginfo("GenerateSearch - Odometry unavailable.")
                res.success = False

        elif (self.type == "POI"):
            if self.odomAvailable:
                # Generate waypoints and trajectory
                self.generatePOITrajectory()

                # Publish trajectory
                self.pubTrajectory.publish(self.trajectoryPoints)
                rospy.loginfo("GenerateSearch - Points for POI search generated.")

                # Set flag and response
                self.odomAvailable = False
                res.success = True

            else:
                rospy.loginfo("GenerateSearch - Odometry unavailable.")
                res.success = False

        return res

    def odometryCb(self, msg):
        self.currOdom = msg
        self.odomAvailable = True

    def carrotReferenceCb(self, msg):
        self.currCarrotReference = msg

    def estimatedTargetPointCb(self, msg):
        self.estimatedTargetPoint = msg

    # Functions

    def updateTakeoffPoint(self):
        """ Convert takeoff data to trajectory point."""
        self.takeOffPoint = MultiDOFJointTrajectoryPoint()
        temp_transform = Transform()
        temp_transform.translation.x = self.initialX
        temp_transform.translation.y = self.initialY
        temp_transform.translation.z = self.z_default
        self.takeOffPoint.transforms.append(temp_transform)

    def initialPositionFromOdometry(self):
        """ Retrieve current information about UAV from odometry. """
        self.initialSearchPoint = MultiDOFJointTrajectoryPoint()
        temp_transform = Transform()
        temp_transform.translation.x = self.currOdom.pose.pose.position.x
        temp_transform.translation.y = self.currOdom.pose.pose.position.y
        temp_transform.translation.z = self.currOdom.pose.pose.position.z

        temp_transform.rotation.x = self.currOdom.pose.pose.orientation.x
        temp_transform.rotation.y = self.currOdom.pose.pose.orientation.y
        temp_transform.rotation.z = self.currOdom.pose.pose.orientation.z
        temp_transform.rotation.w = self.currOdom.pose.pose.orientation.w

        self.initialSearchPoint.transforms.append(temp_transform)

    def initialPositionFromCarrotReference(self):
        """ Retrieve current information about UAV from last carrot reference. """
        self.initialSearchPoint = MultiDOFJointTrajectoryPoint()
        self.initialSearchPoint = self.currCarrotReference

    def getInitialYaw(self):
        """ Calculate current yaw from quaternion data. """
        self.initialSearchYaw = tf.transformations.euler_from_quaternion(
            [self.initialSearchPoint.transforms[0].rotation.x, 
            self.initialSearchPoint.transforms[0].rotation.y,
            self.initialSearchPoint.transforms[0].rotation.z,
            self.initialSearchPoint.transforms[0].rotation.w])[2]

    def generateLine(self):
        """
        Create an array of points for a line search. The search is performed along the line 
        whose center is defined with takeoff point and offsets. The search line lies on 
        the longer axis of the search area.

        :param searchAreaX:     size of search area along the x axis.
        :param searchAreaY:     size of search area along the y axis.
        :param offsetX:         offset from takeoff point along the x axis.
        :param offsetY:         offset from takeoff point along the y axis.
        :param pointsNumLine:   number of points in one direction.
        """
        if (self.searchAreaX > self.searchAreaY):
            temp_x = np.linspace(self.takeOffPoint.transforms[0].translation.x - self.searchAreaX / 2.0 + self.offsetX,\
                self.takeOffPoint.transforms[0].translation.x + self.searchAreaX / 2.0 + self.offsetX, self.pointsNumLine)
            temp_y = [self.initialY]*len(temp_x)
        elif (self.searchAreaY > self.searchAreaX):
            temp_y = np.linspace(self.takeOffPoint.transforms[0].translation.y - self.searchAreaY / 2.0 + self.offsetY, \
                self.takeOffPoint.transforms[0].translation.y + self.searchAreaX / 2.0 + self.offsetY, self.pointsNumLine)
            temp_x = [self.initialX]*len(temp_y)        

        temp_z = [self.z_default]*len(temp_x)
        temp_yaw = [0]*len(temp_x)

        self.line_array = np.zeros((2*self.pointsNumLine, 4))

        for i in range(0, len(temp_x)):
            # self.line_arrayp[i][0] = temp_x[i]
            self.line_array[i][:]= np.array([temp_x[i], temp_y[i], temp_z[i], temp_yaw[i]])

        temp_yaw = [pi]*len(temp_x)

        # Backwards
        for i in range(0, len(temp_x)):
            self.line_array[i+self.pointsNumLine][:]= np.array([temp_x[-i-1], temp_y[-i-1], temp_z[-i-1], temp_yaw[i]])


    def generateLineTrajectory(self):
        """
        Based on waypoints and initial pose of UAV, create a MultiDOFJointTrajectory for a line search.
        Align the points taking into account the orientation of the arena towards the local CS of UAV. 
        Find the nearest waypoint with convenient orientation and arrange waypoints accordingly.

        :param arenaYawOffsetDeg:   arena orientation in local CS of UAV.

        """
        # Add my position
        self.initialPositionFromCarrotReference()
        self.getInitialYaw()
        self.trajectoryPoints.points.append(self.initialSearchPoint)

        dist = []
        temp_yaw = []

        # Align with arena orientation
        theta = np.radians(self.arenaYawOffsetDeg)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))

        for i in range(len(self.line_array)):
            self.line_array[i][3] += self.arenaYawOffsetDeg * pi / 180
            self.line_array[i][:2] = (R.dot(self.line_array[i][:2].reshape(2,1))).flatten()

        # Get current yaw
        yaw_start = tf.transformations.euler_from_quaternion(
            [self.currOdom.pose.pose.orientation.x, 
            self.currOdom.pose.pose.orientation.y,
            self.currOdom.pose.pose.orientation.z,
            self.currOdom.pose.pose.orientation.w])[2]

        # Compare current position and orientation with points in line_array
        for i in range(0,len(self.line_array)):
            dist.append(np.linalg.norm(self.line_array[i][:2]-[self.initialSearchPoint.transforms[0].translation.x, self.initialSearchPoint.transforms[0].translation.y]))
            temp_yaw.append(pi-abs(abs(yaw_start -self.line_array[i][3])-pi))

        # Index of two nearest points in line array with opposite orientation
        idx = np.argpartition(dist, 2)

        # Select one point with an orientation closer to the current yaw
        if (temp_yaw[idx[0]] < temp_yaw[idx[1]]):
            self.startInTrajectoryIdx = idx[0]
        else:
            self.startInTrajectoryIdx = idx[1]

        # Create trajectory points starting from the nearest
        for i in range(self.startInTrajectoryIdx, 2*self.pointsNumLine):

            if (i == self.startInTrajectoryIdx):
                delta = self.initialSearchYaw - self.line_array[i][3]
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

    def generatePOITrajectory(self):
        """
        Create a MultiDOFJointTrajectory for a circle search around Point of Interest (POI).
        Point of Interest is a estimated pose of a target.

        :param radiusPOI:       radius of the circle around POI.
        :param zOffsetPOI:      offset along the z axis that is added to the height of the target.
        :param pointsNumPOI:    number of points for one circle.

        """
        self.initialPositionFromCarrotReference()
        self.getInitialYaw()
        self.trajectoryPoints.points.append(self.initialSearchPoint)

        x = [0] * self.pointsNumPOI
        y = [0] * self.pointsNumPOI
        z = [0] * self.pointsNumPOI
        yaw = [0] * self.pointsNumPOI

        theta = np.linspace(0 , 2*pi, self.pointsNumPOI)

        for i in range(self.pointsNumPOI):
            x[i] = self.estimatedTargetPoint.point.x + self.radiusPOI * cos(theta[i])
            y[i] = self.estimatedTargetPoint.point.y + self.radiusPOI * sin(theta[i])
            z[i] = self.estimatedTargetPoint.point.z + self.zOffsetPOI

            yaw[i] = atan2((self.estimatedTargetPoint.point.y - y[i]),
                self.estimatedTargetPoint.point.x - x[i])

        for i in range (self.pointsNumPOI):

            if (i == 0):
                delta = self.initialSearchYaw - yaw[i]
            else:
                delta = yaw[i-1] - yaw[i]

            if (delta > pi):
                yaw[i] += ceil(floor(fabs(delta)/pi)/(2.0))*2.0*pi
            elif (delta < -pi):
                yaw[i] -= ceil(floor(fabs(delta)/pi)/(2.0))*2.0*pi

            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = x[i]
            temp_transform.translation.y = y[i]
            temp_transform.translation.z = z[i]

            q_backwards = tf.transformations.quaternion_from_euler(0, 0, yaw[i], 'rxyz')

            temp_transform.rotation.x = q_backwards[0]
            temp_transform.rotation.y = q_backwards[1]
            temp_transform.rotation.z = q_backwards[2]
            temp_transform.rotation.w = q_backwards[3]
            temp_point.transforms.append(temp_transform)

            self.trajectoryPoints.points.append(temp_point)

    def run(self):
        """ A loop waiting for a service request. """
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            rospy.spin()
            r.sleep()

if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("generate_search")

    trajectory = RequestSearchTrajectory()
    trajectory.run()