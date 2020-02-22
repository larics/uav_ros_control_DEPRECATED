#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from uav_ros_control.srv import GiveMeBalloonPosition, GiveMeBalloonPositionResponse

import math
import numpy as np

import copy


class BalloonEstimator():
    def __init__(self):
        self.odometry_data_list = []
        self.list_size = 50
        self.new_odometry_data = False
        self.new_distance_data = False
        self.camera_info_flag = False

        self.Tuav_cam = np.zeros((4, 4))
        self.Tuav_cam[0, 2] = 1.0
        self.Tuav_cam[1, 0] = -1.0
        self.Tuav_cam[2, 1] = -1.0
        self.Tuav_cam[3, 3] = 1.0

        self.camera_info = CameraInfo()
        self.distance_data = PoseStamped()
        self.balloons = []
        self.balloons_average = []

        self.camera_info.K = [674.0643310546875, 0.0, 655.4468994140625, 0.0, 674.0643310546875, 368.7719421386719, 0.0, 0.0, 1.0]
        self.z_offset = 1.0
        self.max_allowed_distance = 5.0
        self.min_balloon_measurements = 30

    def odometry_callback(self, data):
        self.new_odometry_data = True
        if (len(self.odometry_data_list) < self.list_size):
            self.odometry_data_list.append(data)
        else:
            self.odometry_data_list.append(data)
            self.odometry_data_list.pop(0)

    def camera_info_callback(self, data):
        self.camera_info_flag = True
        self.camera_info = data

    def distance_callback(self, data):
        if (data.pose.position.z > 0.0):
            self.new_distance_data = True
            self.distance_data = data

    def find_min_time_diff_data(self, data, time_s):
        min_value = 0.0
        return_data = data[0]
        for i in range(len(data)):
            diff = math.fabs(data[i].header.stamp.to_sec() - time_s)

            if (i == 0):
                min_value = diff
                return_data = data[i]
            else:
                if (min_value > diff):
                    min_value = diff
                    return_data = data[i]
                    index = i
        return return_data

    def set_uav_setpoint_publisher(self, publisher):
        self.uav_setpoint_publisher = publisher

    def quaternion2euler(self, quaternion):

        euler = [0.0, 0.0, 0.0]

        euler[0] = math.atan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]), 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]))

        euler[1] = math.asin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]))

        euler[2] = math.atan2(2 * (quaternion[0]*quaternion[3] + quaternion[1]*quaternion[2]), 1 - 2 * (quaternion[2]*quaternion[2] + quaternion[3] * quaternion[3]))

        return euler


    def getRotationTranslationMatrix(self, orientationEuler, position):

        x = orientationEuler[0]
        y = orientationEuler[1]
        z = orientationEuler[2]

        r11 = math.cos(y)*math.cos(z)

        r12 = math.cos(z)*math.sin(x)*math.sin(y) - math.cos(x)*math.sin(z)

        r13 = math.sin(x)*math.sin(z) + math.cos(x)*math.cos(z)*math.sin(y)

        r21 = math.cos(y)*math.sin(z)

        r22 = math.cos(x)*math.cos(z) + math.sin(x)*math.sin(y)*math.sin(z)

        r23 = math.cos(x)*math.sin(y)*math.sin(z) - math.cos(z)*math.sin(x)

        r31 = -math.sin(y)

        r32 = math.cos(y)*math.sin(x)

        r33 = math.cos(x)*math.cos(y)

        t1 = position[0]
        t2 = position[1]
        t3 = position[2]


        rotationTranslationMatrix = np.zeros((4, 4))

        rotationTranslationMatrix[0, 0] = r11
        rotationTranslationMatrix[0, 1] = r12
        rotationTranslationMatrix[0, 2] = r13
        rotationTranslationMatrix[0, 3] = t1

        rotationTranslationMatrix[1, 0] = r21
        rotationTranslationMatrix[1, 1] = r22
        rotationTranslationMatrix[1, 2] = r23
        rotationTranslationMatrix[1, 3] = t2

        rotationTranslationMatrix[2, 0] = r31
        rotationTranslationMatrix[2, 1] = r32
        rotationTranslationMatrix[2, 2] = r33
        rotationTranslationMatrix[2, 3] = t3

        rotationTranslationMatrix[3, 0] = 0.0
        rotationTranslationMatrix[3, 1] = 0.0
        rotationTranslationMatrix[3, 2] = 0.0
        rotationTranslationMatrix[3, 3] = 1.0

        return rotationTranslationMatrix

    def give_me_balloon_callback(self, req):
        res = GiveMeBalloonPositionResponse()
        res.status = False
        min_dist = -1.0
        index = 0

        for i in range(len(self.balloons)):
            if (len(self.balloons[i]) > self.min_balloon_measurements):
                odometry = self.odometry_data_list[-1]

                dx = odometry.pose.pose.position.x - self.balloons_average[i].point.x
                dy = odometry.pose.pose.position.y - self.balloons_average[i].point.y
                dz = odometry.pose.pose.position.z - self.balloons_average[i].point.z

                distance = math.sqrt(dx*dx + dy*dy + dz*dz)

                if (min_dist < 0.0):
                    min_dist = distance
                    index = i
                
                if (distance < min_dist):
                    min_dist = distance
                    index = i

        if (min_dist > 0.0):
            res.status = True
            res.balloon_position = self.balloons_average[index]
            res.balloon_position.point.z = res.balloon_position.point.z + self.z_offset
            self.balloons.pop(index)
            self.balloons_average.pop(index)

        return res

    def run(self, rate):
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            odometry_data = Odometry()

            if self.new_distance_data and self.new_odometry_data and self.camera_info_flag:
                
                odometry_data = self.find_min_time_diff_data(copy.deepcopy(self.odometry_data_list), self.distance_data.header.stamp.to_sec())
                
                x = (self.distance_data.pose.position.x - self.camera_info.K[2]) / self.camera_info.K[0]
                y = (self.distance_data.pose.position.y - self.camera_info.K[5]) / self.camera_info.K[4]

                z_m = self.distance_data.pose.position.z / (math.sqrt(1 + x*x + y*y))
                x_m = x * z_m
                y_m = y * z_m

                Tcam_balloon = np.zeros((4, 4))
                Tcam_balloon[0, 0] = 1.0
                Tcam_balloon[0, 3] = x_m
                Tcam_balloon[1, 1] = 1.0
                Tcam_balloon[1, 3] = y_m
                Tcam_balloon[2, 2] = 1.0
                Tcam_balloon[2, 3] = z_m
                Tcam_balloon[3, 3] = 1.0

                quaternion = [odometry_data.pose.pose.orientation.w, odometry_data.pose.pose.orientation.x, odometry_data.pose.pose.orientation.y, odometry_data.pose.pose.orientation.z]
                euler = self.quaternion2euler(quaternion)
                position = [odometry_data.pose.pose.position.x, odometry_data.pose.pose.position.y, odometry_data.pose.pose.position.z]
                Tworld_uav = self.getRotationTranslationMatrix(euler, position)


                Tuav_balloon = np.dot(self.Tuav_cam, Tcam_balloon)
                Tworld_balloon = np.dot(Tworld_uav, Tuav_balloon)

                balloon_position = PointStamped()
                balloon_position.header = self.distance_data.header
                balloon_position.point.x = Tworld_balloon[0,3]
                balloon_position.point.y = Tworld_balloon[1,3]
                balloon_position.point.z = Tworld_balloon[2,3]

                if (len(self.balloons) == 0):
                    balloon = []
                    balloon.append(balloon_position)
                    self.balloons.append(balloon)
                    self.balloons_average.append(balloon_position)
                else:
                    new_balloon_meas = False
                    for i in range(len(self.balloons)):

                        dx = balloon_position.point.x - self.balloons_average[i].point.x
                        dy = balloon_position.point.y - self.balloons_average[i].point.y
                        dz = balloon_position.point.z - self.balloons_average[i].point.z

                        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

                        if (distance < self.max_allowed_distance):
                            self.balloons[i].append(balloon_position)

                            avg_x = 0.0
                            avg_y = 0.0
                            avg_z = 0.0
                            for j in range(len(self.balloons[i])):
                                avg_x = avg_x + self.balloons[i][j].point.x
                                avg_y = avg_y + self.balloons[i][j].point.y
                                avg_z = avg_z + self.balloons[i][j].point.z

                            avg_x = avg_x / len(self.balloons[i])
                            avg_y = avg_y / len(self.balloons[i])
                            avg_z = avg_z / len(self.balloons[i])

                            self.balloons_average[i].header.stamp = rospy.get_rostime()
                            self.balloons_average[i].header.frame_id = "world"
                            self.balloons_average[i].point.x = avg_x
                            self.balloons_average[i].point.y = avg_y
                            self.balloons_average[i].point.z = avg_z

                            new_balloon_meas = True

                            break

                    if (not new_balloon_meas):
                        balloon = []
                        balloon.append(balloon_position)
                        self.balloons.append(balloon)
                        self.balloons_average.append(balloon_position)

                self.new_distance_data = False
                self.new_odometry_data = False
                
                self.uav_setpoint_publisher.publish(balloon_position)

            r.sleep()




if __name__ == '__main__':
    rospy.init_node('balloonEstimator')

    balloons = BalloonEstimator()

    rospy.Subscriber('/blue/mavros/global_position/local', Odometry, balloons.odometry_callback)
    rospy.Subscriber('/red/camera/color/camera_info', CameraInfo, balloons.camera_info_callback)
    rospy.Subscriber('/blue/green_color_filter/balloon_pose', PoseStamped, balloons.distance_callback)

    uav_setpoint = rospy.Publisher('/blue/target_balloon/setpoint_estimated', PointStamped, queue_size=1)

    rospy.Service('give_me_balloon', GiveMeBalloonPosition, balloons.give_me_balloon_callback)

    balloons.set_uav_setpoint_publisher(uav_setpoint)

    balloons.run(50)