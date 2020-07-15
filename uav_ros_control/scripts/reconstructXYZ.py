#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from uav_object_tracking_msgs.msg import object
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header, Bool, Float32, Int32
from nav_msgs.msg import Odometry
import math
import numpy as np
from numpy.linalg import inv
from math import sqrt, sin, cos, pi, atan, floor, atan2

import message_filters

class reconstructXYZ():
    def __init__(self):
        self.camera_info = CameraInfo()
        self.camera_position_offset = 0.25

        self.new_depth_data = False
        self.depth_data = Float32()

        self.new_detection_data = False
        self.detection_data = object()

        self.new_odometry_data = False
        self.odometry_data = Odometry()

        self.new_target_pose = False
        self.target_pose = PoseStamped()

        self.new_gimbal_data = False
        self.gimbal_pose = PoseStamped()

        self.target_uav_cs = PointStamped()
        self.target_world_cs = PointStamped()

        # Camera coordinates to UAV coordinates
        self.Tuav_cam = np.zeros((4, 4))
        self.Tuav_cam[0, 2] = 1.0
        self.Tuav_cam[1, 0] = -1.0
        self.Tuav_cam[2, 1] = -1.0
        self.Tuav_cam[3, 3] = 1.0

        self.camera_info.K = [674.0643310546875, 0.0, 655.4468994140625, 0.0, 674.0643310546875, 368.7719421386719, 0.0, 0.0, 1.0]

        # Create subscribers
        rospy.Subscriber('/uav_object_tracking/uav/depth_kf', Float32, self.distance_callback)
        rospy.Subscriber('/YOLODetection/tracked_detection', object, self.detection_callback)
        rospy.Subscriber('/yellow/mavros/global_position/local', Odometry, self.odometry_callback)
        rospy.Subscriber('/yellow/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        #rospy.Subscriber('/zedm/zed_node/left/camera_info', CameraInfo, figure8.camera_info_callback)

        # Msgs filters
        target_pose_sub = message_filters.Subscriber('/uav/pose', PoseStamped)
        gimbal_pose_sub = message_filters.Subscriber('/gimbal/pose', PoseStamped)

        self.ts = message_filters.ApproximateTimeSynchronizer([target_pose_sub, gimbal_pose_sub], 10, 0.005)
        self.ts.registerCallback(self.gt_callback)

        # Create publishers
        self.target_world_cs_pub = rospy.Publisher('reconstructXYZ/world_cs/position_estimated', PointStamped, queue_size = 1)
        self.target_uav_cs_pub = rospy.Publisher('reconstructXYZ/uav_cs/position_estimated', PointStamped, queue_size = 1)

        self.depth_relative_pub = rospy.Publisher('reconstructXYZ/depth_relative', Float32, queue_size = 1)
        self.height_relative_pub = rospy.Publisher('reconstructXYZ/height_relative', Float32, queue_size = 1)
        self.yaw_relative_pub = rospy.Publisher('reconstructXYZ/yaw_relative', Float32, queue_size = 1)

        self.target_map_pub = rospy.Publisher('reconstructXYZ/debug/target', PointStamped, queue_size=1)
        self.depth_gt_pub = rospy.Publisher('reconstructXYZ/depth_gt', Float32, queue_size = 1)

    """ Callbacks """

    def camera_info_callback(self, data):
        self.camera_info = data

    def distance_callback(self, data):
        self.new_depth_data = True
        self.depth_data = data
        
    def detection_callback(self, data):
        self.new_detection_data = True
        self.detection_data = data

    def odometry_callback(self, data):
        self.new_odometry_data = True
        self.odometry_data = data

    def gt_callback(self, target, gimbal):
        self.target_pose = target
        self.gimbal_pose = gimbal

        self.new_gimbal_data = True
        self.new_target_pose = True

    def quaternion2euler(self, quaternion):

        euler = [0.0, 0.0, 0.0]

        euler[0] = math.atan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]), 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]))

        euler[1] = math.asin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]))

        euler[2] = math.atan2(2 * (quaternion[0]*quaternion[3] + quaternion[1]*quaternion[2]), 1 - 2 * (quaternion[2]*quaternion[2] + quaternion[3] * quaternion[3]))

        return euler

    def euler2quaternion(self, euler):
        quaternion = [1.0, 0.0, 0.0, 0.0]

        cy = cos(euler[2] * 0.5)
        sy = sin(euler[2] * 0.5)
        cr = cos(euler[0] * 0.5)
        sr = sin(euler[0] * 0.5)
        cp = cos(euler[1] * 0.5)
        sp = sin(euler[1] * 0.5)

        quaternion[0] = cy * cr * cp + sy * sr * sp; #w
        quaternion[1] = cy * sr * cp - sy * cr * sp; #x
        quaternion[2] = cy * cr * sp + sy * sr * cp; #y
        quaternion[3] = sy * cr * cp - cy * sr * sp; #z

        return quaternion

    def getRotationTranslationMatrix(self, orientationEuler, position):
        """
        Rotation & Translation Matrix from UAV to world coordinates.
        """
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

    def transform2global(self, t_x, t_y, t_z):

        Tcam_uav_t= np.zeros((4, 4))
        Tcam_uav_t[0, 0] = 1.0
        Tcam_uav_t[0, 3] = t_x
        Tcam_uav_t[1, 1] = 1.0
        Tcam_uav_t[1, 3] = t_y
        Tcam_uav_t[2, 2] = 1.0
        Tcam_uav_t[2, 3] = t_z
        Tcam_uav_t[3, 3] = 1.0

        quaternion = [self.odometry_data.pose.pose.orientation.w, self.odometry_data.pose.pose.orientation.x, self.odometry_data.pose.pose.orientation.y, self.odometry_data.pose.pose.orientation.z]
        euler = self.quaternion2euler(quaternion)
        position = [self.odometry_data.pose.pose.position.x, self.odometry_data.pose.pose.position.y, self.odometry_data.pose.pose.position.z]

        Tworld_uav = self.getRotationTranslationMatrix(euler, position)

        # Transform from camera coordinates to UAV coordinates
        Tuav_uav1 = np.dot(self.Tuav_cam, Tcam_uav_t)

        # Publish target position in UAV CS
        self.target_uav_cs = PointStamped()
        self.target_uav_cs.header.frame_id = "bebop/base_link"
        self.target_uav_cs.header.stamp = rospy.Time.now()
        self.target_uav_cs.point.x = Tuav_uav1[0,3]
        self.target_uav_cs.point.y = Tuav_uav1[1,3]
        self.target_uav_cs.point.z = Tuav_uav1[2,3]

        self.target_uav_cs_pub.publish(self.target_uav_cs)

        """ Publish relative values for Visual-servoing """

        depth_r_msg = Float32()
        depth_r_msg.data = Tuav_uav1[0,3]
        self.depth_relative_pub.publish(depth_r_msg)

        height_r_msg = Float32()
        height_r_msg.data = -1 * Tuav_uav1[2,3]
        self.height_relative_pub.publish(height_r_msg)

        yaw_r_msg = Float32()
        yaw_r_msg.data = atan2(Tuav_uav1[1,3], Tuav_uav1[0,3])
        self.yaw_relative_pub.publish(yaw_r_msg)

        # Tranform from UAV coordinates to World coordinates
        Tworld_uav1 = np.dot(Tworld_uav, Tuav_uav1)

        # Publish target position in World CS
        self.target_world_cs = PointStamped()
        self.target_world_cs.header.frame_id = "map"
        self.target_world_cs.header.stamp = rospy.Time.now()
        self.target_world_cs.point.x = Tworld_uav1[0,3]
        self.target_world_cs.point.y = Tworld_uav1[1,3]
        self.target_world_cs.point.z = Tworld_uav1[2,3]

        self.target_world_cs_pub.publish(self.target_world_cs)

    def getTrueDepth(self):
        """
        Transform ground truth data of the target's pose to camera frame to get depth.
        """
        #euler = [0.0, 0.0, 1.5707963268] # if you want to use mavros data
        euler = [0.0, 0.0, 0.0]
        position = [0.0 , 0.0, 0.0]

        Tpoint= np.zeros((4, 4))
        Tpoint[0, 0] = 1.0
        Tpoint[0, 3] = self.target_pose.pose.position.x
        Tpoint[1, 1] = 1.0
        Tpoint[1, 3] = self.target_pose.pose.position.y
        Tpoint[2, 2] = 1.0
        Tpoint[2, 3] = self.target_pose.pose.position.z
        Tpoint[3, 3] = 1.0

        # Transform world to map
        Tmap_world = self.getRotationTranslationMatrix(euler, position)

        Tpoint_map = np.dot(Tmap_world, Tpoint)

        target_map = PointStamped()
        target_map.point.x = Tpoint_map[0, 3]
        target_map.point.y = Tpoint_map[1, 3]
        target_map.point.z = Tpoint_map[2, 3]
        target_map.header.stamp = rospy.Time.now()
        target_map.header.frame_id = "map"

        self.target_map_pub.publish(target_map)

        # quaternion = [self.odometry_data.pose.pose.orientation.w, self.odometry_data.pose.pose.orientation.x, self.odometry_data.pose.pose.orientation.y, self.odometry_data.pose.pose.orientation.z]
        quaternion = [self.gimbal_pose.pose.orientation.w, self.gimbal_pose.pose.orientation.x, self.gimbal_pose.pose.orientation.y, self.gimbal_pose.pose.orientation.z]
        euler = self.quaternion2euler(quaternion)
        # position = [self.odometry_data.pose.pose.position.x, self.odometry_data.pose.pose.position.y, self.odometry_data.pose.pose.position.z]
        position = [self.gimbal_pose.pose.position.x, self.gimbal_pose.pose.position.y, self.gimbal_pose.pose.position.z]
        Tworld_uav = self.getRotationTranslationMatrix(euler, position)

        Tpoint_uav = np.dot(np.linalg.inv(Tworld_uav), Tpoint_map)

        Tpoint_cam = np.dot(np.linalg.inv(self.Tuav_cam), Tpoint_uav)

        depth_gt_msg = Float32()
        depth_gt_msg.data = Tpoint_cam[2,3] #- self.camera_position_offset
        self.depth_gt_pub.publish(depth_gt_msg)

        self.new_target_pose = False
        self.new_gimbal_data = False

    def run(self, rate):
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():

            if self.new_target_pose and self.new_gimbal_data:
                self.getTrueDepth()

            if self.new_depth_data and self.new_detection_data and self.new_odometry_data:
                """
                x = (u - cx) * depth / fx
                y = (v - cy) * depth / fy
                z = depth
                """
                x_3d = (self.detection_data.x - self.camera_info.K[2]) * self.depth_data.data / self.camera_info.K[0]
                y_3d = (self.detection_data.y - self.camera_info.K[5]) * self.depth_data.data / self.camera_info.K[4]
                z_3d = self.depth_data.data

                self.transform2global(x_3d, y_3d, z_3d)

                # Reset conditions
                self.new_depth_data = False
                self.new_detection_data = False
                self.new_odometry_data = False

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('reconstructXYZ')

    reconstructor = reconstructXYZ()
    reconstructor.run(50)
