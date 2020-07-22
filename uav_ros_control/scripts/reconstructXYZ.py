#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from uav_object_tracking_msgs.msg import object
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header, Bool, Float32
from nav_msgs.msg import Odometry
import math
import numpy as np
from math import sqrt, sin, cos, pi, atan, floor, atan2

import message_filters

from PositionKalmanFilter import PositionKalmanFilter
from LocalPositionKalmanFilter import LocalPositionKalmanFilter

from dynamic_reconfigure.server import Server
from uav_ros_control.cfg import PositionKalmanFilterParametersConfig

class reconstructXYZ():
    def __init__(self):
        self.camera_info = CameraInfo()

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

        self.target_vel_gt_data = TwistStamped()
        self.new_target_velocity = False

        self.follower_vel_gt_data= TwistStamped()
        self.new_follower_velocity = False

        self.target_uavr_pos_mv = PointStamped()
        self.target_world_pos_mv = PointStamped()

        # Camera coordinates to UAV coordinates
        self.Tuav_cam = np.zeros((4, 4))
        self.Tuav_cam[0, 2] = 1.0
        self.Tuav_cam[1, 0] = -1.0
        self.Tuav_cam[2, 1] = -1.0
        self.Tuav_cam[3, 3] = 1.0

        self.camera_info.K = [674.0643310546875, 0.0, 655.4468994140625, 0.0, 674.0643310546875, 368.7719421386719, 0.0, 0.0, 1.0]

        # Create subscribers
        rospy.Subscriber('/YOLODetection/tracked_detection', object, self.detection_callback)
        rospy.Subscriber('/yellow/mavros/global_position/local', Odometry, self.odometry_callback)
        rospy.Subscriber('/yellow/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/uav/velocity_relative', TwistStamped, self.target_vel_pose_gt_callback)
        rospy.Subscriber('/yellow/velocity_relative', TwistStamped, self.follower_vel_pose_gt_callback)
        #rospy.Subscriber('/zedm/zed_node/left/camera_info', CameraInfo, figure8.camera_info_callback)

        # Msgs filters
        target_pose_sub = message_filters.Subscriber('/uav/pose', PoseStamped)
        gimbal_pose_sub = message_filters.Subscriber('/gimbal/pose', PoseStamped)

        self.ts = message_filters.ApproximateTimeSynchronizer([target_pose_sub, gimbal_pose_sub], 10, 0.005)
        self.ts.registerCallback(self.pose_gt_callback)

        # Create publishers
        # Publishers for position of the target based on [u, v, depth] measurements
        self.target_world_position_mv_pub = rospy.Publisher('reconstructXYZ/world/position_mv', PointStamped, queue_size = 1)
        self.target_uavr_position_mv_pub = rospy.Publisher('reconstructXYZ/uav_relative/position_mv', PointStamped, queue_size = 1)

        self.depth_relative_pub = rospy.Publisher('reconstructXYZ/depth_relative', Float32, queue_size = 1)
        self.height_relative_pub = rospy.Publisher('reconstructXYZ/height_relative', Float32, queue_size = 1)
        self.yaw_relative_pub = rospy.Publisher('reconstructXYZ/yaw_relative', Float32, queue_size = 1)

        # Debug publishers for position and velocity of the target
        self.target_world_position_gt_pub = rospy.Publisher('reconstructXYZ/debug/world/target/position_gt', PointStamped, queue_size=1)
        self.target_uavr_position_gt_pub = rospy.Publisher('reconstructXYZ/debug/uav_relative/target/position_gt', PointStamped, queue_size=1)

        self.target_world_velocity_gt_pub = rospy.Publisher('reconstructXYZ/debug/world/target/velocity_gt', TwistStamped, queue_size = 1)
        self.target_uavr_velocity_gt_pub = rospy.Publisher('reconstructXYZ/debug/uav_relative/target/velocity_gt', TwistStamped, queue_size = 1)

        # KF publishers
        self.position_local_kf_pub = rospy.Publisher('reconstructXYZ/uav_relative/position_kf', PointStamped, queue_size=1)
        self.position_global_kf_pub = rospy.Publisher('reconstructXYZ/world/position_kf', PointStamped, queue_size=1)
        self.position_gkf2uavr_pub = rospy.Publisher('reconstructXYZ/uav_relative/position_gkf', PointStamped, queue_size = 1)

        self.velocity_local_kf_pub = rospy.Publisher('reconstructXYZ/uav_relative/velocity_kf', TwistStamped, queue_size=1)
        self.velocity_global_kf_pub = rospy.Publisher('reconstructXYZ/world/velocity_kf', TwistStamped, queue_size=1)

        self.depth_gt_pub = rospy.Publisher('reconstructXYZ/debug/depth_gt', Float32, queue_size = 1)

        # Dynamic reconfigure
        self.srv = Server(PositionKalmanFilterParametersConfig, self.parametersCb)

        # Kalman filters
        self.position_local_kf = LocalPositionKalmanFilter()
        self.position_global_kf = PositionKalmanFilter()

    """ Callbacks """
    def parametersCb(self, config, level):
        """  Dynamic reconfigure callback for parameters update.  """
        # rospy.loginfo("PositionKalmanFilter - Parameters callback.")

        return config

    def camera_info_callback(self, data):
        self.camera_info = data
        
    def detection_callback(self, data):
        self.new_detection_data = True
        self.detection_data = data

        self.depth_data.data = data.depth
        if (np.isfinite(self.depth_data.data) and not math.isnan(self.depth_data.data)):
            self.new_depth_data = True

    def odometry_callback(self, data):
        self.new_odometry_data = True
        self.odometry_data = data

    def pose_gt_callback(self, target, gimbal):
        self.target_pose = target
        self.gimbal_pose = gimbal

        self.new_gimbal_data = True
        self.new_target_pose = True

    def target_vel_pose_gt_callback(self, data):
        self.target_vel_gt_data = data
        self.new_target_velocity = True

    def follower_vel_pose_gt_callback(self, data):
        self.follower_vel_gt_data = data
        self.new_follower_velocity = True

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

    def transformMeasurements(self, t_x, t_y, t_z):
        # Input: (X, Y, Z) in camera c.s.
        # Output: /reconstructXYZ/debug/world/target/position_mv
        #         /reconstructXYZ/debug/uav_relative/target/position_mv

        Tcam_uav_t= np.zeros((4, 4))
        Tcam_uav_t[0, 0] = 1.0
        Tcam_uav_t[0, 3] = t_x
        Tcam_uav_t[1, 1] = 1.0
        Tcam_uav_t[1, 3] = t_y
        Tcam_uav_t[2, 2] = 1.0
        Tcam_uav_t[2, 3] = t_z
        Tcam_uav_t[3, 3] = 1.0

        # Transform from camera c.s. to uav_relative c.s.
        Tpos_uav = np.dot(self.Tuav_cam, Tcam_uav_t)

        self.target_uavr_pos_mv = PointStamped()
        self.target_uavr_pos_mv.header.frame_id = "yellow/base_link"
        self.target_uavr_pos_mv.header.stamp = rospy.Time.now()
        self.target_uavr_pos_mv.point.x = Tpos_uav[0,3]
        self.target_uavr_pos_mv.point.y = Tpos_uav[1,3]
        self.target_uavr_pos_mv.point.z = Tpos_uav[2,3]

        self.target_uavr_position_mv_pub.publish(self.target_uavr_pos_mv)

        """ Publish relative values for Visual-servoing """
        depth_r_msg = Float32()
        depth_r_msg.data = Tpos_uav[0,3]
        self.depth_relative_pub.publish(depth_r_msg)

        height_r_msg = Float32()
        height_r_msg.data = -1 * Tpos_uav[2,3]
        self.height_relative_pub.publish(height_r_msg)

        yaw_r_msg = Float32()
        yaw_r_msg.data = atan2(Tpos_uav[1,3], Tpos_uav[0,3])
        self.yaw_relative_pub.publish(yaw_r_msg)

        # Tranform from uav_relative c.s. to gazebo c.s.
        quaternion = [self.gimbal_pose.pose.orientation.w, self.gimbal_pose.pose.orientation.x, self.gimbal_pose.pose.orientation.y, self.gimbal_pose.pose.orientation.z]
        euler = self.quaternion2euler(quaternion)
        position = [self.gimbal_pose.pose.position.x, self.gimbal_pose.pose.position.y, self.gimbal_pose.pose.position.z]

        Tgazebo_uav = self.getRotationTranslationMatrix(euler, position)

        Tpos_gazebo = np.dot(Tgazebo_uav, Tpos_uav)

        # Tranform from gazebo c.s. to world c.s.
        euler = [0.0, 0.0, 1.5707963268]
        position = [0.0 , 0.0, 0.0]

        Tworld_gazebo = self.getRotationTranslationMatrix(euler, position)

        Tpos_world = np.dot(Tworld_gazebo, Tpos_gazebo)

        self.target_world_pos_mv = PointStamped()
        self.target_world_pos_mv.header.frame_id = "map"
        self.target_world_pos_mv.header.stamp = rospy.Time.now()
        self.target_world_pos_mv.point.x = Tpos_world[0,3]
        self.target_world_pos_mv.point.y = Tpos_world[1,3]
        self.target_world_pos_mv.point.z = Tpos_world[2,3]

        self.target_world_position_mv_pub.publish(self.target_world_pos_mv)

    def tranformTargetPosition(self):
        #  Input: /uav/pose
        # Output: /reconstructXYZ/debug/world/target/position_gt
        #         /reconstructXYZ/debug/uav_relative/target/position_gt

        Tpoint= np.zeros((4, 4))
        Tpoint[0, 0] = 1.0
        Tpoint[0, 3] = self.target_pose.pose.position.x
        Tpoint[1, 1] = 1.0
        Tpoint[1, 3] = self.target_pose.pose.position.y
        Tpoint[2, 2] = 1.0
        Tpoint[2, 3] = self.target_pose.pose.position.z
        Tpoint[3, 3] = 1.0

        # Transform from gazebo c.s. to world c.s.
        euler = [0.0, 0.0, 1.5707963268]
        position = [0.0 , 0.0, 0.0]

        Tworld_gazebo = self.getRotationTranslationMatrix(euler, position)

        Tpoint_world = np.dot(Tworld_gazebo, Tpoint)

        target_pos_world_msg = PointStamped()
        target_pos_world_msg.header.stamp = rospy.Time.now()
        target_pos_world_msg.header.frame_id = "map"
        target_pos_world_msg.point.x = Tpoint_world[0, 3]
        target_pos_world_msg.point.y = Tpoint_world[1, 3]
        target_pos_world_msg.point.z = Tpoint_world[2, 3]

        self.target_world_position_gt_pub.publish(target_pos_world_msg)

        # Transform from gazebo c.s. to uav_relative c.s. 
        quaternion = [self.gimbal_pose.pose.orientation.w, self.gimbal_pose.pose.orientation.x, self.gimbal_pose.pose.orientation.y, self.gimbal_pose.pose.orientation.z]
        euler = self.quaternion2euler(quaternion)
        position = [self.gimbal_pose.pose.position.x, self.gimbal_pose.pose.position.y, self.gimbal_pose.pose.position.z]

        Tuav_gazebo = self.getRotationTranslationMatrix(euler, position)

        Tpoint_uav = np.dot(np.linalg.inv(Tuav_gazebo), Tpoint)

        target_pos_uavr_msg = PointStamped()
        target_pos_uavr_msg.header.stamp = rospy.Time.now()
        target_pos_uavr_msg.header.frame_id = "yellow/base_link"
        target_pos_uavr_msg.point.x = Tpoint_uav[0, 3]
        target_pos_uavr_msg.point.y = Tpoint_uav[1, 3]
        target_pos_uavr_msg.point.z = Tpoint_uav[2, 3]

        self.target_uavr_position_gt_pub.publish(target_pos_uavr_msg)

        # Tranform from uav_relative c.s. to camera c.s.
        Tpoint_cam = np.dot(np.linalg.inv(self.Tuav_cam), Tpoint_uav)

        depth_gt_msg = Float32()
        depth_gt_msg.data = Tpoint_cam[2,3]
        self.depth_gt_pub.publish(depth_gt_msg)

        # Reset new data indicators
        self.new_target_pose = False
        self.new_gimbal_data = False

    def transformTargetVelocity(self):
        # Input: /uav/velocity_relative
        # Output: /reconstructXYZ/debug/world/target/velocity_gt
        #         /reconstructXYZ/debug/uav_relative/target/velocity_gt

        Tvelocity = np.zeros((4,4))
        Tvelocity[0, 0] = 1.0
        Tvelocity[0, 3] = self.target_vel_gt_data.twist.linear.x
        Tvelocity[1, 1] = 1.0
        Tvelocity[1, 3] = self.target_vel_gt_data.twist.linear.y
        Tvelocity[2, 2] = 1.0
        Tvelocity[2, 3] = self.target_vel_gt_data.twist.linear.z
        Tvelocity[3, 3] = 1.0

        # Transform from gazebo c.s. to world c.s. 
        euler = [0.0, 0.0, 1.5707963268]
        position = [0.0 , 0.0, 0.0]

        Tworld_gazebo = self.getRotationTranslationMatrix(euler, position)

        Tvel_world = np.dot(Tworld_gazebo, Tvelocity)

        target_vel_world_msg = TwistStamped()
        target_vel_world_msg.header.stamp = rospy.Time.now()
        target_vel_world_msg.twist.linear.x = Tvel_world[0,3]
        target_vel_world_msg.twist.linear.y = Tvel_world[1,3]
        target_vel_world_msg.twist.linear.z = Tvel_world[2,3]

        self.target_world_velocity_gt_pub.publish(target_vel_world_msg)

        # Transform from gazebo c.s. to uav_relative c.s. from gazebo c.s.
        quaternion = [self.gimbal_pose.pose.orientation.w, self.gimbal_pose.pose.orientation.x, self.gimbal_pose.pose.orientation.y, self.gimbal_pose.pose.orientation.z]
        euler = self.quaternion2euler(quaternion)
        position = [0.0 , 0.0, 0.0]

        Tuav_gazebo = self.getRotationTranslationMatrix(euler, position)

        Tvel_uav = np.dot(np.linalg.inv(Tuav_gazebo), Tvelocity)

        target_vel_uavr_msg = TwistStamped()
        target_vel_uavr_msg.header.stamp = rospy.Time.now()
        target_vel_uavr_msg.twist.linear.x = Tvel_uav[0,3]
        target_vel_uavr_msg.twist.linear.y = Tvel_uav[1,3]
        target_vel_uavr_msg.twist.linear.z = Tvel_uav[2,3]

        self.target_uavr_velocity_gt_pub.publish(target_vel_uavr_msg)

        # Reset new data indicators
        self.new_target_velocity = False
        self.new_follower_velocity = False

    def gkf2uavRelative(self, data):
        # Input: [X, Y, Z] in world c.s. form KF
        # Output: [X, Y, Z] in uav_relative c.s.
        Tpoint= np.zeros((4, 4))
        Tpoint[0, 0] = 1.0
        Tpoint[0, 3] = data.point.x
        Tpoint[1, 1] = 1.0
        Tpoint[1, 3] = data.point.y
        Tpoint[2, 2] = 1.0
        Tpoint[2, 3] = data.point.z
        Tpoint[3, 3] = 1.0

        # Transform from world c.s. to gazebo c.s.
        euler = [0.0, 0.0, 1.5707963268]
        position = [0.0 , 0.0, 0.0]

        Tgazebo_world = self.getRotationTranslationMatrix(euler, position)

        Tpos_gazebo = np.dot(np.linalg.inv(Tgazebo_world), Tpoint)

        # Tranform from gazebo c.s. to uav_relative
        quaternion = [self.gimbal_pose.pose.orientation.w, self.gimbal_pose.pose.orientation.x, self.gimbal_pose.pose.orientation.y, self.gimbal_pose.pose.orientation.z]
        euler = self.quaternion2euler(quaternion)
        position = [self.gimbal_pose.pose.position.x, self.gimbal_pose.pose.position.y, self.gimbal_pose.pose.position.z]

        Tuav_gazebo = self.getRotationTranslationMatrix(euler, position)

        Tpoint_uav = np.dot(np.linalg.inv(Tuav_gazebo), Tpos_gazebo)

        gkf_msg = PointStamped()
        gkf_msg.header.stamp = rospy.Time.now()
        gkf_msg.point.x = Tpoint_uav[0,3]
        gkf_msg.point.y = Tpoint_uav[1,3]
        gkf_msg.point.z = Tpoint_uav[2,3]

        self.position_gkf2uavr_pub.publish(gkf_msg)

    def publishKalman(self):
        if (self.position_local_kf.isFilterInitialized()):
            self.position_local_kf_pub.publish(self.position_local_kf.getPosition())
            self.velocity_local_kf_pub.publish(self.position_local_kf.getVelocity())

        if (self.position_global_kf.isFilterInitialized()):
            self.position_global_kf_pub.publish(self.position_global_kf.getPosition())
            self.velocity_global_kf_pub.publish(self.position_global_kf.getVelocity())

            self.gkf2uavRelative(self.position_global_kf.getPosition())

    def run(self, rate):
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():

            if self.new_target_pose and self.new_gimbal_data:
                self.tranformTargetPosition()

            if self.new_target_velocity and self.new_follower_velocity:
                self.transformTargetVelocity()

            if self.new_depth_data and self.new_detection_data and self.new_odometry_data:
                """
                x = (u - cx) * depth / fx
                y = (v - cy) * depth / fy
                z = depth
                """
                x_3d = (self.detection_data.x - self.camera_info.K[2]) * self.depth_data.data / self.camera_info.K[0]
                y_3d = (self.detection_data.y - self.camera_info.K[5]) * self.depth_data.data / self.camera_info.K[4]
                z_3d = self.depth_data.data

                self.transformMeasurements(x_3d, y_3d, z_3d)

                self.position_local_kf.filter(1.0 / rate, self.target_uavr_pos_mv, True, self.follower_vel_gt_data)
                self.position_global_kf.filter(1.0 / rate, self.target_world_pos_mv, True)

                # Reset conditions
                self.new_depth_data = False
                self.new_detection_data = False
                self.new_odometry_data = False

            else:
                self.position_local_kf.filter(1.0 / rate, self.target_uavr_pos_mv, False, self.follower_vel_gt_data)
                self.position_global_kf.filter(1.0 / rate, self.target_world_pos_mv, False)

            self.publishKalman()

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('reconstructXYZ')

    reconstructor = reconstructXYZ()
    reconstructor.run(50)
