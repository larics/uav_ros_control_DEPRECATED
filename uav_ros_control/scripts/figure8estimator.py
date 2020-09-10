#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray, Pose, TransformStamped
from uav_object_tracking_msgs.msg import object
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header, Bool, Float32
from nav_msgs.msg import Odometry, Path
import math
import numpy as np
from numpy.linalg import inv
from math import sqrt, sin, cos, pi, atan, floor
from std_srvs.srv import Empty
import Queue
import threading
import thread

import tf2_ros
import tf

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.distance import directed_hausdorff

from uav_ros_control.srv import GetLocalConstraints

class Tfm_Aprox():
    def __init__(self):
        self.camera_info = CameraInfo()
        self.new_depth_data = False
        self.list_size = 50
        self.detection_data_list = []
        self.odometry_data_list = []
        self.Tuav_cam = np.zeros((4, 4))
        self.new_detection_data = False
        self.new_odometry_data = False
        self.depth_header = Header()

        self.start_estimator = False

        self.Tuav_cam[0, 2] = 1.0
        self.Tuav_cam[1, 0] = -1.0
        self.Tuav_cam[2, 1] = -1.0
        self.Tuav_cam[3, 3] = 1.0

        self.camera_info.K = [674.0643310546875, 0.0, 655.4468994140625, 0.0, 674.0643310546875, 368.7719421386719, 0.0, 0.0, 1.0]

        self.a = 0.0
        self.d = 0.0
        self.p = []
        self.p_reconstructed = []
        self.time_vector = []

        self.x = []
        self.y = []
        self.z = []

        self.x_l = []
        self.y_l = []
        self.z_l = []

        self.x_g = []
        self.y_g = []
        self.z_g = []

        self.path_g = Path()

        self.xcoord = []
        self.ycoord = []
        self.zcoord = []

        self.path_coord = Path()

        self.hausdorff_counter = 0
        self.num_of_received_points = 0

        self.hausdorff = -1.0
        self.num_of_pts_hausdorff = 50

        self.hausdorff_threshold = 0.1
        self.max_a = 30.0
        self.min_a = 15.0
 
        self.direction_estimation_data_size = 5
        self.direction_estimation_min_time_diff = 1.0

        self.num_of_estimated_pts = 1000

        self.z_offset = 0.0
        self.estimator_state = False
        self.goal_setpoint = PoseStamped()
        self.backup_point = PoseStamped()

        self.goal_x_l = 0.0
        self.goal_y_l = 0.0
        self.goal_z_l = 0.0

        self.goal_x_g = 0.0
        self.goal_y_g = 0.0
        self.goal_z_g = 0.0

        self.target_x_l = 0.0
        self.target_y_l = 0.0
        self.target_z_l = 0.0

        self.target_x_g = 0.0
        self.target_y_g = 0.0
        self.target_z_g = 0.0

        # tightening lemniscate
        self.x_shift = [0, 0, 0]
        # Lock the orientation of the lemniscate
        self.is_x_direction_stationary = False

        # Get the corners of the arena in local frame
        self.local_corners = None
        try:
            rospy.wait_for_service("get_local_constraints")
            get_constraints = rospy.ServiceProxy("get_local_constraints", GetLocalConstraints)
            resp = get_constraints()
            self.local_corners = resp.constraints
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Couldn't get the local arena constraints. Service call failed with error: %s", e)
        self.disable_boundary_check = rospy.get_param('disable_boundary_check', False)

        rospy.loginfo('Geofencing is {}'.format("OFF" if self.disable_boundary_check else "ON"))

        # Tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        self.tf_lemniscate = TransformStamped()
        self.tf_lemniscate.header.frame_id = "map"
        self.tf_lemniscate.child_frame_id = "lemniscate"

    # Callbacks
    def detection_callback(self, data):
        self.new_detection_data = True
        if (len(self.detection_data_list) < self.list_size):
            self.detection_data_list.append(data)
        else:
            self.detection_data_list.append(data)
            self.detection_data_list.pop(0)

        # TO DO: add an exception for data out of sensor range
        if (np.isfinite(data.depth)):
            self.new_depth_data = True
        self.depth_data = data.depth
        self.depth_header = data.header

    def camera_info_callback(self, data):
        self.camera_info = data

    def odometry_callback(self, data):
        self.new_odometry_data = True
        if (len(self.odometry_data_list) < self.list_size):
            self.odometry_data_list.append(data)
        else:
            self.odometry_data_list.append(data)
            self.odometry_data_list.pop(0)

    def start_estimator_callback(self, data):
        self.start_estimator  = data

    #Publishers
    def set_uav_position_publisher(self, publisher):
        self.uav_position_publisher = publisher

    def set_uav_setpoint_publisher(self, publisher):
        self.uav_setpoint_publisher = publisher

    def set_uav_backup_setpoint_publisher(self, publisher):
        self.uav_backup_setpoint_publisher = publisher

    def set_path_g_publisher(self, publisher):
        self.path_g_publisher = publisher

    def set_path_coord_publisher(self, publisher):
        self.path_coord_publisher = publisher

    def set_estimator_state_publisher(self, publisher):
        self.estimator_state_publisher = publisher

    def set_num_of_points_publisher(self, publisher):
        self.num_of_received_points_publisher = publisher

    #Geofencing
    def check_inside_2d(self, point):
        if self.local_corners is None:
            rospy.logwarn_once("I don't have arena corners. Assuming all points are valid.")
            return True

        def is_left(P0, P1, P2):
            return (P1.x - P0.x) * (P2.y - P0.y) - (P2.x -  P0.x) * (P1.y - P0.y)

        wn = 0
        for i in range(len(self.local_corners) - 1):
            if self.local_corners[i].y <= point.y:
                if self.local_corners[i + 1].y > point.y:
                    if is_left(self.local_corners[i], self.local_corners[i + 1], point) > 0:
                        wn += 1
            elif self.local_corners[i + 1].y <= point.y:
                if is_left(self.local_corners[i], self.local_corners[i + 1], point) < 0:
                    wn -= 1

        return wn > 0
        
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
        return return_data

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

    def cosine_similarity(self, x, y):
        dot_product = np.dot(x,y)

        norm_x = np.linalg.norm(x)
        norm_y = np.linalg.norm(y)

        return dot_product / (norm_x * norm_y)

    def get_transform(self,X):
        # Find the average of points (centroid) along the columns
        C = np.average(X, axis=0)
        # Create CX vector (centroid to point) matrix
        CX = X - C
        # Singular value decomposition
        U, S, V = np.linalg.svd(CX)
        # The last row of V matrix indicate the eigenvectors of
        # smallest eigenvalues (singular values).
        T = np.zeros((4, 4))
        T[0:3, 0:3] = np.transpose(V)

        z = T[0:3, 2]
        z2 = -z
        if np.dot(z, np.array([0,0,1]))<np.dot(z2, np.array([0,0,1])):
            T[0, 2] = z2[0]
            T[1, 2] = z2[1]
            T[2, 2] = z2[2]
            z = z2

        x = T[0:3, 0]
        x2 = -x
        if (not self.is_x_direction_stationary):
            if np.dot(x, np.array([1,0,0]))<np.dot(x2, np.array([1,0,0])):
                T[0, 0] = x2[0]
                T[1, 0] = x2[1]
                T[2, 0] = x2[2]
                x = x2
        else:
            x1_cos_sim = 1 + self.cosine_similarity(x, self.X_rot)
            x2_cos_sim = 1 + self.cosine_similarity(x2, self.X_rot)

            if (x2_cos_sim > x1_cos_sim):
                T[0, 0] = x2[0]
                T[1, 0] = x2[1]
                T[2, 0] = x2[2]
                x = x2

        y = np.cross(z, x)

        T[0, 1] = y[0]
        T[1, 1] = y[1]
        T[2, 1] = y[2]
        T[0, 3] = C[0]
        T[1, 3] = C[1]
        T[2, 3] = C[2]
        T[3, 3] = 1.0

        return T

    def plot(self):

        fig = plt.figure()
        plt3d = plt.subplot(111, projection='3d')
        #plt.axis('equal')

        plt3d.scatter(self.x, self.y, self.z, color='b', marker="o") #2d
        plt3d.scatter(self.x_l, self.y_l, self.z_l, color='r', marker="_") #2d estimirano
        plt3d.scatter(self.x_g, self.y_g, self.z_g, color='g', marker="_") #3d estimirano
        plt3d.scatter(self.xcoord, self.ycoord, self.zcoord, color='r', marker="_") #3d stvarno
        plt3d.scatter(self.goal_x_g, self.goal_y_g, self.goal_z_g, color='c', marker="+", s=1000) #3d stvarno
        plt3d.scatter(self.target_x_g, self.target_y_g, self.target_z_g, color='r', marker="+", s=1000) #3d stvarno
        plt3d.scatter(self.backup_point.pose.position.x, self.backup_point.pose.position.y, self.backup_point.pose.position.z, color='b', marker="+", s=1000) #3d stvarno

        plt.show()

    def euclidean_distance(self, array_x, array_y):
        n = array_x.shape[0]
        ret = 0.
        for i in range(n):
            ret += (array_x[i]-array_y[i])**2
        return sqrt(ret)

    def get_hausdorff_distance(self, XA, XB):
        nA = XA.shape[0]
        nB = XB.shape[0]
        cmax_a = 0.
        for i in range(nA):
            cmin = np.inf
            for j in range(nB):
                d = self.euclidean_distance(XA[i,:], XB[j,:])
                if d<cmin:
                    cmin = d

                if cmin<cmax_a:
                    break
            if cmin>cmax_a and np.inf>cmin:
                cmax_a = cmin

        return cmax_a

    def reset_estimator_callback(self, req):
        self.a = 0.0
        self.d = 0.0
        self.p = []
        self.p_reconstructed = []

        self.x = []
        self.y = []
        self.z = []

        self.x_l = []
        self.y_l = []
        self.z_l = []

        self.x_g = []
        self.y_g = []
        self.z_g = []

        self.xcoord = []
        self.ycoord = []
        self.zcoord = []

        self.hausdorff_counter = 0

        self.hausdorff = -1.0

    def plot_callback(self, req):
        self.plot()

    def publishLemniscateTf(self, T):

        self.tf_lemniscate.header.stamp = rospy.Time.now()
        self.tf_lemniscate.transform.translation.x = T[0,3]
        self.tf_lemniscate.transform.translation.y = T[1,3]
        self.tf_lemniscate.transform.translation.z = T[2,3]

        q = tf.transformations.quaternion_from_matrix(T)

        self.tf_lemniscate.transform.rotation.x = q[0]
        self.tf_lemniscate.transform.rotation.y = q[1]
        self.tf_lemniscate.transform.rotation.z = q[2]
        self.tf_lemniscate.transform.rotation.w = q[3]

        self.br.sendTransform(self.tf_lemniscate)

    # Estimation functions
    def estimateBackupSetpoint(self, real_data, estimated_data, time_vector):
        found_flag = False
        index = 0

        for i in range(len(real_data) - self.direction_estimation_data_size):
            counter = 0

            for j in range(self.direction_estimation_data_size):
                diff = math.fabs(time_vector[i+j]-time_vector[i+j+1])

                if (diff < self.direction_estimation_min_time_diff):
                    counter = counter + 1

            if (counter == self.direction_estimation_data_size):
                index = i
                found_flag = True
                break

        if (found_flag):

            pt1 = real_data[index]
            pt2 = real_data[index+self.direction_estimation_data_size - 1]

            self.backup_point = PoseStamped()
            self.backup_point.header.stamp = rospy.get_rostime()
            self.backup_point.header.frame_id = "map"
            self.backup_point.pose.position.x = pt2[0]
            self.backup_point.pose.position.y = pt2[1]
            self.backup_point.pose.position.z = pt2[2] + self.z_offset

            dx = pt1[0] - pt2[0]
            dy = pt1[1] - pt2[1]
            yaw = math.atan2(dy, dx)

            euler = [0.0, 0.0, yaw]
            quaternion = self.euler2quaternion(euler)

            self.backup_point.pose.orientation.x = quaternion[1]
            self.backup_point.pose.orientation.y = quaternion[2]
            self.backup_point.pose.orientation.z = quaternion[3]
            self.backup_point.pose.orientation.w = quaternion[0]

            self.uav_backup_setpoint_publisher.publish(self.backup_point)

    def estimateDirection(self, real_data, estimated_data, time_vector):
        index = 0
        found_flag = False
        index_list = []
        sign = 0
        
        for i in range(len(real_data) - self.direction_estimation_data_size):
            counter = 0

            for j in range(self.direction_estimation_data_size):
                # Looking for subsequent measurements
                diff = math.fabs(time_vector[i+j]-time_vector[i+j+1])
                # Check if we have found enough subsequent measurements
                if (diff < self.direction_estimation_min_time_diff):
                    counter = counter + 1

            if (counter == self.direction_estimation_data_size):
                index = i
                found_flag = True
                break

        for i in range(self.direction_estimation_data_size):
            min_distance_index = 0
            min_distance = 0.0
            for j in range(len(estimated_data)):
                distance = self.euclidean_distance(estimated_data[j], real_data[index+i])
                if (j == 0):
                    min_distance_index = j
                    min_distance = distance

                if (min_distance > distance):
                    min_distance = distance
                    min_distance_index = j
            # List of indices of corresponding estimated data
            index_list.append(min_distance_index)

        # Compare first and last element
        if (index_list[0]-index_list[-1] < 0.0):
            sign = -1 # Ascending list (we assumed correctly)
        else:
            sign = 1 # Descending list

        return sign

    def estimatefigure8(self, data): #data je pointstamped

        row = [data.point.x, data.point.y, data.point.z]

        self.p.append(row)
        self.time_vector.append(data.header.stamp.to_sec())

        # Number of measurements of the target's position
        num_of_pts = len(self.p)

        self.xcoord = [0] * num_of_pts
        self.ycoord = [0] * num_of_pts
        self.zcoord = [0] * num_of_pts

        for i in range (len(self.p)):
            self.xcoord[i]=self.p[i][0]
            self.ycoord[i]=self.p[i][1]
            self.zcoord[i]=self.p[i][2]

        # Convert list to array of shape (num_of_pts, 3)
        pp = np.asarray(self.p)

        T = self.get_transform(pp)
        T_inv = np.linalg.inv(T)

        self.publishLemniscateTf(T)

        self.x = [0] * num_of_pts
        self.y = [0] * num_of_pts
        self.z = [0] * num_of_pts

        for i in range(num_of_pts):
            p_in = np.asarray([[self.xcoord[i]], [self.ycoord[i]], [self.zcoord[i]], [1]])
            p_out = np.dot(T_inv, p_in)
            self.x[i] = p_out[0]
            self.y[i] = p_out[1]
            self.z[i] = p_out[2]

        data_x = np.asarray(self.x).flatten()
        data_y = np.asarray(self.y).flatten()
        x0 = np.mean(data_x)
        y0 = np.mean(data_y)

        # racunanje a i d kao sredine najudaljenijih tocaka
        r = np.zeros(len(data_x))
        for i in range(len(data_x)):
            if data_x[i] >= 0:
                r[i] = sqrt(pow(data_x[i] - x0, 2) + pow(data_y[i] - y0, 2))
            else:
                r[i] = -sqrt(pow(data_x[i] - x0, 2) + pow(data_y[i] - y0, 2))

        self.d = (np.max(r)-np.min(r))/2.0
        self.a = self.d / sqrt(2)

        # Tightening lemniscate 
        x00 = (np.max(r)+np.min(r))/2.0
        self.x_shift = np.dot(T[:3,:3], [x00, 0, 0])
        #print(" x shift", self.x_shift)

        #generiranje tocnih tocki lemniskate u 2D
        t_points = np.linspace(0, 2 * pi, self.num_of_estimated_pts)
        self.x_l=[]
        self.y_l=[]
        for t in t_points:
            self.x_l.append(self.a * sqrt(2) * cos(t) / (sin(t) * sin(t) + 1))
            self.y_l.append(self.a * sqrt(2) * cos(t) * sin(t) / (sin(t) * sin(t) + 1))
        self.z_l=[0]*len(t_points)

        self.x_g = [0] * len(t_points)
        self.y_g = [0] * len(t_points)
        self.z_g = [0] * len(t_points)
        self.p_reconstructed = []
        for i in range(len(self.x_l)):
            p_in = np.asarray([[self.x_l[i]], [self.y_l[i]], [self.z_l[i]], [1]])
            p_out = np.dot(T, p_in)
            self.x_g[i] = p_out[0] + self.x_shift[0]
            self.y_g[i] = p_out[1] + self.x_shift[1]
            self.z_g[i] = p_out[2] + self.x_shift[2]

            row = [p_out[0] + self.x_shift[0], p_out[1] + self.x_shift[1], p_out[2] + self.x_shift[2]]
            self.p_reconstructed.append(row)

    def run(self, rate):
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            if (self.start_estimator):
                rospy.loginfo_once('Figure-8-estimator started.')
                global_position = PointStamped()
                detection_data = object()
                odometry_data = Odometry()
                if self.new_depth_data and self.new_detection_data and self.new_odometry_data:

                    # TO DO: detection and depth data are already synced in YOLODetection
                    detection_data = self.find_min_time_diff_data(self.detection_data_list, self.depth_header.stamp.to_sec())
                    odometry_data = self.find_min_time_diff_data(self.odometry_data_list, self.depth_header.stamp.to_sec())

                    # Uncomment in case of distance instead depth
                    # x = (detection_data.x - self.camera_info.K[2]) / self.camera_info.K[0]
                    # y = (detection_data.y - self.camera_info.K[5]) / self.camera_info.K[4]

                    # z_m = self.depth_data.point.x / (math.sqrt(1 + x*x + y*y))

                    # x_m = x * z_m
                    # y_m = y * z_m

                    x_m = (detection_data.x - self.camera_info.K[2]) * self.depth_data / self.camera_info.K[0]
                    y_m = (detection_data.y - self.camera_info.K[5]) * self.depth_data / self.camera_info.K[4]
                    z_m = self.depth_data

                    Tcam_uav1 = np.zeros((4, 4))
                    Tcam_uav1[0, 0] = 1.0
                    Tcam_uav1[0, 3] = x_m
                    Tcam_uav1[1, 1] = 1.0
                    Tcam_uav1[1, 3] = y_m
                    Tcam_uav1[2, 2] = 1.0
                    Tcam_uav1[2, 3] = z_m
                    Tcam_uav1[3, 3] = 1.0

                    # Transform from camera c.s. to uav_relative c.s.
                    Tuav_uav1 = np.dot(self.Tuav_cam, Tcam_uav1)

                    quaternion = [odometry_data.pose.pose.orientation.w, odometry_data.pose.pose.orientation.x, odometry_data.pose.pose.orientation.y, odometry_data.pose.pose.orientation.z]
                    euler = self.quaternion2euler(quaternion)
                    position = [odometry_data.pose.pose.position.x, odometry_data.pose.pose.position.y, odometry_data.pose.pose.position.z]

                    Tworld_uav = self.getRotationTranslationMatrix(euler, position)

                    # Tranform from uav_relative c.s. to gazebo c.s.
                    Tworld_uav1 = np.dot(Tworld_uav, Tuav_uav1)

                    # Tranform from gazebo c.s. to world(mavros) c.s.
                    # Comment out this part if you use odometry from mavros
                    euler = [0.0, 0.0, 1.5707963268]
                    position = [0.0 , 0.0, 0.0]

                    Tworld_gazebo = self.getRotationTranslationMatrix(euler, position)
                    Tworld_uav1 = np.dot(Tworld_gazebo, Tworld_uav1)

                    uav_position = PointStamped()
                    uav_position.header = self.depth_header
                    uav_position.header.frame_id = "map"
                    uav_position.point.x = Tworld_uav1[0,3]
                    uav_position.point.y = Tworld_uav1[1,3]
                    uav_position.point.z = Tworld_uav1[2,3]

                    if self.check_inside_2d(uav_position.point) or self.disable_boundary_check:
                        self.hausdorff_counter = self.hausdorff_counter + 1
                        self.num_of_received_points = self.num_of_received_points + 1

                        self.num_of_received_points_publisher.publish(self.num_of_received_points)
                        rospy.loginfo_throttle(10, "Figure-8-estimator - Received %d points" %self.num_of_received_points)

                        if (not self.estimator_state):
                            self.estimatefigure8(uav_position)
                    else:
                        rospy.logwarn("Figure-8-estimator - Receieved point not inside GEO Fence.")

                    if (self.hausdorff_counter > self.num_of_pts_hausdorff):
                        p_reconstructed = np.asarray(self.p_reconstructed)#, dtype=None, order=None)
                        # Prepare (self.num_of_estimated_pts, 3) shape for built-in function
                        p_reconstructed_2dim = p_reconstructed.reshape(len(p_reconstructed),-1)
                        p = np.asarray(self.p)

                        # Custom hausdorff function
                        #self.hausdorff = self.get_hausdorff_distance(p, p_reconstructed)

                        # General symmetric Hausdorff
                        self.hausdorff = max(directed_hausdorff(p, p_reconstructed_2dim), directed_hausdorff(p_reconstructed_2dim, p))[0]
                        self.hausdorff_counter = 0
                        #print(self.hausdorff, self.hausdorff/self.a, 'hausdorff')

                        #gledamo s 1/4 pi prema 3/4 pi
                        direction = self.estimateDirection(p, p_reconstructed, self.time_vector)
                        self.estimateBackupSetpoint(p, p_reconstructed, self.time_vector)

                        self.goal_x_l = 0.0
                        self.goal_y_l = 0.0
                        self.goal_z_l = 0.0

                        self.target_x_l = 0.0
                        self.target_y_l = 0.0
                        self.target_z_l = 0.0

                        if (direction < 0.0):
                            t = 1.0 * pi / 4.0
                        else:
                            t = 3.0 * pi / 4.0

                        self.goal_x_l = self.a * sqrt(2) * cos(t) / (sin(t) * sin(t) + 1)
                        self.goal_y_l = self.a * sqrt(2) * cos(t) * sin(t) / (sin(t) * sin(t) + 1)

                        if (direction < 0.0):
                            t = 3.0 * pi / 4.0
                        else:
                            t = 1.0 * pi / 4.0
                            
                        self.target_x_l = self.a * sqrt(2) * cos(t) / (sin(t) * sin(t) + 1)
                        self.target_y_l = self.a * sqrt(2) * cos(t) * sin(t) / (sin(t) * sin(t) + 1)

                        pp = np.asarray(self.p)
                        T = self.get_transform(pp)

                        p_in = np.asarray([[self.goal_x_l], [self.goal_y_l], [self.goal_z_l], [1]])
                        p_out = np.dot(T, p_in)
                        self.goal_x_g = p_out[0] + self.x_shift[0]
                        self.goal_y_g = p_out[1] + self.x_shift[1]
                        self.goal_z_g = p_out[2] + self.x_shift[2]

                        p_in = np.asarray([[self.target_x_l], [self.target_y_l], [self.target_z_l], [1]])
                        p_out = np.dot(T, p_in)
                        self.target_x_g = p_out[0] + self.x_shift[0]
                        self.target_y_g = p_out[1] + self.x_shift[1]
                        self.target_z_g = p_out[2] + self.x_shift[2]

                        self.goal_setpoint.header.stamp = rospy.get_rostime()
                        self.goal_setpoint.header.frame_id = "map"
                        self.goal_setpoint.pose.position.x = self.goal_x_g
                        self.goal_setpoint.pose.position.y = self.goal_y_g
                        self.goal_setpoint.pose.position.z = self.goal_z_g + self.z_offset

                        dx = self.target_x_g - self.goal_x_g
                        dy = self.target_y_g - self.goal_y_g
                        yaw = math.atan2(dy, dx)

                        euler = [0.0, 0.0, yaw]
                        quaternion = self.euler2quaternion(euler)

                        self.goal_setpoint.pose.orientation.x = quaternion[1]
                        self.goal_setpoint.pose.orientation.y = quaternion[2]
                        self.goal_setpoint.pose.orientation.z = quaternion[3]
                        self.goal_setpoint.pose.orientation.w = quaternion[0]

                        if (self.a < self.max_a and self.a > self.min_a and self.hausdorff/self.a >= 0.0 and self.hausdorff/self.a < self.hausdorff_threshold and not self.estimator_state):
                            self.estimator_state = True
                            
                            #self.plot()

                    self.uav_position_publisher.publish(uav_position)

                    self.new_depth_data = False

                self.uav_setpoint_publisher.publish(self.goal_setpoint)

                if self.estimator_state: rospy.logwarn_once("Figure-8-estimator has found interception setpoint.")
                if self.num_of_received_points > 1 :
                    rospy.loginfo_throttle(10, "Figure-8-estimator - Hausdorff distance is " + "{:.3f}".format(self.hausdorff/self.a) + ". TH: " + str(self.hausdorff_threshold))

                estimator_state_msg = Bool()
                estimator_state_msg.data = self.estimator_state
                self.estimator_state_publisher.publish(estimator_state_msg)

                # Check if x direction can be stated stationary
                if (self.a > self.min_a and not self.is_x_direction_stationary):
                    self.is_x_direction_stationary = True
                    self.X_rot = T[0:3,0]
                    rospy.logwarn("Figure-8-estimator - Declaring x direction stationary.")


                # plot publish
                self.path_g = Path()
                for i in range(len(self.x_g)):
                    temp_pose = PoseStamped()
                    temp_pose.pose.position.x = self.x_g[i]
                    temp_pose.pose.position.y = self.y_g[i]
                    temp_pose.pose.position.z = self.z_g[i]

                    temp_header = Header()
                    temp_header.stamp = rospy.Time.now()
                    temp_header.frame_id = "map"

                    temp_pose.header = temp_header

                    self.path_g.poses.append(temp_pose)

                self.path_g.header.frame_id = "map"

                self.path_coord = Path()
                for i in range(len(self.xcoord)):
                    temp_pose = PoseStamped()
                    temp_pose.pose.position.x = self.xcoord[i]
                    temp_pose.pose.position.y = self.ycoord[i]
                    temp_pose.pose.position.z = self.zcoord[i]

                    temp_header = Header()
                    temp_header.stamp = rospy.Time.now()
                    temp_header.frame_id = "map"

                    temp_pose.header = temp_header

                    self.path_coord.poses.append(temp_pose)

                self.path_coord.header.frame_id = "map"

                self.path_g_publisher.publish(self.path_g)
                self.path_coord_publisher.publish(self.path_coord)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('figure8estimator')

    figure8 = Tfm_Aprox()

    rospy.Subscriber('/YOLODetection/tracked_detection', object, figure8.detection_callback)
    #rospy.Subscriber('/camera/color/camera_info', CameraInfo, figure8.camera_info_callback)
    rospy.Subscriber('/zedm/zed_node/left/camera_info', CameraInfo, figure8.camera_info_callback)

    #rospy.Subscriber('mavros/global_position/local', Odometry, figure8.odometry_callback)
    rospy.Subscriber('/gimbal/odometry', Odometry, figure8.odometry_callback)


    rospy.Subscriber('sm_pursuit/start_estimator', Bool, figure8.start_estimator_callback)


    rospy.Service('reset_figure8_estimator', Empty, figure8.reset_estimator_callback)
    rospy.Service('plot', Empty, figure8.plot_callback)

    uav_backup_setpoint = rospy.Publisher('figure8/target/setpoint_estimated_backup', PoseStamped, queue_size=1)
    uav_position = rospy.Publisher('figure8/target/position_estimated', PointStamped, queue_size=1)
    uav_setpoint = rospy.Publisher('figure8/target/setpoint_estimated', PoseStamped, queue_size=1)

    estimated_figure_pub = rospy.Publisher('figure8/debug/path_estimated', Path, queue_size=1)
    received_points_pub = rospy.Publisher('figure8/debug/path_collected', Path, queue_size=1)
    num_of_received_points_pub = rospy.Publisher('figure8/debug/points_number', Float32, queue_size=1)
    estimator_state_pub = rospy.Publisher('figure8/state', Bool, queue_size=1)

    figure8.set_uav_position_publisher(uav_position)
    figure8.set_uav_setpoint_publisher(uav_setpoint)
    figure8.set_uav_backup_setpoint_publisher(uav_backup_setpoint)
    figure8.set_path_g_publisher(estimated_figure_pub)
    figure8.set_path_coord_publisher(received_points_pub)
    figure8.set_estimator_state_publisher(estimator_state_pub)
    figure8.set_num_of_points_publisher(num_of_received_points_pub)


    figure8.run(50)
