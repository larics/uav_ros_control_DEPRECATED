#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray, Pose
from uav_object_tracking.msg import object
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
import math
import numpy as np
from numpy.linalg import inv
from math import sqrt, sin, cos, pi, atan, floor
from std_srvs.srv import Empty
import Queue
import threading
import thread

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Tfm_Aprox():
    def __init__(self):
        self.camera_info = CameraInfo()
        self.new_distance_data = False
        self.list_size = 50
        self.distance_data = PointStamped()
        self.detection_data_list = []
        self.odometry_data_list = []
        self.Tuav_cam = np.zeros((4, 4))
        self.new_detection_data = False
        self.new_odometry_data = False


        self.Tuav_cam[0, 2] = 1.0
        self.Tuav_cam[1, 0] = -1.0
        self.Tuav_cam[2, 1] = -1.0
        self.Tuav_cam[3, 3] = 1.0

        self.camera_info.K = [674.0643310546875, 0.0, 655.4468994140625, 0.0, 674.0643310546875, 368.7719421386719, 0.0, 0.0, 1.0]

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

        self.path_g = Path()

        self.xcoord = []
        self.ycoord = []
        self.zcoord = []

        self.path_coord = Path()

        self.hausdorff_counter = 0

        self.hausdorff = -1.0
        self.num_of_pts_hausdorff = 100

        self.hausdorff_threshold = 0.25

        self.z_offset = 0.0
        self.publish_goal_setpoint = False
        self.goal_setpoint = PoseStamped()

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


    def distance_callback(self, data):
        self.new_distance_data = True
        self.distance_data = data
        
    def detection_callback(self, data):
        self.new_detection_data = True
        if (len(self.detection_data_list) < self.list_size):
            self.detection_data_list.append(data)
        else:
            self.detection_data_list.append(data)
            self.detection_data_list.pop(0)

    def camera_info_callback(self, data):
        self.camera_info = data

    def odometry_callback(self, data):
        self.new_odometry_data = True
        if (len(self.odometry_data_list) < self.list_size):
            self.odometry_data_list.append(data)
        else:
            self.odometry_data_list.append(data)
            self.odometry_data_list.pop(0)

    def set_uav_position_publisher(self, publisher):
        self.uav_position_publisher = publisher

    def set_uav_setpoint_publisher(self, publisher):
        self.uav_setpoint_publisher = publisher

    def set_path_g_publisher(self, publisher):
        self.path_g_publisher = publisher

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


    def run(self, rate):
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            global_position = PointStamped()
            detection_data = object()
            odometry_data = Odometry()
            if self.new_distance_data and self.new_detection_data and self.new_odometry_data:

                self.hausdorff_counter = self.hausdorff_counter + 1

                detection_data = self.find_min_time_diff_data(self.detection_data_list, self.distance_data.header.stamp.to_sec())
                odometry_data = self.find_min_time_diff_data(self.odometry_data_list, self.distance_data.header.stamp.to_sec())

                x = (detection_data.x - self.camera_info.K[2]) / self.camera_info.K[0]
                y = (detection_data.y - self.camera_info.K[5]) / self.camera_info.K[4]

                z_m = self.distance_data.point.x / (math.sqrt(1 + x*x + y*y))

                x_m = x * z_m
                y_m = y * z_m

                #treba transformirati iz kamere u globalni sustav

                Tcam_uav1 = np.zeros((4, 4))
                Tcam_uav1[0, 0] = 1.0
                Tcam_uav1[0, 3] = x_m
                Tcam_uav1[1, 1] = 1.0
                Tcam_uav1[1, 3] = y_m
                Tcam_uav1[2, 2] = 1.0
                Tcam_uav1[2, 3] = z_m
                Tcam_uav1[3, 3] = 1.0

                quaternion = [odometry_data.pose.pose.orientation.w, odometry_data.pose.pose.orientation.x, odometry_data.pose.pose.orientation.y, odometry_data.pose.pose.orientation.z]
                euler = self.quaternion2euler(quaternion)
                position = [odometry_data.pose.pose.position.x, odometry_data.pose.pose.position.y, odometry_data.pose.pose.position.z]

                Tworld_uav = self.getRotationTranslationMatrix(euler, position)

                Tuav_uav1 = np.dot(self.Tuav_cam, Tcam_uav1)

                Tworld_uav1 = np.dot(Tworld_uav, Tuav_uav1)

                uav_position = PointStamped()
                uav_position.header = self.distance_data.header
                uav_position.point.x = Tworld_uav1[0,3]
                uav_position.point.y = Tworld_uav1[1,3]
                uav_position.point.z = Tworld_uav1[2,3]

                if (not self.publish_goal_setpoint):
                    self.estimatefigure8(uav_position)

                if (self.hausdorff_counter > self.num_of_pts_hausdorff):
                    p_reconstructed = np.asarray(self.p_reconstructed)#, dtype=None, order=None)
                    p = np.asarray(self.p)
                    self.hausdorff = self.get_hausdorff_distance(p, p_reconstructed)
                    self.hausdorff_counter = 0
                    print(self.hausdorff, self.hausdorff/self.a, 'hausdorff')


                    if (self.hausdorff/self.a >= 0.0 and self.hausdorff/self.a < self.hausdorff_threshold and not self.publish_goal_setpoint):
                        #gledamo s 1/4 pi prema 3/4 pi
                        self.goal_x_l = 0.0
                        self.goal_y_l = 0.0
                        self.goal_z_l = 0.0

                        self.target_x_l = 0.0
                        self.target_y_l = 0.0
                        self.target_z_l = 0.0

                        t = 1.0 * pi / 4.0

                        self.goal_x_l = self.a * sqrt(2) * cos(t) / (sin(t) * sin(t) + 1)
                        self.goal_y_l = self.a * sqrt(2) * cos(t) * sin(t) / (sin(t) * sin(t) + 1)

                        t = 3.0 * pi / 4.0
                        self.target_x_l = self.a * sqrt(2) * cos(t) / (sin(t) * sin(t) + 1)
                        self.target_y_l = self.a * sqrt(2) * cos(t) * sin(t) / (sin(t) * sin(t) + 1)

                        pp = np.asarray(self.p)
                        T = self.get_transform(pp)

                        p_in = np.asarray([[self.goal_x_l], [self.goal_y_l], [self.goal_z_l], [1]])
                        p_out = np.dot(T, p_in)
                        self.goal_x_g = p_out[0]
                        self.goal_y_g = p_out[1]
                        self.goal_z_g = p_out[2]

                        p_in = np.asarray([[self.target_x_l], [self.target_y_l], [self.target_z_l], [1]])
                        p_out = np.dot(T, p_in)
                        self.target_x_g = p_out[0]
                        self.target_y_g = p_out[1]
                        self.target_z_g = p_out[2]

                        self.publish_goal_setpoint = True
                        self.goal_setpoint.header.stamp = rospy.get_rostime()
                        self.goal_setpoint.header.frame_id = "world"
                        self.goal_setpoint.pose.position.x = self.goal_x_g
                        self.goal_setpoint.pose.position.y = self.goal_y_g
                        self.goal_setpoint.pose.position.z = self.goal_z_g + self.z_offset

                        dx = self.target_x_g - self.goal_x_g
                        dy = self.target_y_g - self.goal_y_g
                        yaw = math.atan2(dy, dx)

                        euler = [0.0, 0.0, yaw]
                        quaternion = self.euler2quaternion(euler)

                        print yaw

                        self.goal_setpoint.pose.orientation.x = quaternion[1]
                        self.goal_setpoint.pose.orientation.y = quaternion[2]
                        self.goal_setpoint.pose.orientation.z = quaternion[3]
                        self.goal_setpoint.pose.orientation.w = quaternion[0]
                        
                        #self.plot()

                self.uav_position_publisher.publish(uav_position)

                self.new_distance_data = False

            if (self.publish_goal_setpoint):
                    self.uav_setpoint_publisher.publish(self.goal_setpoint)

            # plot publish
            self.path_g = Path()
            for i in range(len(self.x_g)):
                temp_pose = PoseStamped()
                temp_pose.pose.position.x = self.x_g[i]
                temp_pose.pose.position.y = self.y_g[i]
                temp_pose.pose.position.z = self.z_g[i]

                temp_header = Header()
                temp_header.stamp = rospy.Time.now()
                temp_header.frame_id = "world"

                temp_pose.header = temp_header

                self.path_g.poses.append(temp_pose)

            self.path_g.header.frame_id = "world"

            self.path_g_publisher.publish(self.path_g)

            r.sleep()

    def get_line(self, B, x):
        return B[0]*x + B[1]

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
        if np.dot(x, np.array([1,0,0]))<np.dot(x2, np.array([1,0,0])):
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

    def equations(self, p, *data): 
        w=p
        a, x0, y0, t, xl, yl, angl = data
        #print(a, x0, y0, t, xl, yl, angl)
        #f1 = a * sqrt(2) * cos(2*pi*self.w_atm*self.t) / (sin(2*pi*self.w_atm*self.t) * sin(2*pi*self.w_atm*self.t) + 1)
        f1 = (a * sqrt(2) * cos(2*pi*w*t) / (sin(2*pi*w*t) * sin(2*pi*w*t) + 1) * cos(angl) - a * sqrt(2) * cos(2*pi*w*t) * sin(2*pi*w*t) / (
                        sin(2*pi*w*t) * sin(2*pi*w*t) + 1) * sin(angl) +x0 - xl)
        return f1

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
        self.publish_goal_setpoint = False

    def plot_callback(self, req):
        self.plot()


    def estimatefigure8(self, data): #data je pointstamped

        row = [data.point.x, data.point.y, data.point.z]

        self.p.append(row)

        num_of_pts = len(self.p)

        self.xcoord = [0] * num_of_pts
        self.ycoord = [0] * num_of_pts
        self.zcoord = [0] * num_of_pts

        for i in range (len(self.p)):
            self.xcoord[i]=self.p[i][0]
            self.ycoord[i]=self.p[i][1]
            self.zcoord[i]=self.p[i][2]

        pp = np.asarray(self.p)

        T = self.get_transform(pp)
        T_inv = np.linalg.inv(T)

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
        #print("a = ", self.a)
        #print "\n"

        #shift po x osi

        x00 = (np.max(r)+np.min(r))/2.0
        #print("pomak centra", x00)
        xshift = [0] * len(self.x)
        for i in range (len(self.x)-1):
            xshift[i]=self.x[i]-x00

        #generiranje tocnih tocki lemniskate u 2D

        t_points = np.linspace(0, 2 * pi, num_of_pts)
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
            self.x_g[i] = p_out[0]
            self.y_g[i] = p_out[1]
            self.z_g[i] = p_out[2]
            row = [p_out[0], p_out[1], p_out[2]]
            self.p_reconstructed.append(row)

if __name__ == '__main__':
    rospy.init_node('figure8estimator')

    figure8 = Tfm_Aprox()

    rospy.Subscriber('/uav_object_tracking/uav/distance_kf_header', PointStamped, figure8.distance_callback)
    rospy.Subscriber('/YOLODetection/tracked_detection', object, figure8.detection_callback)
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, figure8.camera_info_callback)

    rospy.Subscriber('/red/mavros/global_position/local', Odometry, figure8.odometry_callback)

    rospy.Service('reset_figure8_estimator', Empty, figure8.reset_estimator_callback)
    rospy.Service('plot', Empty, figure8.plot_callback)

    uav_position = rospy.Publisher('/red/target_uav/position_estimated', PointStamped, queue_size=1)
    uav_setpoint = rospy.Publisher('/red/target_uav/setpoint_estimated', PoseStamped, queue_size=1)

    estimated_figure_pub = rospy.Publisher('/red/figure8_estimated', Path, queue_size=1)


    figure8.set_uav_position_publisher(uav_position)
    figure8.set_uav_setpoint_publisher(uav_setpoint)
    figure8.set_path_g_publisher(estimated_figure_pub)

    figure8.run(50)
