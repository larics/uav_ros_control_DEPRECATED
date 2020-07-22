#!/usr/bin/env python

import rospy
import numpy as np

from dynamic_reconfigure.msg import Config
from geometry_msgs.msg import PointStamped, TwistStamped

class PositionKalmanFilter():

    def __init__(self):
        self.kalmanInitialized = False
        self.predictTime = 0.0
        self.maxpredictTime = 2.0

        # Create subscribers.
        rospy.Subscriber('/yellow/reconstructXYZ/parameter_updates', Config, self.parametersCb, queue_size=1)

    def parametersCb(self, data):
        """  Dynamic reconfigure callback for parameters update.  """
        rospy.loginfo("GlobalPositionKalmanFilter - Parameters callback.")

        if(self.kalmanInitialized):
            self.setPositionNoise(rospy.get_param("/yellow/reconstructXYZ/GKFpositionNoise"))
            self.setVelocityNoise(rospy.get_param("/yellow/reconstructXYZ/GKFvelocityNoise"))
            self.setMeasurementNoise(rospy.get_param("/yellow/reconstructXYZ/GKFmeasurementNoise"))

    def init(self, timeStep, initialState):
        """
        Initialize parameters for discrete Kalman filter.

        The 6-dimensional state space:

                x y z v_x v_y v_z

        contains the 3D position and velocities of the moving target in respect to the tracker.

        Object motion follows a constant velocity model.
        """
        self.dt = timeStep
        #Prior (predicted) state covariance matrix
        self.P0 = 10 * np.identity(6)
        #State estimate vector
        self.X0 = np.zeros((6,1))
        self.X0[0,0] = initialState.point.x 
        self.X0[1,0] = initialState.point.y
        self.X0[2,0] = initialState.point.z

        #Covariance of the process noise
        self.positionNoise = 10.0
        self.velocityNoise = 100.0
        self.Q = 1000 * np.identity(6)
        self.L = np.zeros((6,6))
        self.L[0,0] = pow(self.dt,2) / 2
        self.L[1,1] = pow(self.dt,2) / 2
        self.L[2,2] = pow(self.dt,2) / 2
        self.L[3,3] = self.dt
        self.L[4,4] = self.dt
        self.L[5,5] = self.dt
        self.Q = np.dot(self.L, np.dot(self.Q, self.L.transpose()))

        #Covariance of the observation noise
        self.R = np.zeros((3,3))
        self.R[0,0] = 1
        self.R[1,1] = 1
        self.R[2,2] = 1

        #Process matrix, assuming constant velocity model
        self.A = np.zeros((6,6))
        self.A[0,0] = 1
        self.A[0,3] = self.dt
        self.A[1,1] = 1
        self.A[1,4] = self.dt
        self.A[2,2] = 1
        self.A[2,5] = self.dt
        self.A[3,3] = 1
        self.A[4,4] = 1
        self.A[5,5] = 1

        #The observation model
        self.H = np.zeros((3,6))
        self.H[0,0] = 1
        self.H[1,1] = 1
        self.H[2,2] = 1

        self.X_old = np.zeros((6,1))
        self.X_old = self.X0

        self.P_old = np.zeros((6,6))
        self.P_old = self.P0

    def predict(self):
        """  Predict state vector X and variance of uncertainty P (covariance)
            - m stands for minus
        """
        self.X_hat_m = np.dot(self.A, self.X_old)
        self.P_m = np.dot(self.A, np.dot(self.P_old, self.A.transpose())) + self.Q

        self.X_old = self.X_hat_m
        self.P_old = self.P_m

    def update(self, measurement):
        """ Update state vector X and variance of uncertainty P (covariance)
            - p stands for plus
        """
        #Identity matrix
        I = np.identity(6)

        #Vector of observations
        self.Y = np.zeros((3,1))
        self.Y[0] = measurement.point.x
        self.Y[1] = measurement.point.y
        self.Y[2] = measurement.point.z

        #Kalman gain
        self.K = np.dot(np.dot(self.P_m, self.H.transpose()), np.linalg.inv(np.dot(self.H, np.dot(self.P_m, self.H.transpose())) + self.R))

        self.X_hat_p = self.X_hat_m + np.dot(self.K, (self.Y - np.dot(self.H, self.X_hat_m)))
        self.P_p = np.dot((I - np.dot(self.K, self.H)) ,np.dot(self.P_m, np.transpose(I - np.dot(self.K, self.H))))+ np.dot(self.K, np.dot(self.R, self.K.transpose()))

        self.X_old = self.X_hat_p
        self.P_old = self.P_p

    def filter(self, dt, measurement, newMeasurementFlag):

        if (not self.kalmanInitialized):
            self.resetState();

        if (not self.kalmanInitialized and not newMeasurementFlag):
            return

        if (not self.kalmanInitialized and newMeasurementFlag):
            self.kalmanInitialized = True
            self.init(dt, measurement)
            print " KF initialized"

        self.predict()

        if (newMeasurementFlag):
            self.update(measurement)
            self.predictTime = 0.0
        else:
            self.predictTime += dt

        if (self.predictTime > self.maxpredictTime):
            self.resetState()

    def resetState(self):
        self.kalmanInitialized = False
        self.predictTime = 0.0

    def setMeasurementNoise(self, noise):
        print noise
        self.R[0,0] = noise
        self.R[1,1] = noise
        self.R[2,2] = noise

    def setPositionNoise(self, noise):
        self.positionNoise = noise
        self.updateProcessNoise()

    def setVelocityNoise(self, noise):
        self.velocityNoise = noise
        self.updateProcessNoise()

    def updateProcessNoise(self):
        self.Q = np.identity(6)
        self.Q[0,0] = self.positionNoise
        self.Q[1,1] = self.positionNoise
        self.Q[2,2] = self.positionNoise
        self.Q[3,3] = self.velocityNoise
        self.Q[4,4] = self.velocityNoise
        self.Q[5,5] = self.velocityNoise

    def setProcessNoise(self, noise):
        #Covariance of the process noise
        self.Q = noise * np.identity(6)
        self.Q = np.dot(self.L, np.dot(self.Q, self.L.transpose()))

    def getPosition(self):
        msg = PointStamped()
        msg.point.x = self.X_old[0,0]
        msg.point.y = self.X_old[1,0]
        msg.point.z = self.X_old[2,0]

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        return msg

    def getVelocity(self):
        msg = TwistStamped()
        msg.twist.linear.x = self.X_old[3,0]
        msg.twist.linear.y = self.X_old[4,0]
        msg.twist.linear.z = self.X_old[5,0]

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        return msg

    def isFilterInitialized(self):
        return self.kalmanInitialized
