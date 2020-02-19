#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from math import sin, cos
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class DynamixelNet:

    def __init__(self):
        self.rate = rospy.get_param('~rate', 50)
        self.current_dynamixel_position = 0.0
        self.received_first_measurement = False
        self.current_state = "nothing"
        self.dynamixel_velocity = Twist()

        self.fully_winded = rospy.get_param('~fully_winded', -30.0)
        self.fully_unwinded = rospy.get_param('~fully_unwinded', 30.0)
        self.winding_speed = rospy.get_param('~winding_speed', 1.0)

        self.cmd_vel_pub = rospy.Publisher('dynamixel_workbench/cmd_vel', 
            Twist, queue_size=1)

        rospy.Subscriber('dynamixel_workbench/joint_states', JointState, 
            self.jointStateCallback, queue_size=1)
        rospy.Subscriber('dynamixel_workbench/set_state', String, 
            self.stateCallback, queue_size=1)

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.current_state == "winding":
                if self.current_dynamixel_position <= self.fully_winded:
                    self.dynamixel_velocity.angular.z = 0.0
                    self.current_state = "nothing"
                else:
                    self.dynamixel_velocity.angular.z = self.winding_speed
            elif self.current_state == "unwinding":
                if self.current_dynamixel_position >= self.fully_unwinded:
                    self.dynamixel_velocity.angular.z = 0.0
                    self.current_state = "nothing"
                else:
                    self.dynamixel_velocity.angular.z = -self.winding_speed

            self.cmd_vel_pub.publish(self.dynamixel_velocity)
            rate.sleep()

    def jointStateCallback(self, msg):
        self.received_first_measurement = True
        self.current_dynamixel_position = msg.position[0]

    def stateCallback(self, msg):
        if self.current_state == "nothing":
            self.current_state = msg.data
        else:
            print "Denied. Current state is: "
            print self.current_state

if __name__ == '__main__':

    rospy.init_node('dynamixel_net')
    go_to = DynamixelNet()
    go_to.run()

