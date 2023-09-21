#!/usr/bin/env python3

import os
import rospy
import os
import math

import numpy as np
from pathlib import Path

# from sensor_msgs.msg import Imu
from std_msgs.msg import Header, Float32, String
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from include.differential_drive_kinematics import DifferentialDriveKinematics
from duckietown.dtros import DTROS, NodeType


class HeadingTracker(DTROS):
    def __init__(self, node_name, robot_name, gain=1, trim=0.0):
        super(HeadingTracker, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.loginfo("Initializing")

        #Robot params      -------- CHANGE THIS  ---------       
        self.robot_width = 0.089     # in m
        self.wheel_radius = 0.032   # in m
        self.robot_name = robot_name

        # IMU ---------------------------CHANGE THIS WITH OPTITRACK HEADING--------------------------------------------
        self.heading = 0
        self.state = '1'
        # self.imu_sub = rospy.Subscriber(f"/{self.robot_name}/imu_node/data", Imu, callback=self.integrate)
        self.heading_sub = rospy.Subscriber(f"/{self.robot_name}/heading", Float32, callback = self.heading_callback)
        self.heading_ref_sub = rospy.Subscriber(f"/{self.robot_name}/heading_ref", Float32, callback = self.heading_ref_cb)
        self.control = 0
        self.ctrl_topic_name = "/"+robot_name+"/high_level_cmd"
        self.control_sub = rospy.Subscriber(self.ctrl_topic_name, String, self.control_cmd)

        #Wheel speed speed tracking
        self.vleft  = 0
        self.vleft = 0
        self.left_wheel_duty = 0
        self.right_wheel_duty = 0
        self.k_r_inv = (gain + trim)/27.0
        self.k_l_inv = (gain-trim)/27.0
        self.ktheta = -5

        #Wheel speed publisher
        self.pub_wheel_cmd = rospy.Publisher(f"/{self.robot_name}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)

        rospy.loginfo("Heading PID initialized. ")

    def control_cmd(self, data):
        self.control = float(data.data)

    def heading_callback(self, heading):
        self.heading = heading.data * math.pi/180 # Convert to radians
    
    def heading_ref_cb(self, heading_ref):
        self.heading_ref = heading_ref.data

    def high_level_state_cb(self, data):
        self.state = data.data
        # print(" + High level cmd = " + str(self.state))
    
    def publish_wheel_cmd_input(self):
        self.right_wheel_duty = np.max([np.min([self.right_wheel_duty, 1]), -1])
        self.left_wheel_duty = np.max([np.min([self.left_wheel_duty, 1]), -1])
        print("Left wheel command: '%f'" % self.left_wheel_duty)
        print("Right wheel command: '%f'" % self.right_wheel_duty)

        wheelsCmd = WheelsCmdStamped()
        header = Header()
        wheelsCmd.header = header
        header.stamp = rospy.Time.now()
        wheelsCmd.vel_left = self.left_wheel_duty
        wheelsCmd.vel_right = self.right_wheel_duty
        self.pub_wheel_cmd.publish(wheelsCmd)

    def track_heading_w_wheel_cmd(self, des_heading, des_speed):
        """
        based on https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/amod/AMOD_2020/20201019-05%20-%20ETHZ%20-%20Control%20in%20Duckietown%20(PID).pdf
        des_heading = theta_r (theta_reference in the robot's frame of reference)
        des_speed = V_r (velocity_reference in robot's frame of reference)
        """
        heading_error = des_heading - self.heading
        # Va = des_speed*np.cos(heading_error)
        # omega = des_speed/self.robot_width*np.sin(heading_error)
        Va = des_speed
        omega = self.ktheta*(heading_error)
        print("Computing wheel speeds ")
        print("current heading: '%f'" % self.heading)
        print("heading error: '%f'" % heading_error)
        print("omega: '%f'" % omega)
        self.vleft  = (Va - 0.5 * omega * self.robot_width) 
        self.vright = (Va + 0.5 * omega * self.robot_width) 
        self.left_wheel_duty = self.vleft/self.wheel_radius * self.k_l_inv
        self.right_wheel_duty = self.vright/self.wheel_radius * self.k_r_inv
        print("Left wheel speed: '%f'" % self.vleft)
        print("Right wheel speed: '%f'" % self.vright)
        print("Left wheel duty: '%f'" % self.left_wheel_duty)
        print("Right wheel duty: '%f'" % self.right_wheel_duty)
        print(" ")

        self.publish_wheel_cmd_input()
    
    def run_wheel_cmd(self):
        rate = rospy.Rate(1)
        rospy.loginfo("Trying to run wheel commANDS")
        rate.sleep()
        # while not rospy.is_shutdown():
        while not rospy.is_shutdown():
            if self.control == 1:
                des_heading = self.heading_ref
                des_speed = 0.2
                self.track_heading_w_wheel_cmd(des_heading, des_speed)
                rate.sleep()
            else:
                self.on_shutdown()
            
    def on_shutdown(self):
    #send a zero velocity wheel command
        wheelsCmd = WheelsCmdStamped()
        header = Header()
        wheelsCmd.header = header
        header.stamp = rospy.Time.now()
        wheelsCmd.vel_right = 0
        wheelsCmd.vel_left = 0
        self.pub_wheel_cmd.publish(wheelsCmd)
        
        
if __name__ == "__main__":
    node = HeadingTracker(node_name="heading_tracker", robot_name="emma")
    node.run_wheel_cmd()
    rospy.spin()
