#!/usr/bin/env python3

import os
import rospy
import os
import math

import numpy as np
from pathlib import Path

from include.PIDF import PIDF

# from sensor_msgs.msg import Imu
from as_msgs.msg import WheelOdometry
from std_msgs.msg import Header, Float32
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from offset_calculator import OffsetCalculator
from include.differential_drive_kinematics import DifferentialDriveKinematics


class HeadingTracker():
    def __init__(self, robot_name):
        rospy.loginfo("Initializing")

        #Robot params      -------- CHANGE THIS  ---------       
        self.robot_width = 0.254     # in m
        self.wheel_radius = 0.0212    # in m
        self.length = 0.01 # in m 

        self.heading_kP = 0.02
        self.heading_kI = 0
        self.heading_kD = 0

        self.left_wheel_kP = 0.02
        self.left_wheel_kI = 0
        self.left_wheel_kD = 0

        self.right_wheel_kP = 0.02
        self.right_wheel_kI = 0
        self.right_wheel_kD = 0

        self.robot_name = robot_name

        # IMU ---------------------------CHANGE THIS WITH OPTITRACK HEADING--------------------------------------------
        self.heading = 0
        # self.imu_sub = rospy.Subscriber(f"/{self.robot_name}/imu_node/data", Imu, callback=self.integrate)
        self.heading_sub = rospy.Subscriber(f"/{self.robot_name}/heading", Float32, callback = self.heading_callback)
        #Wheel speed speed tracking
        self.left_speed  = -255
        self.right_speed = -255
        self.odometery_sub = rospy.Subscriber(f"/{self.robot_name}/wheel_odometry/odometry",WheelOdometry,self.odometry_cb)

        #Wheel speed publisher
        self.wheel_pub = rospy.Publisher(f"/{self.robot_name}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)

        # Input publisher to joy mapper:
        car_cmd_topic = f"/{self.robot_name}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic, Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        #Wheel speed PIDs
        self.left_pid  = PIDF(self.left_wheel_kP,  self.left_wheel_kI,  self.left_wheel_kD,  0)
        self.right_pid = PIDF(self.right_wheel_kP, self.right_wheel_kI, self.right_wheel_kD, 0)
        self.heading_pid = PIDF(self.heading_kP, self.heading_kI, self.heading_kD, 0)

        # rospy.loginfo("     CALCULATING IMU OFFSET")
        # offfset_calc = OffsetCalculator()

        rospy.loginfo("initialized")

    def truncate(self, num:float, dec_places:int):
        mult = 10**dec_places
        return int(num * ()) * mult / mult

    def heading_callback(self, heading):
        self.heading = heading.data
        rospy.logdebug(" + Heading values = " + str(self.heading))

    def integrate(self, imu_out):
        """
        Will integrate the yaw rate command from the Gyroscope. Automatically does not get called if #IMU (line 25 is commented out) 
        """
        self.heading = self.heading + (self.truncate(imu_out.angular_velocity.x, 4) - 0.005)
        rospy.logdebug(" + Heading Values = " + str(self.heading))

    def publish_input(self):
        """Publishes a car command message.

        Args:
            u (:obj:`tuple(double, double)`): tuple containing [v, w] for the control action.
        """
        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()
        car_control_msg.v = self.v  # v
        car_control_msg.omega = self.omega  # omega
        self.pub_car_cmd.publish(car_control_msg)
    
    def track_heading_speed(self, des_heading, des_speed):
        self.heading_pid.set(des_heading)
        self.omega = self.heading_pid.update(self.heading)
        self.v = des_speed

    def track_heading_and_speed(self, des_heading, curr_heading, des_speed, curr_speed):
        """
        based on https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/amod/AMOD_2020/20201019-05%20-%20ETHZ%20-%20Control%20in%20Duckietown%20(PID).pdf
        des_heading = theta_r (theta_reference in the robot's frame of reference)
        des_speed = V_r (velocity_reference in robot's frame of reference)
        """
        #CONSTANTS
        dt = 0.01 # Should I calculate this value? self.rospy.Time

        self.heading_pid.set(des_heading)
        omega = self.heading_pid.update(curr_heading)

        left_goal  = des_speed + ((omega * self.robot_width) / self.wheel_radius)
        right_goal = des_speed - ((omega * self.robot_width) / self.wheel_radius)

        if left_goal >= 0.5:
            rospy.loginfo("Left goal too high: ", left_goal)
            left_goal = 0.5
        
        if right_goal >= 0.5:
            rospy.loginfo("right goal too high: ", right_goal)
            right_goal = 0.5
            
        wheelsCmd = WheelsCmdStamped()
        header = Header()
        wheelsCmd.header = header
        header.stamp = rospy.Time.now()
        current_time = rospy.get_time()

        self.left_pid.set(left_goal)
        self.right_pid.set(right_goal)
        wheelsCmd.vel_left = self.left_pid.update(left_goal, dt)
        wheelsCmd.vel_right = self.right_pid.update(right_goal, dt)
        self.wheel_pub.publish(wheelsCmd)


    def set_wheel_speeds(self, left_speed:float, right_speed:float):
        wheelsCmd = WheelsCmdStamped()
        header = Header()
        wheelsCmd.header = header
        header.stamp = rospy.Time.now()
        current_time = rospy.get_time()

        wheelsCmd.vel_left = self.left_pid.update(left_speed,0.1)
        wheelsCmd.vel_right = self.right_pid.update(right_speed,0.1)
        self.wheel_pub.publish(wheelsCmd)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            des_heading = 0
            des_speed = 0.1
            self.track_heading_and_speed(des_heading, self.heading, des_speed, self.speed)
            rate.sleep()
    
    def run_heading_tracking(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            heading_ref = 0
            speed_ref = 0.05
            des_heading, des_speed = self.reference_to_robot_frame(heading_ref, speed_ref)
            self.track_heading_speed(des_heading=des_heading, des_speed=des_speed)
            rate.sleep()
            self.publish_input()


    def on_shutdown(self):
    #send a zero velocity wheel command

        wheelsCmd = WheelsCmdStamped()
        header = Header()
        wheelsCmd.header = header
        wheelsCmd.vel_right = 0
        wheelsCmd.vel_left = 0
        self.wheel_pub.publish(wheelsCmd)

    def odometry_cb(self,data):
        self.left_speed = data.left_wheel_velocity
        self.right_speed = data.right_wheel_velocity
        self.log(data)
        
if __name__ == "__main__":
    rospy.init_node(f"offset_calculator_node", log_level=rospy.DEBUG)
    node = HeadingTracker()
    node.run_heading_tracking()
    rospy.spin()
