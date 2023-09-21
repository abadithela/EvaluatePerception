#!/usr/bin/env python3
# Computes reference heading from simple pure pursuit
import os
import rospy
import os
import math

import numpy as np
from pathlib import Path

# from sensor_msgs.msg import Imu
from std_msgs.msg import Header, Float32, String
from geometry_msgs.msg import PoseStamped, WheelsCmdStamped
from duckietown_msgs.msg import LanePose
from include.differential_drive_kinematics import DifferentialDriveKinematics
from duckietown.dtros import DTROS, NodeType, TopicType


class LanePosePublisher(DTROS):
    def __init__(self, node_name, robot_name, gain=1, trim=0.0):
        super(LanePosePublisher, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.loginfo("Initializing")

        #Robot params      -------- CHANGE THIS  ---------       
        self.robot_width = 0.089     # in m
        self.wheel_radius = 0.032   # in m
        self.robot_name = robot_name
        self.origin_x_1 = -1.72  #bottom of mat bottom corner/middle (must be right most in lab)
        self.origin_x_2 = -1.574  #bottom of mat bottom corner/middle (must be left most in lab)
        self.origin_lane_center = (self.origin_x_1 + self.origin_x_2)/2
        self.lane_width = self.origin_lane_center - self.origin_x_1

        # IMU ---------------------------CHANGE THIS WITH OPTITRACK HEADING--------------------------------------------
        self.heading = 0
        self.state = '1'
        position_sub_topic_name = "/vrpn_client_node/"+self.robot_name+"/pose"
        self.state_sub = rospy.Subscriber(f"/{self.robot_name}/abstract_state/pose", String, callback = self.high_level_state_cb)

        self.position_sub = rospy.Subscriber(position_sub_topic_name, PoseStamped, self.position_sub_cb)
        self.heading_sub = rospy.Subscriber(f"/{self.robot_name}/heading", Float32, callback = self.heading_callback)
        self.control = 0

        #Wheel speed speed tracking
        self.heading_ref = 0
        self.lateral_ref = 0.114 # m
        self.L = 0.2 # m. 

        #Wheel speed publisher
        self.pub_lane_pose = rospy.Publisher(f"/{self.robot_name}/lane_filter_node/lane_pose", LanePose, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        rospy.loginfo("pure pursuit initialized")

    def high_level_state_cb(self, data):
        self.state = data.data
        # print(" + High level cmd = " + str(self.state))

    def position_sub_cb(self, data):
        self.x = data.pose.position.x
        self.z = data.pose.position.z

    def heading_callback(self, heading):
        self.heading = heading.data

    def publish_lane_pose(self):
        """Publishes heading reference
        """
        lanePose = LanePose()
        header = Header()
        lanePose.header = header
        lanePose.d = self.x - self.origin_lane_center
        lanePose.d_ref = -1*self.lateral_ref
        if self.x <= self.origin_x_2 and self.x >= self.origin_x_1:
            lanePose.in_lane = True
        else:
            lanePose.in_lane = False
        lanePose.phi = self.heading
        lanePose.phi_ref = self.heading_ref
        self.pub_lane_pose.publish(lanePose)

    def run_publish_lane_pose(self):
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            self.publish_lane_pose()
            rate.sleep()
            
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
    node = LanePosePublisher(node_name="lane_pose_pub", robot_name="emma")
    node.run_publish_lane_pose()
    rospy.spin()