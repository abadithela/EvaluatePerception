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
from geometry_msgs.msg import PoseStamped
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, LanePose
from include.differential_drive_kinematics import DifferentialDriveKinematics
from duckietown.dtros import DTROS, NodeType, TopicType


class HeadingRefPP(DTROS):
    def __init__(self, node_name, robot_name, gain=1, trim=0.0):
        super(HeadingRefPP, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospy.loginfo("Initializing")

        #Robot params      -------- CHANGE THIS  ---------       
        self.robot_width = 0.089     # in m
        self.wheel_radius = 0.032   # in m
        self.robot_name = robot_name
        self.origin_x_1 = -1.72  #bottom of mat bottom corner/middle (must be right most in lab)
        self.origin_x_2 = -1.574  #bottom of mat bottom corner/middle (must be left most in lab)
        self.origin_lane_center = np.abs((self.origin_x_1 + self.origin_x_2)/2) # center of lane/not yellow line
        self.lane_width = self.origin_lane_center - self.origin_x_1
        self.lateral_ref = 0.114 # m

        # IMU ---------------------------CHANGE THIS WITH OPTITRACK HEADING--------------------------------------------
        self.heading = 0
        self.state = '1'
        position_sub_topic_name = "/vrpn_client_node/"+self.robot_name+"/pose"
        self.position_sub = rospy.Subscriber(position_sub_topic_name, PoseStamped, self.position_sub_cb)
        self.heading_sub = rospy.Subscriber(f"/{self.robot_name}/heading", Float32, callback = self.heading_callback)
        self.control = 0

        #Wheel speed speed tracking
        self.heading_ref = 0
        self.L = 0.2 # m. 

        #Wheel speed publisher
        self.pub_heading_ref = rospy.Publisher(f"/{self.robot_name}/heading_ref", Float32, queue_size=5)
        self.pub_lane_pose = rospy.Publisher(f"/{self.robot_name}/lane_filter_node/lane_pose", LanePose, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        rospy.loginfo("pure pursuit initialized")

    def position_sub_cb(self, data):
        self.x = np.abs(data.pose.position.x)
        self.z = data.pose.position.z

    def heading_callback(self, heading):
        self.heading = heading.data

    def compute_heading_ref(self):
        d = self.x - self.origin_lane_center
        self.heading_ref = np.arctan(d/self.L)
        print("Computing heading reference at ")
        print("Heading reference: '%f'" % self.heading_ref)
        print("d to reference: '%f'" % d)
        print("Absolute position: '%f'" % self.x)
        print("Lane center: '%f'" % self.origin_lane_center)
        print(" ")
    

    def publish_heading_ref(self):
        """Publishes heading reference
        """
        self.pub_heading_ref.publish(self.heading_ref)
        

    def run_publish_heading_cmd(self):
        rate = rospy.Rate(1) # 1Hz
        rate.sleep()
        while not rospy.is_shutdown():
            self.compute_heading_ref()
            self.publish_heading_ref()
            rate.sleep()
        
        
if __name__ == "__main__":
    node = HeadingRefPP(node_name="heading_ref_pub", robot_name="emma")
    node.run_publish_heading_cmd()
    rospy.spin()