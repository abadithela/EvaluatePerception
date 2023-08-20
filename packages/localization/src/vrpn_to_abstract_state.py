#!/usr/bin/env python3
'''
Script that subscribes to vrpn_client topics that publish opti-track 
coordinates of the robots. These coordinates are converted to abstract state coordinates as 
defined in coordinate_translator.py.
'''

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import pdb
from coordinate_translator import TileMapTranslator

class vrpn_subscriber(DTROS):

    def __init__(self, node_name, robot_name):
        # initialize the DTROS parent class
        super(vrpn_subscriber, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct subscriber for the robot:
        self.robot_name = robot_name
        self.vrpn_position_x = None
        self.vrpn_position_z = None
        self.vrpn_orientation = None
        self.translator = self.construct_translator()
        sub_topic_name = "/vrpn_client_node/"+self.robot_name+"/pose"
        self.sub = rospy.Subscriber(sub_topic_name, PoseStamped, self.callback)
        pub_topic_name = "/"+robot_name+"/abstract_state/pose"
        self.pub = rospy.Publisher(pub_topic_name, String, queue_size=10)

    def callback(self, data):
        self.vrpn_position_x = data.pose.position.x
        self.vrpn_position_z = data.pose.position.z
        self.vrpn_orientation = data.pose.orientation
        # rospy.loginfo(rospy.get_caller_id() + 'I heard position %s %s', self.vrpn_position_x, self.vrpn_position_z)
        # rospy.loginfo(rospy.get_caller_id() + 'I heard orientation %s', self.vrpn_orientation)
    
    
    def convert_to_abstract_state(self):
        print(self.vrpn_position_x)
        if self.vrpn_position_x is not None:
            area_number, relative_y = translator.translate_coordinate(self.vrpn_position_x, self.vrpn_position_z)
            return area_number, relative_y
    
    def publish_abstract_state(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            rate.sleep()
            area_number, relative_y = self.convert_to_abstract_state()
            message = str(area_number)
            rospy.loginfo("Publishing message: '%s'" % message)
            self.pub.publish(message)
            

    def construct_translator(self):
        AREA_WIDTH = 0.146
        AREA_HEIGHT = 0.288931
        origin_x_1 = -1.72  #bottom of mat bottom corner/middle (must be right most in lab)
        origin_y_1 = -1.478
        origin_x_2 = -1.574  #bottom of mat bottom corner/middle (must be left most in lab)
        origin_y_2 = -1.478

        translator = TileMapTranslator(AREA_WIDTH, AREA_HEIGHT, origin_x_1, origin_x_2, origin_y_1, origin_y_2)
        return translator

if __name__ == '__main__':
    # create the node
    node = vrpn_subscriber(node_name='vrpn_subscriber', robot_name="ida")
    translator = node.construct_translator()
    node.publish_abstract_state()

    rospy.spin()
