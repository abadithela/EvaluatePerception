#!/usr/bin/env python3
'''
Script to drive robot forward by one step.
'''
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class simple_controller(DTROS):

    def __init__(self, node_name, robot_name):
        # initialize the DTROS parent class
        super(simple_controller, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct subscriber for the robot:
        self.robot_name = robot_name
        self.state = 1
        sub_topic_name = "/"+robot_name+"/abstract_state/pose"
        self.sub = rospy.Subscriber(sub_topic_name, String, self.callback)
        pub_topic_name = "/"+robot_name+"/high_level_cmd"
        self.pub_ctrl = rospy.Publisher(pub_topic_name, String, queue_size=10)

    def callback(self, data):
        self.state = int(data.data)

    def next_command(self):
        if self.state <= 5:
            speed = 1
        elif self.state == 6:
            speed = 0
        return speed

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            next_cmd = self.next_command()
            message = str(next_cmd)
            rospy.loginfo("Next control command: '%s'" % message)
            self.pub_ctrl.publish(message)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = simple_controller(node_name='hlp', robot_name="emma")
    # run node
    node.run()
    # keep spinning
    rospy.spin()