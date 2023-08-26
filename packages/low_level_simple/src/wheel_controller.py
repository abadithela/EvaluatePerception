#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Header
from duckietown_msgs.msg import WheelsCmdStamped
from pid import PID
import pdb
#from as_msgs.msg import WheelOdometry

class SpeedControlNode(DTROS):

    def __init__(self, node_name, robot_name):
        # initialize the DTROS parent class
        super(SpeedControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.robot_name = robot_name
        self.wheel_pub_topic = "/"+robot_name+"/wheels_driver_node/wheels_cmd"
        self.wheel_pub = rospy.Publisher(self.wheel_pub_topic, WheelsCmdStamped, queue_size=1)
        self.left_speed = 0
        self.right_speed = 0
        self.left_goal = 0.1
        self.right_goal = 0.1
        self.left_pid = PID(1,1,0.03,0)
        self.right_pid = PID(1,1,0.03,0)

        self.left_pid.set(self.left_goal)
        self.right_pid.set(self.right_goal)

        self.control = 0
        
        self.ctrl_topic_name = "/"+robot_name+"/high_level_cmd"
        self.control_sub = rospy.Subscriber(self.ctrl_topic_name, String, self.control_cmd)
        # self.odometery_sub = rospy.Subscriber(f"~wheel_odometry/odometry",WheelOdometry,self.odometry_cb)
        # self.goal_speed_sub = rospy.Subscriber(f"~speed_control/goal",WheelOdometry,self.goal_speed_cb)
        
        self.log("Initialized")

    def control_cmd(self, data):
        self.control = float(data.data)

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(10) # 1Hz
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            self.log("sending command")

            wheelsCmd = WheelsCmdStamped()
            header = Header()
            wheelsCmd.header = header
            header.stamp = rospy.Time.now()
            # Running the bot on different velocities at different time interval
            if self.control == 1:
                self.left_pid.set(0.043)
                self.right_pid.set(0.053)
                wheelsCmd.vel_right = self.right_pid.update(self.right_speed,0.1)
                wheelsCmd.vel_left = self.left_pid.update(self.left_speed,0.1)
                #rospy.loginfo("Next control command: '%s'" % message)
            else:
               wheelsCmd.vel_right = 0
               wheelsCmd.vel_left = 0


            self.wheel_pub.publish(wheelsCmd)
            
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = SpeedControlNode(node_name='speed_control_node', robot_name="emma")
    # run node
    node.run()
    # keep spinning
    rospy.spin()