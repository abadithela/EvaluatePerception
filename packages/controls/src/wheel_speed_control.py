#!/usr/bin/env python3

"""
    @author: Saeshu karthika
"""

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Float32, Header, String
from duckietown_msgs.msg import WheelsCmdStamped
from include.PIDF import PIDF
from as_msgs.msg import WheelOdometry

class SpeedControlNode(DTROS):
    """

    """

    def __init__(self, node_name,robot_name):
        # initialize the DTROS parent class
        super(SpeedControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.robot_name = robot_name
        self.wheel_pub = rospy.Publisher(f"/{self.robot_name}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.left_speed = 0
        self.right_speed = 0
        self.left_goal = 0.1
        self.right_goal = 0.1
        self.left_pid = PIDF(1,1,0.03,0)
        self.right_pid = PIDF(1,1,0.03,0)

        self.left_pid.set(self.left_goal)
        self.right_pid.set(self.right_goal)
        
        self.odometery_sub = rospy.Subscriber(f"/{self.robot_name}/wheel_odometry/odometry",WheelOdometry,self.odometry_cb)
        # self.goal_speed_sub = rospy.Subscriber(f"~speed_control/goal",WheelOdometry,self.goal_speed_cb)
        
        self.log("Initialized")

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            self.log("sending command")

            wheelsCmd = WheelsCmdStamped()
            header = Header()
            wheelsCmd.header = header
            header.stamp = rospy.Time.now()

            current_time = rospy.get_time()

            # Running the bot on different velocities at different time interval
            if 5 < current_time - start_time <= 25:
                print("slow wheel command")
                wheelsCmd.vel_right = self.right_pid.update(self.right_speed,0.1)
                wheelsCmd.vel_left = self.left_pid.update(self.left_speed,0.1)

            elif 25 < current_time - start_time <= 50:
                self.left_pid.set(0.2)
                self.right_pid.set(0.2)
                print("intermediate wheel command")
                wheelsCmd.vel_right = self.right_pid.update(self.right_speed,0.1)
                wheelsCmd.vel_left = self.left_pid.update(self.left_speed,0.1)

            elif 50 < current_time - start_time <= 75:
                self.left_pid.set(0.3)
                self.right_pid.set(0.3)
                wheelsCmd.vel_right = self.right_pid.update(self.right_speed,0.1)
                wheelsCmd.vel_left = self.left_pid.update(self.left_speed,0.1)
            
            elif 75 < current_time - start_time <= 100:
                self.left_pid.set(0.4)
                self.right_pid.set(0.4)
                wheelsCmd.vel_right = self.right_pid.update(self.right_speed,0.1)
                wheelsCmd.vel_left = self.left_pid.update(self.left_speed,0.1)
            
            elif current_time - start_time > 100:
                self.left_pid.set(0)
                self.right_pid.set(0)
                wheelsCmd.vel_right = self.right_pid.update(self.right_speed,0.1)
                wheelsCmd.vel_left = self.left_pid.update(self.left_speed,0.1)
                print("Running on 0.0")
                
            else:
                print("direct wheels zero")
                wheelsCmd.vel_right = 0
                wheelsCmd.vel_left = 0
            self.wheel_pub.publish(wheelsCmd)
            
            rate.sleep()

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

    def goal_speed_cb(self,data):
        self.left_goal = data.left_wheel_velocity
        self.right_goal = data.right_wheel_velocity
        #self.left_goal = 0.5
        #self.right_goal = 0.5
        self.left_pid.set(self.left_goal)
        self.right_pid.set(self.right_goal)
        self.log(data)


if __name__ == '__main__':
    # create the node
    robot_name = "emma"
    node = SpeedControlNode(node_name='speed_control_node', robot_name="emma")
    # run node
    # node.run()
    node.on_shutdown()
    # keep spinning
    rospy.spin()