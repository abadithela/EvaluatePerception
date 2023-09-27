#!/usr/bin/env python3
import rospy

class vrpn_speed_calculator():
    def __init__(self, node_name, robot_name) -> None:
        pass
    
    def callback(self):
        pass
    
    def run(self):
        pass


if __name__ == '__main__':
    # create the node
    node = vrpn_speed_calculator(node_name='vrpn_speed_calculator', robot_name="duck7")

    rospy.spin()