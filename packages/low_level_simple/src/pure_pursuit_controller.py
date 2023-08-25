'''
Implementing the pure pursuit algorithm given here:
https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/amod/AMOD_2020/20201019-05%20-%20ETHZ%20-%20Control%20in%20Duckietown%20(PID).pdf
'''

from utils.pure_pursuit.diffdrive_kinematics import DifferentialDriveKinematics
from utils.pure_pursuit.pure_pursuit import PurePursuit
from utils.pure_pursuit.point import Point
from utils.pure_pursuit.distance_calculator import EuclideanDistanceCalcualtor
from utils.pure_pursuit.error_calculator import HeadingToPointErrorCalculator
from utils.pure_pursuit.controller import Controller
import matplotlib.pyplot as plt
import rospy
import math
import os

class PurePursuitNode(DTROS):
    def __init__(self, look_ahead, robot_name):
        self.look_ahead = look_ahead
        self.waypoints = [Point(0,1), Point(1,1), Point(2,1), Point(3,1), Point(4,1)]
        self.dist_calc = EuclideanDistanceCalcualtor()
        self.heading_err = HeadingToPointErrorCalculator()
        self.controller = Controller()
        self.robot_name = robot_name
        self.x = 0 # x position in robot coordinate frame
        self.y = 0 # y position in robot coordinate frame
        self.psi = 0 # Angle from longitudinal deviation.
        self.r = rospy.Rate(10) # 10hz
        self.dt = 1.0/self.r
        self.offset_x = math.pi/2
        self.prev_psi = 0
        self.curr_psi = 0

        # Control input commands:
        self.omega = omega
        self.v = v

    def set_waypoints(self, waypoints):
        self.waypoints = waypoints

    def instantiate_purepursuit(self):
        self.p = PurePursuit(self.look_ahead, self.waypoints, self.dist_calc, self.heading_err, self.controller)
    
    def setup_publishers(self):
        # Command publisher
        car_cmd_topic = f"/{self.robot_name}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic, Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

    def setup_subscribers(self):
        '''
        Fill in with orientation subscriber and pose subscriber
        '''
        self.orientation_sub = ...
        self.pose_sub = ...
        pass

    def orientation_callback(self, data):
        '''
        Callback function to store consecutive frames of orientation data
        '''
        self.prev_psi = self.curr_psi
        self.curr_psi = data
    
    def pose_callback(self, data):
        '''
        Callback function to store consecutive frames of orientation data
        '''
        self.x = data.x
        self.y = data.y
    
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


    def compute_theta_dot(self):
        '''
        Function that computes theta_dot numerically 
        from successive optitrack measurements of orientation
        '''
        self.prev_th = self.prev_psi + self.offset_x
        self.curr_th = self.curr_psi + self.offset_x
        self.theta_dot = (self.curr_th - self.prev_th)/self.dt
        self.controller.set_grad_angular_pos(self.theta_dot)
    
    def update_pos(self):
        self.pos = Point(x=self.x, y=self.y, heading=self.psi)

    def control_input(self):
        '''
        Function to receive control input
        '''
        self.compute_theta_dot()
        self.omega, self.v = self.p.update(self.pos)

    
