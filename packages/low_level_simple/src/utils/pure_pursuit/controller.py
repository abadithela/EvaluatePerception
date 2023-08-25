import math

class Controller:
    """
    Controller interface for purepursuit. Pure Pursuit needs a controller
    that will move the robot in a way that will minimize the error.
    """
    def __init__(self):
        self.R = 0
        self.v = 0

    def set_grad_angular_pos(self, theta_dot):
        '''
        Set gradient of theta from current robot orientations.
        theta is the orientation of the robot from the x-axis.
        '''
        self.theta_dot = theta_dot

    def set_control(L: float, alpha: float):
        '''
        L: Distance from current position to goal point
        alpha: error in heading angle in radians.
        Total cross track error is e = Lsin(alpha)
        '''
        self.R = L/2*math.sin(alpha)
        omega = theta_dot
        v = self.R*omega
        return omega, v
