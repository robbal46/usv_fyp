import rclpy

from rclpy.node import Node

from math import exp


from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class VelocityLogisticController(Node):

    def __init__(self):
        super().__init__('velocity_sigmoid_controller')

        # Subscribers
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb)

        # Parameters
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('zero_band', 0.1)
        self.declare_parameter('distance_to_max', 2.0)

        self.vel_max = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.zero_band = self.get_parameter('zero_band').get_parameter_value().double_value
        self.dist_to_max = self.get_parameter('distance_to_max').get_parameter_value().double_value


    def demand_vel(self, dist):
        tol = 5 # e^-tol

        # Logistic function params
        L = self.vel_max
        k = 10/self.dist_to_max
        x0 = tol/k + self.zero_band        

        vel = L/(1 + exp(-k * (dist - x0))) + L/(1 + exp(-k * (dist + x0))) - L

        return vel
