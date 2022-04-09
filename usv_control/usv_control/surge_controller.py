# Surge controller

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist


class SurgeController(Node):

    def __init__(self):
        super().__init__('surge_controller')

        self.surge_pub = self.create_publisher(Int8, '/thrusters/surge', 10)

        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_cb, 10)  

        self.declare_parameter('order', 3)
        self.order = self.get_parameter('order').get_parameter_value().integer_value     
        

    # Convert to thrust - open loop control
    def vel_cb(self, msg):
        target_vel = msg.linear.x

        thrust = self.vel_model(target_vel)

        surge = Int8()
        surge.data = thrust
        self.surge_pub.publish(surge)

    
    def vel_model(self, x):
        # Didn't collect backwards data
        if x < 0:
            x = 0
        # Small deadzone
        if x < 0.05:
            x = 0

        # Quadratic fit of experimental data
        if self.order == 2:
            thrust = 48.84*(x*x) + 9.775*x + 2.081
        # Cubic fit
        elif self.order == 3:
            thrust = 66.99*(x*x*x) - 76.18*(x*x) + 65.22*x - 0.5194
        else:
            thrust = 0
            self.get_logger().error('Invalid order parameter (2 or 3 accepted)')

        if thrust > 127:
            thrust = 127
        
        return int(thrust)


def main(args=None):
    rclpy.init(args=args)
    
    surge_controller = SurgeController()
    rclpy.spin(surge_controller)

    surge_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






