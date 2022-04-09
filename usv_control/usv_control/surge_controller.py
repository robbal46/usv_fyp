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
        

    # Convert to thrust - open loop control
    def vel_cb(self, msg):
        target_vel = msg.linear.x

        thrust = self.vel_model(target_vel)

        surge = Int8()
        surge.data = int(thrust)
        self.surge_pub.publish(surge)

    # Quadratic fit of experimental data
    def vel_model(x):
        thrust = 75.93*(x*x) - 35.96*x + 18.17

        if thrust > 255:
            thrust = 255
        elif thrust < -255:
            thrust = -255
        
        return thrust


def main(args=None):
    rclpy.init(args=args)
    
    surge_controller = SurgeController()
    rclpy.spin(surge_controller)

    surge_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






