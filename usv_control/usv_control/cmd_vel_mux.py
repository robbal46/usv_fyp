# Takes in multiple velocity sources (linear, angular) from
# different nodes and combines them into one topic

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class CmdVelMux(Node):

    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.vel_pub = self.create_subscription(Twist, '/cmd_vel', 10)

        self.linear_sub = self.create_subscription(Twist, '/cmd_vel/surge', self.surge_cb, 10)
        self.angular_sub = self.create_subscription(Twist, '/cmd_vel/yaw', self.yaw_cb, 10)

        self.cmd = Twist()

    def yaw_cb(self, msg):
        yaw = msg.angular.z
        self.cmd.angular.z = yaw

        self.vel_pub.publish(self.cmd)
    
    def surge_cb(self, msg):
        surge = msg.linear.x
        self.cmd.linear.x = surge

        self.vel_pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)

    mux = CmdVelMux()
    rclpy.spin(mux)

    mux.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()