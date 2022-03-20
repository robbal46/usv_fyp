import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist


class VelocityTest(Node):

    def __init__(self):
        super().__init__('velocity_test')

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.declare_parameter('surge', 1.0)
        self.declare_parameter('yaw', 0.0)
        x_vel = self.get_parameter('surge').get_parameter_value().double_value
        yaw_vel = self.get_parameter('yaw').get_parameter_value().double_value

        cmd = Twist()
        cmd.linear.x = x_vel
        cmd.angular.z = yaw_vel

        self.vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    velocity_test = VelocityTest()
    rclpy.spin(velocity_test)

    velocity_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()