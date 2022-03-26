import rclpy

from rclpy.node import Node

#from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray


class VelocityTest(Node):

    def __init__(self):
        super().__init__('velocity_test')

        self.vel_pub = self.create_publisher(Int8MultiArray, 'cmd_raw', 10)

        self.declare_parameter('surge', 10)
        self.declare_parameter('yaw', 0)
        self.declare_parameter('offsets', [0,0])

        x_vel = self.get_parameter('surge').get_parameter_value().integer_value
        yaw_vel = self.get_parameter('yaw').get_parameter_value().integer_value
        offsets = self.get_parameter('offsets').get_parameter_value().integer_array_value

        left = (x_vel - yaw_vel) + offsets[0]
        right = (x_vel + yaw_vel) + offsets[1]


        cmd = Int8MultiArray()
        cmd.data = [left, right]

        self.vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    velocity_test = VelocityTest()
    rclpy.spin(velocity_test)

    velocity_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()