import rclpy
from rclpy.node import Node

import tf_transformations

from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Imu

from simple_pid import PID

from math import pi


class HeadingTest(Node):

    def __init__(self):
        # ROS stuff
        super().__init__('heading_test')

        self.cmd_pub = self.create_publisher(Int8MultiArray, 'cmd_raw', 10)

        self.imu_sub = self.create_subscription(Imu, 'bno055/imu', self.imu_cb, 10)

        self.declare_parameter('surge', 20)
        surge_speed = self.get_parameter('surge').get_parameter_value().integer_value
        self.declare_parameter('heading', 0)
        heading = self.get_parameter('heading').get_parameter_value().integer_value
        self.declare_parameter('pid', [0.2,0.01,0.0])
        pid = self.get_parameter('pid').get_parameter_value().double_array_value

        self.cmd = [surge_speed, surge_speed]

        # Setup yaw PI controller
        self.yaw_controller = PID(pid[0], pid[1], pid[2], setpoint=heading)
        self.yaw_controller.output_limits = (-50, 50)



    def imu_cb(self, msg):

        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(quat)

        yaw *= 180/pi

        yaw_speed = int(self.yaw_controller(yaw))
        print(yaw_speed)
                    
        cmd = Int8MultiArray()
        cmd.data = [self.cmd[0] - yaw_speed, self.cmd[1] + yaw_speed]
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    heading_test = HeadingTest()
    rclpy.spin(heading_test)

    heading_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()