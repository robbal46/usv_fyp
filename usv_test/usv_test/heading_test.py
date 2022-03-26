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

        self.declare_parameter('surge_time', 5.0)
        self.surge_time = self.get_parameter('surge_time').get_parameter_value().double_value
        self.declare_parameter('surge_speed', 20)
        self.surge_speed = self.get_parameter('surge_speed').get_parameter_value().integer_value

        


        self.datum = None
        self.target = 0
        self.turns = 0

        # Setup yaw PI controller
        self.yaw_controller = PID(1,0.1,0)
        self.yaw_controller.output_limits = (-50, 50)

    def execute(self):
        self.surge(self.surge_speed)

    def surge(self):
        cmd = Int8MultiArray()
        cmd.data = [self.surge_speed, self.surge_speed]
        self.cmd_pub.publish(cmd)
        self.surge_timer = self.create_timer(self.surge_time, self.surge_timer_cb)

    def surge_timer_cb(self):
        cmd = Int8MultiArray()
        cmd.data = [0,0]
        self.cmd_pub.publish(cmd)
        self.destroy_timer(self.surge_timer)

        self.yaw()


    def yaw(self):
        self.target += (pi/2)       



    def imu_cb(self, msg):

        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(quat)


        if self.datum == None:
            self.datum = yaw
            print(self.datum)
            self.target = yaw
        else:
            rel_yaw = yaw - self.datum
            print(rel_yaw)
                
            yaw_err = self.target - rel_yaw       

            yaw_speed = int(self.yaw_controller(yaw_err))

            cmd = Int8MultiArray()
            cmd.data = [yaw_speed, -1*yaw_speed]
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    heading_test = HeadingTest()
    rclpy.spin(heading_test)

    heading_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()