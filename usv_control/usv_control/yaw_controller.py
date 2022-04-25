# YawController - PID controlled yaw using IMU angular velocity

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from simple_pid import PID

class YawController(Node):

    def __init__(self):
        super().__init__('yaw_controller')

        self.yaw_pub = self.create_publisher(Int8, '/thrusters/yaw', 10)

        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_cb, 10)

        self.declare_parameter('pid', [140.0, 90.0, 4.0])
        gains = self.get_parameter('pid').get_parameter_value().double_array_value

        self.pid = PID(gains[0], gains[1], gains[2])
        self.pid.output_limits = (-100,100)

    # Get desired yaw velocity from cmd_vel and make PID setpoint
    def vel_cb(self, msg):
        target_vel = msg.angular.z
        self.pid.setpoint = target_vel

    # Drive PID loop in IMU callback
    def imu_cb(self, msg):
        yaw_vel = msg.angular_velocity.z

        yaw_effort = int(self.pid(yaw_vel))

        cmd = Int8()
        cmd.data = yaw_effort
        self.yaw_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    yaw_controller = YawController()
    rclpy.spin(yaw_controller)

    yaw_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()