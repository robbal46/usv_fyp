# YawController - PID controlled yaw using IMU angular velocity

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from simple_pid import PID

class SurgeController(Node):

    def __init__(self):
        super().__init__('surge_controller')

        self.surge_pub = self.create_publisher(Int8, '/thrusters/surge', 10)

        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_cb, 10)

        self.declare_parameter('pid', [10.0, 0.0, 0.0])
        pid = self.get_parameter('pid').get_parameter_value().double_array_value

        self.controller = PID(pid[0], pid[1], pid[2])
        self.controller.output_limits = (-100,100)

        self.vel = 0
        self.t_prev = 0

    # Get desired yaw velocity from cmd_vel and make PID setpoint
    def vel_cb(self, msg):
        target_vel = msg.linear.x
        self.controller.setpoint = target_vel

    # Drive PID loop in IMU callback
    def imu_cb(self, msg):
        x_accel = msg.linear_acceleration.x


        # Integrate acceleration from IMU to get velocity
        # This is absolutely useless btw
        t_now = msg.header.stamp.nanosec / 1e9
        self.vel += x_accel * (t_now - self.t_prev)      
        self.t_prev = t_now

        self.int_count = 0

        self.get_logger().info(f'Velocity: {self.vel}')       


        surge_effort = int(self.controller(self.vel))

        cmd = Int8()
        cmd.data = surge_effort
        self.surge_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    surge_controller = SurgeController()
    rclpy.spin(surge_controller)

    surge_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






