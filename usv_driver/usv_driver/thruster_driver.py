import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray

class ThrusterDriver(Node):

    def __init__(self):
        super().__init__('thruster_driver')

        self.vel_sub = self.create_subscription(Twist, 'cmd_vel', self.vel_cmd_callback, 10)
        self.cmd_sub = self.create_subscription(Int8MultiArray, 'cmd_raw', self.raw_cmd_callback, 10)

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.ser = serial.Serial(port, baud)


    # Velocity cmd from nav stack - take and convert to thrust
    def vel_cmd_callback(self, msg):
        # placeholder, work out kinematics and control
        x_vel = msg.linear.x
        yaw_vel = msg.angular.z

        thrust_l = (x_vel - yaw_vel) * 100
        thrust_r = (x_vel + yaw_vel) * 100

        self.set_thrust(thrust_l, thrust_r)

    def raw_cmd_callback(self, msg):
        self.set_thrust(msg.data[0], msg.data[1])


    def set_thrust(self, left, right):
        thrust_l = abs(int(left))
        dir_l = 0 if left >= 0 else 1
        thrust_r = abs(int(right))
        dir_r = 0 if right >= 0 else 1

        cmd = bytearray([thrust_l, dir_l, thrust_r, dir_r, 0xFF])

        self.ser.write(cmd)


def main(args=None):
    rclpy.init(args=args)

    thruster_driver = ThrusterDriver()
    rclpy.spin(thruster_driver)

    thruster_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        



