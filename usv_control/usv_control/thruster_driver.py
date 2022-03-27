import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Int8MultiArray

class ThrusterDriver(Node):

    def __init__(self):
        super().__init__('thruster_driver')

        self.yaw_sub = self.create_subscription(Int8, '/thrusters/yaw', self.yaw_cb, 10)
        self.surge_sub = self.create_subscription(Int8, '/thruster/surge', self.surge_cb, 10)
        self.raw_sub = self.create_subscription(Int8MultiArray, '/thrusters/raw_cmd', self.raw_cmd_cb, 10)

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.ser = serial.Serial(port, baud)

        self.yaw_cmd = 0
        self.surge_cmd = 0
    

    def yaw_cb(self, msg):
        self.yaw_cmd = msg.data
        self.set_thrust()

    def surge_cb(self, msg):
        self.surge_cmd = msg.data
        self.set_thrust()

    def update_thrust(self):
        left = self.surge_cmd - self.yaw_cmd
        right = self.surge_cmd + self.yaw_cmd
        self.set_thrust(left, right)

    def raw_cmd_cb(self, msg):
        self.set_thrust(msg.data[0], msg.data[1])


    def set_thrust(self, left, right):
        if left > 100:
            left = 100
        elif left < -100:
            left = -100
        if right > 100:
            right = 100
        elif right < -100:
            right = -100

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
        



