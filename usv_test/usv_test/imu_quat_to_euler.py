import rclpy
from rclpy.node import Node

from tf_transformations import euler_from_quaternion

from sensor_msgs.msg import Imu

from math import pi

class ImuQuatToEuler(Node):

    def __init__(self):
        super().__init__('imu_quat_to_euler')

        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

    def imu_cb(self, msg):
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)

        roll *= 180/pi
        pitch *= 180/pi
        yaw *= 180/pi

        print(f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}')


def main(args=None):
    rclpy.init(args=args)
    q2e = ImuQuatToEuler()
    rclpy.spin(q2e)

if __name__ == '__main__':
    main()

