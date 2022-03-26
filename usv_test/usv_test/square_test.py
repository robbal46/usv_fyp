import rclpy
from rclpy.node import Node

import tf_transformations

from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Imu

from simple_pid import PID

from math import pi


class SquareTest(Node):

    def __init__(self):
        # ROS stuff
        super().__init__('square_test')

        self.cmd_pub = self.create_publisher(Int8MultiArray, 'cmd_raw', 10)

        self.imu_sub = self.create_subscription(Imu, 'bno055/imu', self.imu_cb, 10)

        self.declare_parameter('surge_time', 5.0)
        self.surge_time = self.get_parameter('surge_time').get_parameter_value().double_value
        self.declare_parameter('surge_speed', 20)
        self.surge_speed = self.get_parameter('surge_speed').get_parameter_value().integer_value

        self.declare_parameter('turns', [90])
        self.turns = self.get_parameter('turns').get_parameter_value().integer_array_value  

        self.declare_parameter('pid', [0.2,0.0,0.0])
        pid = self.get_parameter('pid').get_parameter_value().double_array_value      


        self.datum = None
        self.heading = 0

        self.turning = False
        self.executed = 0

        self.cmd = [0,0]

        # Setup yaw PI controller
        self.yaw_controller = PID(pid[0], pid[1], pid[2])
        self.yaw_controller.output_limits = (-50, 50)

        self.surge()


    def surge(self):
        print('surge')
        self.cmd = [self.surge_speed, self.surge_speed]
        self.surge_timer = self.create_timer(self.surge_time, self.surge_timer_cb)

    def surge_timer_cb(self):
        print('stop')
        self.cmd = [0,0]
        self.destroy_timer(self.surge_timer)
        if self.executed < len(self.turns):
            self.turn()


    def turn(self):
        print('turn')
        self.heading += self.turns[self.executed]  
        if self.heading > 180:
            self.heading -= 360
        elif self.heading < -180:
            self.heading += 360 

        self.yaw_controller.setpoint = self.heading
        
        self.turning = True  



    def imu_cb(self, msg):

        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(quat)

        yaw *= 180/pi
        # if yaw < 0:
        #     yaw += 360

        # Set datum
        if self.datum == None:
            self.datum = yaw
            self.heading = yaw - self.datum
    
        # Calculate relative yaw from datum
        rel_yaw = yaw - self.datum
        if rel_yaw < 0:
            rel_yaw += 360
        if rel_yaw > 180:
            rel_yaw -= 360
        elif rel_yaw < -180:
            rel_yaw += 360
        

        if self.turning == True:
            if abs(rel_yaw - self.heading) < 5:
                self.turning = False
                self.executed += 1
                self.surge()


        yaw_speed = self.yaw_controller(rel_yaw)

        cmd = Int8MultiArray()               
        cmd.data = [self.cmd[0] - int(yaw_speed), self.cmd[1] + int(yaw_speed)]
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    square_test = SquareTest()
    rclpy.spin(square_test)

    square_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()