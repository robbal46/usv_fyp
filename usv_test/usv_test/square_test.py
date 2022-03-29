import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8, Int16
from sensor_msgs.msg import Imu

from tf_transformations import euler_from_quaternion
from math import pi

class SquareTest(Node):

    def __init__(self):
        # ROS stuff
        super().__init__('square_test')

        self.yaw_pub = self.create_publisher(Int16, '/target/yaw', 10)
        self.surge_pub = self.create_publisher(Int8, '/thrusters/surge', 10)

        self.yaw_thrust_sub = self.create_subscription(Int8, '/thrusters/yaw', self.yaw_cb, 10)

        self.declare_parameter('surge_time', 5.0)
        self.surge_time = self.get_parameter('surge_time').get_parameter_value().double_value
        self.declare_parameter('surge_speed', 20)
        self.surge_speed = self.get_parameter('surge_speed').get_parameter_value().integer_value

        self.declare_parameter('turns', [90])
        self.turns = self.get_parameter('turns').get_parameter_value().integer_array_value   

        self.check = self.create_timer(0.5, self.check_timer) 

        self.heading = 0
        self.datum = None
        self.turning = False
        self.executed = 0
        self.effort = 0

        # Start by surging
        self.surge()


    def surge(self):
        self.get_logger().info('Surge')

        surge = Int8()
        surge.data = self.surge_speed
        self.surge_pub.publish(surge)

        self.surge_timer = self.create_timer(self.surge_time, self.surge_timer_cb)

    def surge_timer_cb(self):
        self.get_logger().info('Stop')
        
        self.destroy_timer(self.surge_timer)

        if self.executed < len(self.turns):
            self.turn()
        else:
            self.get_logger().info('Finished')

            stop = Int8()
            stop.data = 0
            self.surge_pub.publish(stop)


    def turn(self):
        self.get_logger().info('Turning')

        self.heading += self.turns[self.executed] 
        # Convert t0 +/- 180 - might break if given > 360 rotation, don't do that
        if self.heading > 180:
            self.heading -= 360
        elif self.heading < -180:
            self.heading += 360

        cmd = Int16()
        cmd.data = self.heading
        self.yaw_pub.publish(cmd)

        self.turning = True  
        self.executed += 1


    def yaw_cb(self, msg):
        self.effort = msg.data
        
                
    def check_timer(self):
        if self.turning:
            if abs(self.effort) <= 1:
                self.turning = False
                self.surge()




def main(args=None):
    rclpy.init(args=args)

    square_test = SquareTest()
    rclpy.spin(square_test)

    square_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()