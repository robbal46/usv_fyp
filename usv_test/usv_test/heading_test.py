import rclpy
from rclpy.node import Node


from std_msgs.msg import Int8, Int16

class HeadingTest(Node):

    def __init__(self):
        # ROS stuff
        super().__init__('heading_test')

        self.yaw_pub = self.create_publisher(Int16, '/target/yaw', 10)

        # Direct control of surge - no feedback controller setup yet
        self.surge_pub = self.create_publisher(Int8, '/thrusters/surge', 10)

        self.declare_parameter('speed', 20)
        self.surge_speed = self.get_parameter('speed').get_parameter_value().integer_value
        self.declare_parameter('heading', 0)
        self.heading = self.get_parameter('heading').get_parameter_value().integer_value


        self.pub_timer = self.create_timer(1.0, self.timer_cb)

    # Publish on timer so not missed
    def timer_cb(self):
        
        yaw = Int16()
        yaw.data = self.heading
        self.yaw_pub.publish(yaw)

        surge = Int8()
        surge.data = self.surge_speed
        self.surge_pub.publish(surge)


def main(args=None):
    rclpy.init(args=args)

    heading_test = HeadingTest()
    rclpy.spin(heading_test)

    heading_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()