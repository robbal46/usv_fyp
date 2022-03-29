import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from math import pi
from simple_pid import PID


class VelocityPIDController(Node):

    def __init__(self):
        super().__init__('velocity_pid_controller')      
        

        self.declare_parameter('angular', True)
        self.angular = self.get_parameter('angular').get_parameter_value().bool_value
        self.declare_parameter('relative', True)
        self.relative = self.get_parameter('relative').get_parameter_value().bool_value        

        if self.angular:
            self.vel_pub = self.create_publisher(Twist, '/cmd_vel/yaw', 10)

            self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

            self.create_subscription(Int16, '/target/yaw', self.yaw_cb, 10)

            self.datum = None

            self.target = 0

        else:
            self.target = [0,0]


        # Setup PID controller
        self.declare_parameter('pid', [0.2,0.0,0.0])
        gains = self.get_parameter('pid').get_parameter_value().double_array_value
        self.pid = PID(gains[0], gains[1], gains[2])


    def imu_cb(self, msg):
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)

        yaw *= 180/pi

        if self.relative:            
            if self.datum == None:
                self.datum = yaw

            yaw -= self.datum
        
        angle = self.target - yaw
        if angle > 180:
            angle -= 360
        elif angle < -180:
            angle += 360

        vel = self.pid(angle)

        cmd = Twist()
        cmd.angular.z = -vel
        self.vel_pub.publish(cmd)


    def yaw_cb(self, msg):
        self.target = msg.data

        


            

def main(args=None):
    rclpy.init(args=args)
    
    controller = VelocityPIDController()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





