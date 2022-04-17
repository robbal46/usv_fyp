import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from tf_transformations import euler_from_quaternion, quaternion_from_euler

from math import exp, pow, sqrt, atan2

class NavToPose(Node):

    def __init__(self):
        super().__init__('nav_to_pose')

        self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, 10)

        self.create_subscription(Odometry, 'odometry/filtered', self.odom_cb, 10)

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.feedback_pub = self.create_publisher(Bool, 'new_goal', 10)

        self.create_timer(0.2, self.timer_cb)


        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Parameters
        self.declare_parameter('pose_tolerance', 1.0)
        self.pose_tol = self.get_parameter('pose_tolerance').get_parameter_value().double_value 

        self.declare_parameter('rotate_to_goal', True)
        self.rotate = self.get_parameter('roatate_to_goal').get_parameter_value().bool_value      


        self.goal_pose = [0,0,0]
        self.current_pose = [0,0,0]

    # Get new goal pose
    def goal_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        quat = [msg.pose.orientation.x, 
                msg.pose.orientation.y, 
                msg.pose.orientation.z, 
                msg.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quat)

        self.goal_pose = [x,y,yaw]

        fb = Bool()
        fb.data = False
        self.feedback_pub.publish(fb)

    # Get current pose from odom
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        quat = [msg.pose.pose.orientation.x, 
                msg.pose.pose.orientation.y, 
                msg.pose.pose.orientation.z, 
                msg.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quat)

        self.current_pose = [x,y,yaw]



    # Get current pose from tf
    def get_current_pose(self):
        now = self.get_clock().now()

        trans = self.buffer.lookup_transform('odom', 'base_link', now)

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        quat = [trans.transform.orientation.x, 
                trans.transform.orientation.y, 
                trans.transform.orientation.z, 
                trans.transform.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quat)

        self.current_pose = [x,y,yaw]

        

    # Update velocity
    def timer_cb(self):

        #self.get_current_pose()

        x_dist = self.goal_pose[0] - self.current_pose[0]
        y_dist = self.goal_pose[1] - self.current_pose[1]
        
        dist_to_goal = sqrt(pow(x_dist,2) + pow(y_dist,2))
        heading = atan2(y_dist, x_dist)
        print(f'Distance: {dist_to_goal}, Heading: {heading}')

        cmd = Twist()

        # Goal reached
        if abs(dist_to_goal) < self.pose_tol:

            #Currently don't care about final heading

            fb = Bool()
            fb.data = True
            self.feedback_pub.publish(fb)

        else:
            heading_error = heading - self.current_pose[2]
            if heading_error > 3.14159:
                heading_error -= 3.14159
            elif heading_error < -3.14159:
                heading_error += 3.14159

            cmd = Twist()
            # Stop and rotate to heading
            if abs(heading_error) > 0.2 and self.rotate:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = self.surge_vel(dist_to_goal)                

            cmd.angular.z = self.yaw_vel(heading_error)

            print(f'Surge: {cmd.linear.x}, Yaw: {cmd.angular.z}')

        self.vel_pub.publish(cmd)



    def surge_vel(self, dist):
        return self.logistic_vel(dist, 0.6, 0.1, 1.0)

    def yaw_vel(self, dist):
        return self.logistic_vel(dist, 2.0, 0.08, 1.0)


    # Generate velocity demand based on logistic function
    def logistic_vel(self, dist, vmax, zb, d2m):
        tol = 5 # e^-tol
        # Logistic function params
        L = vmax
        k = 10 / d2m
        x0 = tol/k + zb       

        vel = L/(1 + exp(-k * (dist - x0))) + L/(1 + exp(-k * (dist + x0))) - L

        return vel


def main(args=None):
    rclpy.init(args=args)
    nav_to_pose = NavToPose()
    rclpy.spin(nav_to_pose)

    nav_to_pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

