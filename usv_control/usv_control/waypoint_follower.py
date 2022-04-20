import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('waypoints', [3.0, 0.0, 3.0, 3.0])
        self.waypoints = self.get_parameter('waypoints').get_parameter_value().double_array_value

        self.pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        self.prog_sub = self.create_subscription(Bool, 'new_goal', self.next_waypoint, 10)

        self.prev_state = False
        self.idx = 0

    # This should really be an action...

    def next_waypoint(self, msg):
        # Can send next goal pose
        if msg.data == True and self.prev_state == False:
            if self.idx < len(self.waypoints):
                pose = PoseStamped()
                pose.pose.position.x = self.waypoints[self.idx]
                pose.pose.position.y = self.waypoints[self.idx+1]

                self.pose_pub.publish(pose)

                self.idx += 2
            
        self.prev_state = msg.data

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)

    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

