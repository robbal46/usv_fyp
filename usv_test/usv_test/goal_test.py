import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose

class GoalTest(Node):

    def __init__(self):
        super().__init__('goal_test')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.declare_parameter('x', 1.0)
        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.declare_parameter('y', 0.0)
        self.y = self.get_parameter('y').get_parameter_value().double_value

        self.send_goal()

    def send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = self.x
        goal.pose.pose.position.y = self.y

        self.client.wait_for_server()

        self.client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    goal_test = GoalTest()
    rclpy.spin_once(goal_test)

if __name__ == '__main__':
    main()