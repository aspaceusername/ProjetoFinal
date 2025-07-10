# nav_goal_sender/send_goal.py
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class GoalSender(Node):
    def __init__(self, x, y):
        super().__init__('goal_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_x = x
        self.goal_y = y

        self.timer = self.create_timer(0.5, self.send_goal_once, oneshot=True)

    def send_goal_once(self):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server não disponível!')
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Enviando objetivo para ({self.goal_x}, {self.goal_y})')
        self._client.send_goal_async(goal_msg)
        rclpy.shutdown()

def main():
    if len(sys.argv) != 3:
        print("Uso: ros2 run nav_goal_sender send_goal.py X Y")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])

    rclpy.init()
    node = GoalSender(x, y)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
