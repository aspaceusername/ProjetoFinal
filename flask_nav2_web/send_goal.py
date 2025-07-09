import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalSender(Node):
    def __init__(self, x, y):
        super().__init__('goal_sender')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.x = x
        self.y = y
        self.timer = self.create_timer(0.5, self.send_goal, oneshot=True)

    def send_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.x
        goal.pose.position.y = self.y
        goal.pose.orientation.w = 1.0

        self.publisher.publish(goal)
        self.get_logger().info(f'Enviado objetivo para ({self.x}, {self.y})')

        rclpy.shutdown()

def main():
    if len(sys.argv) != 3:
        print("Uso: ros2 run nav_goal_sender send_goal.py X Y")
        return

    x, y = float(sys.argv[1]), float(sys.argv[2])
    rclpy.init()
    node = GoalSender(x, y)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
