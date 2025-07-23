import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
import math
import csv

class PlannerMetrics(Node):
    def __init__(self):
        super().__init__('planner_metrics_collector')
        self.navigator = BasicNavigator()

        self.total_distance = 0.0
        self.last_pos = None
        self.min_dist = float('inf')
        self.max_dist = 0.0
        self.collision = False

        # Current pose from camera
        self.current_pose = None

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/gui/camera/pose', self.camera_pose_callback, 10)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_pos:
            dx = x - self.last_pos[0]
            dy = y - self.last_pos[1]
            self.total_distance += math.hypot(dx, dy)
        self.last_pos = (x, y)

    def scan_callback(self, msg):
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                self.min_dist = min(self.min_dist, r)
                self.max_dist = max(self.max_dist, r)
            if r < 0.1:
                self.collision = True

    def camera_pose_callback(self, msg):
        self.current_pose = msg

    def send_goal_and_collect(self, planner_name, goal_pose):
        self.navigator.lifecycleStartup()
        time.sleep(1.0)

        # Use latest camera pose as initial pose, if available
        if self.current_pose is not None:
            self.navigator.setInitialPose(self.current_pose)
        else:
            self.get_logger().warn("No camera pose received yet, using default initial pose")
            # You can either skip or set some default pose here
            # For example, just comment out this line:
            # self.navigator.setInitialPose(PoseStamped())

        time.sleep(1.0)
        self.get_logger().info(f"[Planner: {planner_name}] Sending goal...")

        #self.navigator.changePlanner(planner_name)
        self.navigator.goToPose(goal_pose)

        start_time = time.time()
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        duration = time.time() - start_time

        result = self.navigator.getResult()

        # Save to CSV
        with open('resultados_nav2.csv', mode='a') as f:
            writer = csv.writer(f)
            writer.writerow([
                planner_name,
                duration,
                round(self.total_distance, 3),
                round(self.min_dist, 3),
                round(self.max_dist, 3),
                self.collision
            ])

        self.get_logger().info(f"[Planner: {planner_name}] Finalizado em {duration:.2f}s com {'COLISÃO' if self.collision else 'sucesso'}")

        # Reset
        self.total_distance = 0.0
        self.last_pos = None
        self.min_dist = float('inf')
        self.max_dist = 0.0
        self.collision = False

def main():
    rclpy.init()
    node = PlannerMetrics()

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = 2.0
    goal.pose.position.y = 2.0
    goal.pose.orientation.w = 1.0

    planners = ['GridBased', 'SmacPlanner', 'ThetaStar']
    for planner in planners:
        node.send_goal_and_collect(planner, goal)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
