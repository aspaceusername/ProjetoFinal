from flask import Flask, render_template, request, jsonify
import subprocess
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

app = Flask(__name__)

robot_pose = {'x': 0.0, 'y': 0.0}

def ros_thread():
    rclpy.init()
    node = Node('pose_listener')

    def pose_callback(msg):
        robot_pose['x'] = msg.pose.pose.position.x
        robot_pose['y'] = msg.pose.pose.position.y

    node.create_subscription(PoseWithCovarianceStamped, '/pose', pose_callback, 10)
    rclpy.spin(node)

threading.Thread(target=ros_thread, daemon=True).start()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send_goal', methods=['POST'])
def send_goal():
    x = float(request.json['x'])
    y = float(request.json['y'])
    print(f"Recebido objetivo: x={x}, y={y}")
    thread = threading.Thread(target=run_send_goal, args=(x, y))
    thread.start()
    return jsonify({'status': 'ok', 'x': x, 'y': y})

def run_send_goal(x, y):
    try:
        subprocess.run(['ros2', 'run', 'nav_goal_sender', 'send_goal.py', str(x), str(y)], check=True)
    except Exception as e:
        print(f"Erro ao executar comando ROS2: {e}")
    print("Objetivo concluído.")

@app.route('/robot_pose')
def get_robot_pose():
    return jsonify(robot_pose)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
