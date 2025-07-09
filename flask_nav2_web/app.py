from flask import Flask, render_template, request, jsonify
import subprocess
import threading

app = Flask(__name__)

goal_active = False
lock = threading.Lock()

@app.route('/')
def index():
    return render_template('index.html')

def run_send_goal(x, y):
    global goal_active
    try:
        subprocess.run(['ros2', 'run', 'nav_goal_sender', 'send_goal.py', str(x), str(y)], check=True)
    except Exception as e:
        print(f"Erro ao executar comando ROS2: {e}")
    finally:
        with lock:
            goal_active = False
        print("Objetivo concluído, pronto para novo objetivo.")

@app.route('/send_goal', methods=['POST'])
def send_goal():
    global goal_active
    with lock:
        if goal_active:
            return jsonify({'status': 'busy', 'message': 'Objetivo em progresso. Aguarde.'}), 409
        goal_active = True

    x = float(request.json['x'])
    y = float(request.json['y'])
    print(f"Recebido objetivo: x={x}, y={y}")

    # Executa envio numa thread para não bloquear o Flask
    thread = threading.Thread(target=run_send_goal, args=(x, y))
    thread.start()

    return jsonify({'status': 'ok', 'x': x, 'y': y})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)





"""from flask import Flask, render_template, request, jsonify
import subprocess

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send_goal', methods=['POST'])
def send_goal():
    x = float(request.json['x'])
    y = float(request.json['y'])
    print(f"Recebido objetivo: x={x}, y={y}")

    try:
        subprocess.run(['ros2', 'run', 'nav_goal_sender', 'send_goal.py', str(x), str(y)], check=True)
    except Exception as e:
        print(f"Erro ao executar comando ROS2: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

    return jsonify({'status': 'ok', 'x': x, 'y': y})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)"""
