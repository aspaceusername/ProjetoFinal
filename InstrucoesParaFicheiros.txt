Instruções:
- Será necessário construir os workspaces dentro de cada pasta, de modo a gerar os ficheiros necessários para correr as simulações, através do comando "colcon build".
- Para o workspace "turtlebot3_ws", o comando que deverá ser utilizado para lançar a simulação é o seguinte: ROS_LOCALHOST_ONLY=1 TURTLEBOT3_MODEL=waffle ros2 launch turtlebot3 simulation.launch.py
- Os ficheiros .sdf utilizam diretorias hard-coded para referenciar os ficheiros .dae, assim, será necessário que o utilizador altere as mesmas.

articubot_ws - Passos necessários para correr o projeto:
- Em todos os terminais, efetuar source:
cd ~/articubot_ws
source install/setup.bash
- Terminal 1:
rviz2 -d src/articubot_one/config/main.rviz
- Terminal 2:
ros2 launch articubot_one launch_sim.launch.py world:=./src/articubot_one/worlds/world_only.sdf
- Terminal 3:
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/articubot_one/config/mapper_params_online_async.yaml use_sim_time:=true
- De seguida, basta correr o comando "python3 app.py" dentro do workspace flask_nav2_web para iniciar o interface e interagir com o veículo através da aplicação flask.
