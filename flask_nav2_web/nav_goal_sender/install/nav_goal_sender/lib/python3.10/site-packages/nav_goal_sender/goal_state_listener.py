import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
import requests

class GoalStateListener(Node):
    def __init__(self):
        super().__init__('goal_state_listener')
        # Subscrição ao tópico de status de objetivos (Nav2 publica aqui)
        self.subscriber = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/status',
            self.status_callback,
            10)

        self.flask_server_url = 'http://localhost:5000/goal_done'  # Endpoint Flask para resetar flag

    def status_callback(self, msg: GoalStatusArray):
        # Verifica se há algum objetivo ativo
        if not msg.status_list:
            return

        # Olha para o último estado do objetivo
        status = msg.status_list[-1].status
        # Status 3 = SUCCEEDED (objetivo alcançado)
        if status == 3:
            self.get_logger().info('Objetivo concluído com sucesso!')

            try:
                # Diz ao Flask que objetivo terminou para desbloquear novo envio
                requests.post(self.flask_server_url)
            except Exception as e:
                self.get_logger().error(f'Erro a comunicar com o Flask: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
