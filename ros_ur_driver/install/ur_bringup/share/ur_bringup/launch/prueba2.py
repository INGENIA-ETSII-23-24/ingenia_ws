import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration  # Importamos el mensaje de Duration
from moveit_msgs.msg import Constraints, PositionIKRequest

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio "/compute_ik" no disponible. Esperando ...')

    def send_goal(self):
        # Configurar la solicitud de cinemática inversa
        req = GetPositionIK.Request()
        req.ik_request.group_name = "manipulator"  # Nombre del grupo de articulaciones
        req.ik_request.pose_stamped.header.frame_id = "base_link"  # Marco de referencia de la pose
        req.ik_request.pose_stamped.pose.position.x = 0.5  # Coordenada x de la pose objetivo
        req.ik_request.pose_stamped.pose.position.y = 0.0  # Coordenada y de la pose objetivo
        req.ik_request.pose_stamped.pose.position.z = 0.5  # Coordenada z de la pose objetivo
        req.ik_request.pose_stamped.pose.orientation.w = 1.0  # Cuaternión de la pose objetivo
        req.ik_request.timeout = Duration(1.0)  # Tiempo de espera máximo para el cálculo de la IK en nanosegundos

        # Enviar la solicitud de cinemática inversa
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.error_code.val == response.error_code.SUCCESS:
                self.get_logger().info('Cinemática inversa calculada exitosamente.')
                joint_positions = response.solution.joint_state.position
                self.get_logger().info('Solución de las articulaciones: %s', joint_positions)
            else:
                self.get_logger().error('Error al calcular la cinemática inversa: %s', response.error_code.val)
        else:
            self.get_logger().error('Error al recibir la respuesta del servicio.')

def main(args=None):
    rclpy.init(args=args)
    ik_client = IKClient()
    ik_client.send_goal()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
