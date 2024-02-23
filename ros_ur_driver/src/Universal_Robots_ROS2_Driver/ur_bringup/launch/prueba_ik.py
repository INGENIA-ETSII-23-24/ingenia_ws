import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.ik_service = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request_ik()

    def request_ik(self):
        request = GetPositionIK.Request()
        # Define el nombre y la posición de las articulaciones
        request.ik_request.group_name = "ur_manipulator"
        request.ik_request.robot_state.joint_state.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        request.ik_request.robot_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Esto establece todas las posiciones en 0

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose.position.x = 0.4
        pose_stamped.pose.position.y = -0.77
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.w = 1.0  # Sin rotación

        request.ik_request.pose_stamped = pose_stamped

        future = self.ik_service.call_async(request)
        future.add_done_callback(self.ik_callback)

    def ik_callback(self, future):
        try:
            response = future.result()
            if response.error_code.val == response.error_code.SUCCESS:
                self.get_logger().info('Solución de IK encontrada:')
                self.get_logger().info(str(response.solution))
            else:
                self.get_logger().info('No se encontró solución de IK')
        except Exception as e:
            self.get_logger().info('La llamada al servicio falló %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    ik_client = IKClient()
    rclpy.spin(ik_client)
    ik_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()