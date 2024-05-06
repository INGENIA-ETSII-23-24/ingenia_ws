import rclpy
import math
import time
import csv
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import ExecuteTrajectory
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

# Servicio utilizado para generar una trayectoria cartesiana. 
# 1) El Request de este servicio recibe los puntos en coordenadas cartesianas, además de algunos
# parámetros que ayudan a generar una trayectoria cartesiana, como el nombre del robot, datos sobre
# la interpolación de la trayectoria (step, puntos intermedios). Más info, inspeccionando la clase
# 2) La Respuesta del servicio te genera una trayectoria de robot
class CartesianPathNode(Node):
    def __init__(self):
        super().__init__('cartesian_path_node')
        self.compute_cartesian_path_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path')

    def compute_cartesian_path(self, waypoints):
        request = GetCartesianPath.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = "ur_manipulator"
        # request.start_state = ???
        request.waypoints = waypoints
        request.max_step = 0.5
        request.avoid_collisions = True

        while not self.compute_cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperando...')

        future = self.compute_cartesian_path_client.call_async(request)

        self.get_logger().info("Esperando servicio...")
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Servicio completado")

        if future.result() is not None:
            trajectory = future.result().solution
            print(future.result().fraction)
            return trajectory
        else:
            self.get_logger().info("Failed to calculate IK solution")
            return None

# Esta clase se utiliza la acción de MoveIt para ejecutar una trayectoria dada

class MyActionClientNode(Node):
    def __init__(self):
        super().__init__('my_action_client_node')

        self.execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')

    def execute_trajectory(self, trajectory_solution):
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            print("El servidor de la acción '/execute_trajectory' no está disponible")
            return

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory_solution

        future = self.execute_client.send_goal_async(goal_msg)

        print("Esperando resultado de la ejecución...")
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        # print(result)

        # if result:
        #     if result.status == 1:
        #         print("Trayectoria ejecutada exitosamente!")
        #     else:
        #         print("Fallo al ejecutar la trayectoria")
        # else:
        #     print("Fallo al ejecutar la trayectoria")

# Función usada para almacenar en una variable tipo lista los puntos de un archivo .csv

def read_positions_from_file(file_path):
    positions = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            positions.append([float(value) for value in row])
    return positions


def main(args=None):
    rclpy.init(args=args)
    cartesian_path_node = CartesianPathNode()
    action_client_node = MyActionClientNode()

    file_path = '/home/adela/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/20240425_medio_cilindro_poses_quat_en_m.csv'
    positions = read_positions_from_file(file_path)

    print("Iniciando trayectoria...\n")

    goal_names = []

    for position in positions:
        #print("Punto leído desde el archivo CSV:", position)
        poses = Pose()
        #poses.position.x, poses.position.y, poses.position.z, poses.orientation.w, poses.orientation.x, poses.orientation.y, poses.orientation.z = position
        poses.position.x, poses.position.y, poses.position.z, ñe, ñee, ñeeee, ñeeeeeeee = position
        poses.position.x = poses.position.x - 0.5
        poses.position.y = poses.position.y -0.3
        poses.position.z = poses.position.z + 0.5


        poses.orientation.w = 1.0
        goal_names.append(poses)

    trajectory_solution = cartesian_path_node.compute_cartesian_path(
        goal_names)

    # print(trajectory_solution)

    if trajectory_solution:
        print("Trayectoria calculada exitosamente.")
        # print(trajectory_solution)

        # No es necesario implementar la clase de previsualización de la trayectoria, ya se ve
        while True:
            user_input = input(
                "¿Desea continuar con la ejecución de la trayectoria? (si/no): ")
            if user_input.lower() == 'sí' or user_input.lower() == 'si':
                print("Ejecutando trayectoria...")
                action_client_node.execute_trajectory(trajectory_solution)
                break
            elif user_input.lower() == 'no':
                print("Ejecución de trayectoria cancelada.")
                break
            else:
                print("Por favor, ingrese 'sí' o 'no'.")

    else:
        print("Fallo al calcular la trayectoria")

    print("\n ###   FINAL DE TRAYECTORIA   ###\n")
    print("      ###### by BBB #######")
    cartesian_path_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
