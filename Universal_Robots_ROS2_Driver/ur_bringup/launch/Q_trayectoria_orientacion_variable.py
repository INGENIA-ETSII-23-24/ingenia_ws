import rclpy
import math
import time
import csv
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import ExecuteTrajectory
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as R


class CartesianPathNode(Node):
    def __init__(self):
        super().__init__('cartesian_path_node')
        self.compute_cartesian_path_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path')

        self.tf_buffer = tf2_ros.Buffer()

    def compute_cartesian_path(self, waypoints):
        request = GetCartesianPath.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = "ur_manipulator"
        # request.start_state = ???
        request.waypoints = waypoints
        request.max_step = 0.5
        request.avoid_collisions = True

        if self.tf_buffer.can_transform("tool0", "base_link", rclpy.time.Time()):
            print("Se puede hacer la conversión")
        else:
            print("No se puede")

        while not self.compute_cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperando...')

        future = self.compute_cartesian_path_client.call_async(request)

        self.get_logger().info("Esperando servicio...")
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Servicio completado")

        if future.result() is not None:
            trajectory = future.result().solution
            # print(trajectory.joint_trajectory.points)
            # print(future.result().fraction)
            # speed_scale = 10
            # aux = RobotTrajectory()
            # scaled_points = JointTrajectoryPoint()
            # scaled_points = trajectory.joint_trajectory.points
            # scaled_points.velocities = scaled_points.velocities/speed_scale
            # scaled_points.duration = scaled_points.duration/speed_scale
            # trajectory.joint_trajectory.points = scaled_points

            return trajectory
        else:
            self.get_logger().info("Failed to calculate IK solution")
            return None


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


def read_positions_from_file(file_path):
    positions = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            positions.append([float(value) for value in row])
    return positions


def write_positions_to_file(quaternion_msg, file_path_write):
    with open(file_path_write, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([quaternion_msg.x, quaternion_msg.y,
                        quaternion_msg.z, quaternion_msg.w])


def main(args=None):
    rclpy.init(args=args)
    cartesian_path_node = CartesianPathNode()
    action_client_node = MyActionClientNode()

    file_path = './src/Universal_Robots_ROS2_Driver/ur_bringup/config/points_quaternion.csv'
    positions = read_positions_from_file(file_path)

    print("Iniciando trayectoria...\n")

    goal_names = []

    for position in positions:
        # print("Punto leído desde el archivo CSV:", position)
        poses = Pose()
        poses.position.x, poses.position.y, poses.position.z, poses.orientation.x, poses.orientation.y, poses.orientation.z, poses.orientation.w = position
    

        quaternion_msg = Quaternion()
        quaternion_msg.x = poses.orientation.x
        quaternion_msg.y = poses.orientation.y
        quaternion_msg.z = poses.orientation.z
        quaternion_msg.w = poses.orientation.w


        poses.orientation = quaternion_msg

        goal_names.append(poses)

    trajectory_solution = cartesian_path_node.compute_cartesian_path(
        goal_names)

    # print(trajectory_solution)

    if trajectory_solution:
        print("Trayectoria calculada exitosamente, ejecutando...")
        action_client_node.execute_trajectory(trajectory_solution)
    else:
        print("Fallo al calcular la trayectoria")

    print("\n ###   FINAL DE TRAYECTORIA   ###\n")
    # cartesian_path_node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
