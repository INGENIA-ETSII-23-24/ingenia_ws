# Librerías de ros
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # Para manejar datos de ros 2 en formato String
from std_msgs.msg import Float64    # Ídem para datos tipo float de 64 bits.
from sensor_msgs.msg._joint_state import JointState  # Para leer el estado de las articulaciones del UR.
from ur_msgs.msg._io_states import IOStates # Para leer entradas y salidas digitales del UR.
from moveit_msgs.srv import GetPositionIK # Para leer los mensajes de moveit.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory
from builtin_interfaces.msg import Duration


# Otras librerías útiles.
import pandas as pd     # Para manejar datos y crear tablas.
import matplotlib.pyplot as plt # Para hacer gráficos con los datos registrados.
import numpy as np
import re
import time
from tqdm import tqdm
import math 
import csv


class CartesianPathNode(Node):
    def __init__(self):
        super().__init__('cartesian_path_node')
        self.compute_cartesian_path_client= self.create_client(GetCartesianPath, '/compute_cartesian_path')

    def compute_cartesian_path(self, waypoints):
        request= GetCartesianPath.Request()
        request.header.stamp= self.get_clock().now().to_msg()
        request.group_name= 'ur_manipulator'
        request.waypoints= waypoints
        request.max_step= 0.5
        request.avoid_collisions= True

        while not self.compute_cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servico no disponible, esperado ...')
        
        future= self.compute_cartesian_path_client.call_async(request)

        self.get_logger().info('Esperando servicio ...')
        rclpy.spin_until_future_complete(self, future)
        # self.get_logger.info('Servicio completado')
        print('Servicio completado')

        if future.result() is not None:
            trajectory= future.result().solution
            print(future.result().fraction)
            return trajectory
        else:
            self.get_logger().info('Se ha fallado calculando la solución en IK.')
            return None

class MyActionClientNode(Node):
    def __init__(self):
        super().__init__('action_client_node') 

        self.execute_client= ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

    def execute_trajectory(self, trajectory_solution):
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('No está disponible el servidor de la acción /execute_trajectory')
            return
        
        goal_msg=ExecuteTrajectory.Goal()
        goal_msg.trajectory=trajectory_solution

        future=self.execute_client.send_goal_async(goal=goal_msg)

        self.get_logger().info('Esperando resultado de la ejecución')

        rclpy.spin_until_future_complete(self, future)
        result=future.result()

def read_postions_from_file(file_path):
    positions=[]
    with open(file_path, 'r') as file:
        csv_reader=csv.reader(file)
        for row in csv_reader:
            positions.append([float(value) for value in row])
        # print(positions)
        print('Hola estoy leyendo el csv de coordenadas cartesianas.')
        return positions

def main(args=None):
    rclpy.init(args=args)

    cartesian_path_node= CartesianPathNode()
    action_client_node= MyActionClientNode()

    file_path= '/home/alvaro/Desktop/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/points_hel.csv'
    positions= read_postions_from_file(file_path)

    print('Iniciando trayectoria ...\n')

    goal_names=[]

    for position in positions:
        poses=Pose()

        poses.position.x, poses.position.y, poses.position.z = position
        poses.orientation.w=1.0
        goal_names.append(poses)

    trajectory_solution= cartesian_path_node.compute_cartesian_path(goal_names)

    if trajectory_solution:
        print('Se ah calculado la trayectoria con éxito, ejecutando ...')
        action_client_node.execute_trajectory(trajectory_solution)
    else:
        print('Fallo en el cálculo de trayectoria')

    
    print('\n--- FIN DE TRAYECTORIA ---\n')

    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()


if __name__== '__main__':
    main()