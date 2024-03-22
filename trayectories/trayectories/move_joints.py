# Librerías de ros
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # Para manejar datos de ros 2 en formato String
from std_msgs.msg import Float64    # Ídem para datos tipo float de 64 bits.
from sensor_msgs.msg._joint_state import JointState  # Para leer el estado de las articulaciones del UR.
from ur_msgs.msg._io_states import IOStates # Para leer entradas y salidas digitales del UR.
from moveit_msgs.srv import GetPositionIK # Para leer los mensajes de moveit.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped # Para manejar datos de las poses adoptadas en coordenadas articulares.
from moveit_msgs.msg import Constraints, RobotState # Control de restricciones impuestas por el urdf o el entorno de movit.
from moveit_msgs.srv import GetCartesianPath # Algoritmo de cálculo de trayectorias en un espacio vectorial cartesiano.
from rclpy.action import ActionClient # Cliente  para efecturar acciones de ros2.
from moveit_msgs.action import ExecuteTrajectory # Acción de ejecución de trayectorias adaptada de moveit.
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


class IKTransformNode(Node):
    def __init__(self):
        super().__init__('ik_transform_node')
        self.ik_client= self.create_client(GetPositionIK, '/compute_ik')
        self.last_state= RobotState()

    def transform_xyz_to_joint_positions(self, goal_names):
        request= GetPositionIK.Request()
        request.ik_request.group_name= 'ur_manipulator'
        request.ik_request.robot_state= self.last_state
        request.ik_request.pose_stamped= goal_names
        request.ik_request.avoid_collisions= True
        future= self.ik_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            joint_positions= future.result().solution.joint_state.position
            self.last_state= future.result().solution
            return joint_positions
        else:
            print('Se ha fallado en el cálculo de trayectoria IK.')
            return None
        

class publisher_joint_trajectory_controller(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_= self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_tarjectory', 10)
        self.cuenta=0

        self.tiempo_sec= 6
        self.tiempo_nanosec= 0
        self.last_sec= 0
        self.last_nanosec= 0

    def AjustarTiempo(self, punto1, punto2):
        velocidad= 0.08
        distancia= math.sqrt((punto2[0]-punto1[0])**2 + 
                             (punto2[1]-punto1[1])**2 +
                             (punto2[2]-punto2[2])**2)
        tiempo=distancia/velocidad

        if(tiempo>= 0.75):
            self.tiempo_sec=math.ceil(tiempo)
            self.tiempo_nanosec= 0
        else:
            self.tiempo_sec= 0
            self.tiempo_nanosec= int(tiempo*1.3*1e9)
    
    def publish_trajectory(self, all_coord_articulares):
        msg= JointTrajectory()
        msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]  # Lista de nombres de las articulaciones
        
        for i in range(len(all_coord_articulares)):
            point= JointTrajectoryPoint()
            point.positions= all_coord_articulares[i]

            if i != 0:
                self.last_sec+= self.tiempo_sec
                self.last_nanosec+= self.tiempo_nanosec

                if self.last_nanosec>= 1e9:
                    extra_seconds= self.last_nanosec//1e9
                    self.last_sec+= int(extra_seconds)
                    self.last_nanosec%= 1e9
                    self.last_nanosec= int(self.last_nanosec)

                self.AjustarTiempo(all_coord_articulares[i], all_coord_articulares[i-1])

                point.time_from_start= Duration(sec= self.tiempo_sec+self.last_sec, nanosec=self.tiempo_nanosec+self.last_nanosec)
                msg.points.append(point)

            self.publisher_.publish(msg)
            self.get_logger().info('Publicando: %s' %msg)

def read_positions_from_file(file_path):
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            yield [float(value) for value in row]

class TrayectoryNodeJ(Node):
    def __init__(self):
        super().__init__('trajectory_node_joints')

        self.ik_node= IKTransformNode()
        self.joint_trajectory_publisher= publisher_joint_trajectory_controller()

        file_path = './src/Universal_Robots_ROS2_Driver/ur_bringup/config/points.csv'

        self.positions_generator = read_positions_from_file(file_path)
        # print(self.positions_generator)

        print('Iniciando trayectoria ...')
        self.all_joint_positions= []

        self.calculo_trayectoria()

    def calculo_trayectoria(self):

        while True:
            try:
                position = next(self.positions_generator)
            except StopIteration:
                break

            print("Punto leído desde el archivo CSV:", position)

            aux_node= rclpy.create_node('node_aux')
            goal_names = PoseStamped()
            goal_names.header.frame_id = "base_link"
            goal_names.header.stamp = aux_node.get_clock().now().to_msg()
            goal_names.pose.position.x, goal_names.pose.position.y, goal_names.pose.position.z = position

            goal_names.pose.orientation.w = 1.0

            joint_position = self.ik_node.transform_xyz_to_joint_positions(goal_names)
            # print(joint_position)
            # print(goal_names)

            if len(joint_position) == 6:
                self.all_joint_positions.append(joint_position)

        self.joint_trajectory_publisher.publish_trajectory(self.all_joint_positions)
        print("\n ---   TRAYECTORIA EJECUTANDOSE CON ÉXITO   ---\n")
        # self.joint_trajectory_publisher.destroy_node()




    def read_positions_from_file(file_path):
        with open(file_path, 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                yield [float(value) for value in row]




def main(args=None):
    rclpy.init(args=args)
  


    node=TrayectoryNodeJ()

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()


