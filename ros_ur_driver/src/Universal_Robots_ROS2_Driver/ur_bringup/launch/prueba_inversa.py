# Importar los módulos necesarios
import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_client_library import urcl # Este módulo reemplaza al ur_robot_driver.urscript

# Crear una clase para el nodo de ROS2
class UR10TrajectoryNode(Node):

    def _init_(self):
        # Inicializar el nodo con el nombre "ur10_trajectory_node"
        super()._init_("ur10_trajectory_node")
        # Crear un publicador para el topic /joint_trajectory_controller/joint_trajectory
        self.publisher = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        # Crear una variable para almacenar el mensaje de tipo JointTrajectory
        self.joint_trajectory = JointTrajectory()
        # Establecer el nombre de las articulaciones del ur10
        self.joint_trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        # Leer los puntos del archivo csv y almacenarlos en una matriz de numpy
        self.points = np.loadtxt("points.csv", delimiter=",")
        # Crear una variable para almacenar el tiempo entre puntos
        self.time_step = 1.0 # segundos
        # Crear una variable para almacenar el tiempo actual
        self.time = 0.0
        # Crear una variable para almacenar el índice del punto actual
        self.index = 0
        # Crear un temporizador para llamar a la función de publicación cada time_step segundos
        self.timer = self.create_timer(self.time_step, self.publish_trajectory)

    def publish_trajectory(self):
        # Comprobar si se han enviado todos los puntos
        if self.index < len(self.points):
            # Obtener el punto actual en forma de pose
            pose = self.points[self.index]
            # Convertir la pose a joint positions usando la función inverse_kinematics de urcl
            joint_positions = urcl.inverse_kinematics(pose, 10, 5) # 10 es el modelo del ur10, 5 es la generación del e-Series
            # Crear un mensaje de tipo JointTrajectoryPoint
            point = JointTrajectoryPoint()
            # Establecer las posiciones de las articulaciones
            point.positions = joint_positions
            # Establecer el tiempo desde el inicio de la trayectoria
            point.time_from_start.sec = int(self.time)
            point.time_from_start.nanosec = int((self.time - point.time_from_start.sec) * 1e9)
            # Añadir el punto a la lista de puntos de la trayectoria
            self.joint_trajectory.points.append(point)
            # Incrementar el tiempo y el índice
            self.time += self.time_step
            self.index += 1
            # Publicar el mensaje de la trayectoria
            self.publisher.publish(self.joint_trajectory)
            # Mostrar un mensaje informativo
            self.get_logger().info(f"Publicando el punto {self.index} de la trayectoria")
        else:
            # Detener el temporizador
            self.timer.cancel()
            # Mostrar un mensaje de finalización
            self.get_logger().info("Trayectoria completada")

# Crear una función principal
def main():
    # Inicializar el contexto de ROS2
    rclpy.init()
    # Crear una instancia del nodo
    node = UR10TrajectoryNode()
    # Ejecutar el nodo hasta que se interrumpa
    rclpy.spin(node)
    # Destruir el nodo
    node.destroy_node()
    # Finalizar el contexto de ROS2
    rclpy.shutdown()

# Ejecutar la función principal si se llama al script
if _name_ == "_main_":
    main()