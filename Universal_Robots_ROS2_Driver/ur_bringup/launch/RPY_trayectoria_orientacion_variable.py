#ESTE PROGRAMA SE REALIZÓ INICIALMENTE PARA CONTROLAR EL UR LEYENDO POSICIONES EN ANGULOS DE EULER.
#FINALMENTE SE DESCARTÓ ESTA OPCIÓN POR ACUERDO CON LOS ENCARGADOS DEL SLICER

#EL UR SOLO ADMITE ORIENTACIONES EN FORMATO DE CUATERIONES POR LO QUE ES NECESARIO REALIZAR UNA TRANSFORMACION DE EULER A CUATERIONES
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PoseStamped
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R   #ESTO ES NECESARIO PARA PODER REALIZAR LA TRANSFORMACION MENCIONADA ANTERIORMENTE

#ESTAS DOS LIBRERIAS SE INVESTIGARON PERO NO SE PUDIERON USAR YA QUE DABAN PROBLEMAS SOBRE LA DEFINICION DE MÉTODO
import tf2_py
import tf2_ros
import csv
import numpy as np




# NODO PARA TRANSFORMAR LAS COORDENADAS DE XYZ A POSICIONES ARTICULARES
class IKTransformNode(Node):
    def __init__(self):
        super().__init__('ik_transform_node')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

    def transform_xyz_to_joint_positions(self, goal_names):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator"
        request.ik_request.pose_stamped = goal_names
        request.ik_request.avoid_collisions = True
        future = self.ik_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            joint_positions = future.result().solution.joint_state.position
            return joint_positions
        else:
            print("Failed to calculate IK solution")
            return None


# NODO PARA PUBLICAR LAS COORDENADAS ARTICULARES
class PublisherJointTrajectoryController(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.cuenta = 0
        self.tiempo_sec = 6
        self.tiempo_nanosec = 0

    def adjust_time(self, point1, point2):
        velocidad = 0.02
        distancia = math.sqrt((point2[0] - point1[0]) ** 2 +
                              (point2[1] - point1[1]) ** 2 +
                              (point2[2] - point1[2]) ** 2)

        tiempo = distancia / velocidad

        if tiempo >= 0.75:
            self.tiempo_sec = math.ceil(tiempo)
            self.tiempo_nanosec = 0
        else:
            self.tiempo_sec = 0
            self.tiempo_nanosec = int(tiempo * 1.3 * 1e9)

    def publish_trajectory(self, coord_articulares):
        msg = JointTrajectory()
        msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        point = JointTrajectoryPoint()
        point.positions = coord_articulares
        point.time_from_start = Duration(sec=self.tiempo_sec, nanosec=self.tiempo_nanosec)
        
        msg.points.append(point)

        self.publisher_.publish(msg)


# NODO PARA ESCUCHAR LAS POSICIONES ARTICULARES DEL ROBOT
class JointStatesListener(Node):
    def __init__(self):
        super().__init__('joint_states_feedback')
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.callback_state, 10)
        self.joint_positions_target = []
        self.current_positions = []
        self.reached_target = False
        self.next_goal = False

    def callback_state(self, msg):
        if self.next_goal:
            tolerance = 0.001
            self.current_positions = msg.position
            last_element = self.current_positions[-1]

            for i in range(len(self.current_positions) - 1, 0, -1):
                self.current_positions[i] = self.current_positions[i - 1]

            self.current_positions[0] = last_element

            if all(abs(curr - target) < tolerance for curr, target in zip(self.current_positions, self.joint_positions_target)):
                self.next_goal = False
                self.reached_target = True


# FUNCIÓN PARA LEER LAS POSICIONES XYZ Y ORIENTACIÓN DESDE UN ARCHIVO .csv
def read_positions_from_file(file_path):
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            yield [float(value) for value in row]


def main(args=None):
    rclpy.init(args=args)
    ik_node = IKTransformNode()
    joint_trajectory_publisher = PublisherJointTrajectoryController()
    joint_states = JointStatesListener()

    tf_buffer = tf2_ros.Buffer()
    node = rclpy.create_node("trayectoria_ingenia")
    tf_listener = tf2_ros.TransformListener(tf_buffer, node=node)  # Se agrega el nodo aquí

    file_path = './src/Universal_Robots_ROS2_Driver/ur_bringup/config/points_orientations.csv'
    positions_generator = read_positions_from_file(file_path)
    last_position = None

    print("Iniciando trayectoria...\n")

    while True:
        try:
            position = next(positions_generator)
        except StopIteration:
            break

        goal_names = PoseStamped()
        goal_names.header.frame_id = "base_link"
        goal_names.header.stamp = node.get_clock().now().to_msg()
        goal_names.pose.position.x, goal_names.pose.position.y, goal_names.pose.position.z, roll, pitch, yaw = position

        # Convertir ángulos de Euler (roll, pitch, yaw) a quaternión
        #quaternion = tf2_py.Quaternion.roll_pitch_yaw_to_quaternion(roll, pitch, yaw)
        r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    
        quaternion = r.as_quat()

        print('position:', position)
        print('roll- pitch-yaw:', roll, pitch, yaw)
        print('quaternion:', quaternion)


        # Después de calcular el cuaternión
        quaternion_msg = Quaternion()
        quaternion_msg.x = quaternion[0]
        quaternion_msg.y = quaternion[1]
        quaternion_msg.z = quaternion[2]
        quaternion_msg.w = quaternion[3]

        # Asignar el cuaternión al campo orientation de PoseStamped
        goal_names.pose.orientation = quaternion_msg

        joint_position = ik_node.transform_xyz_to_joint_positions(goal_names)

        if last_position is not None:
            joint_trajectory_publisher.adjust_time(last_position, position)

        last_position = position

        joint_trajectory_publisher.publish_trajectory(joint_position)
        
        joint_states.joint_positions_target = joint_position
        joint_states.next_goal = True

        while not joint_states.reached_target:
            rclpy.spin_once(joint_states)

        joint_states.reached_target = False
        joint_trajectory_publisher.cuenta += 1

    print("\n ###   FINAL DE TRAYECTORIA   ###\n")
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()
