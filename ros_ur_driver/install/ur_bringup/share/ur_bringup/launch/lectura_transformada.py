import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import csv

class MoveItExampleNode(Node):
    def __init__(self):
        super().__init__('moveit_example_node')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

    def send_goal(self, goal_names):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator"
        request.ik_request.pose_stamped , goal_names
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('IK Solution found')
            
        else:
            self.get_logger().error('IK Solution failed')
        
def read_positions_from_file(file_path):
    positions = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            positions.append([float(value) for value in row])
    return positions

    
class IKTransformNode(Node):
    def __init__(self):
        super().__init__('ik_transform_node')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

    def transform_xyz_to_joint_positions(self, goal_names):
        #este metodo transforma las coordenadas a posciones articulares a traves del servicon /compute_ik
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator" 
        request.ik_request.pose_stamped = goal_names
        future = self.ik_client.call_async(request)
        #print()
        rclpy.spin_until_future_complete(self, future)
        #print("Dentro de transformacion xyz to joint")
        if future.result() is not None:
            joint_positions= future.result().solution.joint_state.position
            #print("Calculando")
            #print(list(joint_positions))
            return joint_positions
        else:
            print("Failed to calculate IK solution")
            return None

def main(args=None):
    global arm_pub
    rclpy.init(args=args)
    moveit_node = MoveItExampleNode()
    ik_node = IKTransformNode()
    #ik_node es una instancia de la clase IKTransformNode.la tengo que usar para llmar al metodo transform_xyz... MANEJA LA TRANSFORMACION DE COORDENADAS
    node = rclpy.create_node("state_publisher")

    file_path = '/home/adela/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/points.csv'
    positions = read_positions_from_file(file_path)

    for position in positions:
        print("Punto leído desde el archivo CSV:", position)
        goal_names = PoseStamped()
        goal_names.header.frame_id = "base_link"  #para especificar el marco de referencia como el marco del extremo del efector final del robot
        goal_names.pose.position.x, goal_names.pose.position.y, goal_names.pose.position.z = position
        goal_names.pose.orientation.w = 1.0

        # Transforma el punto en coordenadas XYZ a posiciones articulares
        joint_positions = [] #lo defino como list. Se usa en  py para vectores
        #print("Antes de la transformada:, goal_names )
        joint_positions = ik_node.transform_xyz_to_joint_positions(goal_names)
        #print("Antes de la transformada:",joint_positions )
        if joint_positions is not None:
            print("Joint positions:", list(joint_positions)) #para ver el contenido de mi lista

        # Envía la posición articulada al nodo de MoveIt para planificar el movimiento
        moveit_node.send_goal(goal_names)


    rclpy.shutdown()


if __name__ == '__main__':
    main()
