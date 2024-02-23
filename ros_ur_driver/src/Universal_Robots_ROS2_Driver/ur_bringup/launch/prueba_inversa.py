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
        request.ik_request.pose_stamped = goal_names
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('IK Solution found')
            #joint_positions = future.result().solution.joint_state.position
            #self.execute_joint_trajectory(joint_positions)
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
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator" 
        request.ik_request.pose_stamped = goal_names
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            joint_positions = future.result().solution.joint_state.position
            return joint_positions
        else:
            print("Failed to calculate IK solution")
            return None


def main(args=None):
    rclpy.init(args=args)
    moveit_node = MoveItExampleNode()
    ik_node = IKTransformNode()

    file_path = '/home/alvaro/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/points.csv'
    positions = read_positions_from_file(file_path)

    for position in positions:
        print("Punto leído desde el archivo CSV:", position)
        goal_names = PoseStamped()
        goal_names.header.frame_id = "wrist_3_link"
        goal_names.pose.position.x, goal_names.pose.position.y, goal_names.pose.position.z = position
        goal_names.pose.orientation.w = 1.0

        joint_positions = ik_node.transform_xyz_to_joint_positions(goal_names)
        if joint_positions is not None:
            print("Joint positions:", list(joint_positions))

        moveit_node.send_goal(goal_names)

    rclpy.spin(moveit_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()