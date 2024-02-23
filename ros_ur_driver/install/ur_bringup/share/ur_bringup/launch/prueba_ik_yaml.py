import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml
import csv

def read_positions_from_file(file_path):
    positions = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            positions.append([float(value) for value in row])
    return positions

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
            joint_positions = future.result().solution.joint_state.position
            self.write_to_yaml(goal_names, joint_positions)
        else:
            self.get_logger().error('IK Solution failed')

    def write_to_yaml(self, goal_names, joint_positions):
        yaml_data = {
            'publisher_joint_trajectory_position_controller': {
                'ros__parameters': {
                    'controller_name': 'joint_trajectory_controller',
                    'wait_sec_between_publish': 6,
                    'goal_names': ['pos1'],
                    'pos1': list(joint_positions),
                    'joints': [
                        'shoulder_pan_joint',
                        'shoulder_lift_joint',
                        'elbow_joint',
                        'wrist_1_joint',
                        'wrist_2_joint',
                        'wrist_3_joint'
                    ],
                    'check_starting_point': True,
                    'starting_point_limits': {
                        'shoulder_pan_joint': [-0.1, 0.1],
                        'shoulder_lift_joint': [-1.6, -1.5],
                        'elbow_joint': [-0.1, 0.1],
                        'wrist_1_joint': [-1.6, -1.5],
                        'wrist_2_joint': [-0.1, 0.1],
                        'wrist_3_joint': [-0.1, 0.1]
                    }
                }
            }
        }
        with open('/home/alvaro/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/joint_positions.yaml', 'w') as yaml_file:
            yaml.dump(yaml_data, yaml_file, default_flow_style=False)

def main(args=None):
    rclpy.init(args=args)
    moveit_node = MoveItExampleNode()

    file_path = '/home/alvaro/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/points.csv'
    positions = read_positions_from_file(file_path)

    for position in positions:
        goal_names = PoseStamped()
        goal_names.header.frame_id = "base_link"
        goal_names.pose.position.x, goal_names.pose.position.y, goal_names.pose.position.z = position
        goal_names.pose.orientation.w = 1.0

        moveit_node.send_goal(goal_names)

    rclpy.spin(moveit_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
