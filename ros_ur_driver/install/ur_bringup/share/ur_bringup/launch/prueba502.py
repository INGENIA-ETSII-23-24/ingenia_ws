import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import csv

class MoveItExampleNode(Node):
    def __init__(self):
        super().__init__('joint_trajectory') #moveit_example_node
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

    def send_goal(self, pose_goal):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator"
        request.ik_request.pose_stamped = pose_goal
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('IK Solution found')
            # Use the result to execute the motion, send the trajectory, etc.
        else:
            self.get_logger().error('IK Solution failed')

def read_positions_from_file(file_path):
    positions = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            positions.append([float(value) for value in row])
    return positions

def main(args=None):
    rclpy.init(args=args)
    node = MoveItExampleNode()

    file_path = '/home/alvaro/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/points.csv'
    positions = read_positions_from_file(file_path)

    for position in positions:
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "tool0"
        pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z = position
        pose_goal.pose.orientation.w = 1.0

        node.send_goal(pose_goal)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
