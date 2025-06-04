from threading import Thread
from math import pi
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur
from scipy.spatial.transform import Rotation
import numpy as np
from tf_transformations import quaternion_multiply, quaternion_from_euler
from pick_n_place_interfaces.msg import ArucoMarkers, ArucoMarkersArray

class UR5MoveNode(Node):
    def __init__(self):
        super().__init__('ur5_move_node')
        
        self.callback_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            ArucoMarkersArray,
            '/transformed_aruco_pose',
            self.pose_callback,
            10)
        
        self.latest_pose = None
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur.joint_names(),
            base_link_name=ur.base_link_name(),
            end_effector_name=ur.end_effector_name(),
            group_name=ur.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1
        self.moveit2.planner_id = "PTP"
        
        self.executor = rclpy.executors.MultiThreadedExecutor(2)
        self.executor.add_node(self)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        
        self.home_position = [0.0, -pi / 2, pi / 2, 0.0, 0.0, 0.0]
        self.move_to_home()
        
    def move_to_home(self):
        self.get_logger().info("Moving to home position")
        traj = self.moveit2.move_to_configuration(self.home_position)
        if traj:
            self.moveit2.execute(traj)
            success = self.moveit2.wait_until_executed()
            if success:
                self.get_logger().info("Moved to home position")
            else:
                self.get_logger().error("Failed to execute home trajectory")

    def pose_callback(self, msg: ArucoMarkersArray):
        self.latest_pose = msg.aruco_array
        self.get_logger().info(f"Received Pose: {msg.aruco_array}")
        self.execute_pick_and_place(self.latest_pose)

    def execute_pick_and_place(self,pose:ArucoMarkersArray.aruco_array):
        if not pose:
            self.get_logger().error("No pose received yet!")
            return
        
        for i in pose:
            
            original_quat = [
                i.poses.orientation.x,
                i.poses.orientation.y,
                i.poses.orientation.z,
                i.poses.orientation.w]
            
            flip_quat = quaternion_from_euler(np.pi, 0, 0)
            new_quat = quaternion_multiply(original_quat, flip_quat)
            z_rot_quat = quaternion_from_euler(0, 0, np.pi/2)
            final_quat = quaternion_multiply(new_quat, z_rot_quat)
            
            target = Pose(
                position=self.latest_pose.position,
                orientation=Quaternion(*final_quat)
            )
            
            approach_target = Pose(
                position=Point(
                    x=target.position.x,
                    y=target.position.y,
                    z=target.position.z + 0.2
                ),
                orientation=target.orientation
            )
            
            self.move_to_pose(approach_target, "Approach")
            self.move_to_pose(target, "Pick")
            self.move_to_pose(approach_target, "Safe Retreat")
            
            place_target = Pose(position=Point(x=0.3, y=0.3, z=0.018), orientation=target.orientation)
            self.move_to_pose(place_target, "Place")
    
    def move_to_pose(self, pose: Pose, action_name: str):
        self.get_logger().info(f"Moving to {action_name} position")
        traj = self.moveit2.move_to_pose(
            position=[pose.position.x, pose.position.y, pose.position.z],
            quat_xyzw=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            cartesian=True,
            cartesian_max_step=0.0025,
            cartesian_fraction_threshold=0.0)
        
        self.moveit2.execute(traj)
        success = self.moveit2.wait_until_executed()
        if not success:
            self.get_logger().error(f"Failed to execute {action_name} trajectory")
        
    def shutdown(self):
        self.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    rclpy.init()
    node = UR5MoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
        node.shutdown()