import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.msg import (
    MotionSequenceRequest,
    MotionSequenceItem,
    Constraints,
    RobotState,
    PositionConstraint,
    OrientationConstraint,
)
from moveit_msgs.action import MoveGroupSequence
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from shape_msgs.msg import SolidPrimitive
from typing import List


class MotionSequenceClient(Node):
    def __init__(self):
        super().__init__("motion_sequence_client")
        self._action_client = ActionClient(self, MoveGroupSequence, "/sequence_move_group")

    def create_motion_sequence_request(self):
        # Create the MotionSequenceRequest
        sequence_request = MotionSequenceRequest()

        start_state = RobotState()
        start_state.joint_state.name = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        start_state.joint_state.position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

        # First MotionSequenceItem
        item = MotionSequenceItem()
        item.blend_radius = 0.1
        item.req.group_name = "ur_manipulator"  # Set your planning group
        item.req.planner_id = "PTP"  # Set the planner type
        item.req.allowed_planning_time = 5.0  # Allow 5 seconds for planning
        item.req.max_velocity_scaling_factor = 0.1
        item.req.max_acceleration_scaling_factor = 0.1
        item.req.start_state = start_state

        # Create first target pose (Cartesian goal)
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"  # Set the frame
        target_pose.pose.position = Point(x=0.6, y=-0.3, z=0.5)  # Set position
        target_pose.pose.orientation = Quaternion(
            x=0.0, y=0.7071067811865475, z=0.0, w=0.7071067811865476
        )  # Set orientation

        # Create Constraints and append PositionConstraint and OrientationConstraint
        constraints = Constraints()

        position_constraint = self.create_position_constraint(
            position=[target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z]
        )
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = self.create_orientation_constraint(
            orientation=[
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            ]
        )
        constraints.orientation_constraints.append(orientation_constraint)

        # Append constraints to the MotionSequenceItem goal_constraints
        item.req.goal_constraints.append(constraints)

        # Add item to the sequence request
        sequence_request.items.append(item)

        # --- Add more MotionSequenceItems for the other waypoints ---
        # Last item blend_radius must be 0!
        last_item = MotionSequenceItem()
        last_item.blend_radius = 0.0  # Set the last item blend_radius to 0
        last_item.req.group_name = "ur_manipulator"
        last_item.req.planner_id = "PTP"
        last_item.req.allowed_planning_time = 2.0
        last_item.req.max_velocity_scaling_factor = 0.1
        last_item.req.max_acceleration_scaling_factor = 0.1
        # Create last target pose (Cartesian goal)
        last_target_pose = PoseStamped()
        last_target_pose.header.frame_id = "base_link"
        last_target_pose.pose.position = Point(x=0.6, y=0.3, z=0.0)
        last_target_pose.pose.orientation = Quaternion(x=0.5, y=0.5, z=0.5, w=0.5)

        # Create Constraints and append PositionConstraint and OrientationConstraint for the last item
        last_constraints = Constraints()
        last_position_constraint = self.create_position_constraint(
            position=[
                last_target_pose.pose.position.x,
                last_target_pose.pose.position.y,
                last_target_pose.pose.position.z,
            ]
        )
        last_constraints.position_constraints.append(last_position_constraint)

        last_orientation_constraint = self.create_orientation_constraint(
            orientation=[
                last_target_pose.pose.orientation.x,
                last_target_pose.pose.orientation.y,
                last_target_pose.pose.orientation.z,
                last_target_pose.pose.orientation.w,
            ]
        )
        last_constraints.orientation_constraints.append(last_orientation_constraint)

        # Append the constraints for the last item
        last_item.req.goal_constraints.append(last_constraints)

        sequence_request.items.append(last_item)

        return sequence_request

    def send_motion_sequence_goal(self):
        # Create the motion sequence request
        sequence_request = self.create_motion_sequence_request()

        # Create an action goal and assign the request to it
        goal_msg = MoveGroupSequence.Goal()
        goal_msg.request = sequence_request

        # Configure planning options
        # goal_msg.planning_options.planning_scene_diff.is_diff = False
        # goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = False
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 10

        # Wait for the action server
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        # Send the goal asynchronously
        self.get_logger().info("Sending motion sequence goal...")
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Wait for result and handle response
        rclpy.spin_until_future_complete(self, send_goal_future)

        # Retrieve the result using get_result
        goal_handle = send_goal_future.result()
        print(goal_handle)
        if goal_handle:
            result = goal_handle.get_result()
            if result:
                self.get_logger().info(f"Action result: {result}")
            else:
                self.get_logger().error("Action failed to return result.")
        else:
            self.get_logger().error("Goal was rejected.")

    def feedback_callback(self, feedback_msg):
        # Log feedback from the action server
        self.get_logger().info(f"Feedback: {feedback_msg.feedback.state}")
        if feedback_msg:
            self.get_logger().info(f"Feedback state: {feedback_msg.feedback.state}, message: {feedback_msg.feedback}")

    def create_position_constraint(
        self,
        position: List[float],
        tolerance: float = 0.01,
        weight: float = 1.0,
    ) -> PositionConstraint:
        # Create a new position constraint
        constraint = PositionConstraint()

        # Set the reference frame and target link
        constraint.header.frame_id = "base_link"
        constraint.link_name = "tool0"

        # Define target position
        constraint.constraint_region.primitive_poses.append(Pose())
        constraint.constraint_region.primitive_poses[0].position.x = float(position[0])
        constraint.constraint_region.primitive_poses[0].position.y = float(position[1])
        constraint.constraint_region.primitive_poses[0].position.z = float(position[2])

        # Define the constraint region as a sphere with radius equal to the tolerance
        constraint.constraint_region.primitives.append(SolidPrimitive())
        constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
        constraint.constraint_region.primitives[0].dimensions = [tolerance]

        # Set the weight of the constraint
        constraint.weight = weight

        return constraint

    def create_orientation_constraint(
        self,
        orientation: List[float],
        tolerance: float = 0.1,
        weight: float = 1.0,
    ) -> OrientationConstraint:
        # Create a new orientation constraint
        constraint = OrientationConstraint()

        # Set the reference frame and target link
        constraint.header.frame_id = "base_link"
        constraint.link_name = "tool0"

        # Define target orientation
        constraint.orientation.x = float(orientation[0])
        constraint.orientation.y = float(orientation[1])
        constraint.orientation.z = float(orientation[2])
        constraint.orientation.w = float(orientation[3])

        # Define the constraint region as a sphere with radius equal to the tolerance
        constraint.absolute_x_axis_tolerance = tolerance
        constraint.absolute_y_axis_tolerance = tolerance
        constraint.absolute_z_axis_tolerance = tolerance

        # Set the weight of the constraint
        constraint.weight = weight

        return constraint


def main(args=None):
    rclpy.init(args=args)

    motion_sequence_client = MotionSequenceClient()

    motion_sequence_client.send_motion_sequence_goal()

    motion_sequence_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
