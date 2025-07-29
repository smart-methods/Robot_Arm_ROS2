#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import moveit_commander
from moveit_commander.robot_trajectory import RobotTrajectory
from moveit_commander.time_parameterization import IterativeParabolicTimeParameterization

class TrajectoryFixer(Node):
    def __init__(self):
        super().__init__('trajectory_fixer')

        # Initialize MoveIt
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")  # Change to your group name

        self.plan_and_execute()

    def plan_and_execute(self):
        # Plan a motion (named target, or set joint/pose goal)
        self.group.set_named_target("home")  # Or use set_joint_value_target(), set_pose_target()
        success, plan, _, _ = self.group.plan()

        if not success or not plan.joint_trajectory.points:
            self.get_logger().error("Planning failed.")
            return

        # Convert to RobotTrajectory
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = plan.joint_trajectory

        # Apply time parameterization
        iptp = IterativeParabolicTimeParameterization()
        success = iptp.compute_time_stamps(robot_traj)

        if not success:
            self.get_logger().error("Time parameterization failed.")
            return

        # Replace the original plan with the time-parameterized one
        plan.joint_trajectory = robot_traj.joint_trajectory

        # Execute
        self.group.execute(plan, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        self.get_logger().info("Motion executed successfully with fixed timestamps.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
