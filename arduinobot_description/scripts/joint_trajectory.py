#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TARGET_JOINTS = ['joint_1', 'joint_2', 'joint_3']

class JointStateToTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_state_to_trajectory')
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.get_logger().info('Listening to /joint_states...')

    def joint_state_callback(self, msg: JointState):
        # Map joint names to positions
        joint_map = dict(zip(msg.name, msg.position))

        # Filter and preserve order of TARGET_JOINTS
        if all(j in joint_map for j in TARGET_JOINTS):
            positions = [joint_map[j] for j in TARGET_JOINTS]

            traj_msg = JointTrajectory()
            traj_msg.joint_names = TARGET_JOINTS

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = 2  # desired execution time

            traj_msg.points.append(point)

            self.trajectory_pub.publish(traj_msg)
            self.get_logger().info(f'Published trajectory: {positions}')
        else:
            self.get_logger().warn('Missing one or more target joints in /joint_states.')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
