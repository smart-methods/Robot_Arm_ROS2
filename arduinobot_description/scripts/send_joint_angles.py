#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Time
import serial

TARGET_JOINTS = ['joint_1', 'joint_2', 'joint_3']

class JointProcessor(Node):
    def __init__(self):
        super().__init__('joint_state_to_serial_and_trajectory')

        self.declare_parameter('port', '/dev/ttyACM0')
        port_name = self.get_parameter('port').get_parameter_value().string_value
        self.get_logger().info(f'Using serial port: {port_name}')
        try:
            self.serial_port = serial.Serial(port_name, 115200, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {str(e)}")
            self.serial_port = None

        self.subscription = self.create_subscription(
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

    def joint_state_callback(self, msg: JointState):
        # --- SERIAL SEND ---
        if self.serial_port:
            try:
                angles = [int(a * 180 / 3.14159) for a in msg.position[:4]]
                serial_msg = f"<{','.join(map(str, angles))}>\n"
                self.serial_port.write(serial_msg.encode('utf-8'))
                self.get_logger().info(f'[Serial] Sent: {serial_msg.strip()}')
            except Exception as e:
                self.get_logger().error(f"[Serial] Error: {str(e)}")

        # --- TRAJECTORY PUBLISH ---
        joint_map = dict(zip(msg.name, msg.position))
        if all(j in joint_map for j in TARGET_JOINTS):
            positions = [joint_map[j] for j in TARGET_JOINTS]

            traj_msg = JointTrajectory()
            traj_msg.joint_names = TARGET_JOINTS
            traj_msg.header.stamp = self.get_clock().now().to_msg()

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = 2

            traj_msg.points.append(point)

            self.trajectory_pub.publish(traj_msg)
            self.get_logger().info(f'[Trajectory] Published: {positions}')
        else:
            self.get_logger().warn('[Trajectory] Missing one or more target joints.')

def main(args=None):
    rclpy.init(args=args)
    node = JointProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
