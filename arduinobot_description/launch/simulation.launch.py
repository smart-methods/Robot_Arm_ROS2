import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction




def generate_launch_description():
    # Paths
    arduinobot_description = get_package_share_directory('arduinobot_description')
    arduinobot_description_share = get_package_prefix('arduinobot_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Launch args
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(arduinobot_description, 'urdf', 'arduinobot.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )

    # Set Gazebo model path
    model_path = os.path.join(arduinobot_description, "models")
    model_path += pathsep + os.path.join(arduinobot_description_share, "share")
    env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)

    # Robot description using xacro
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Gazebo server/client
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        )
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # Spawn robot in Gazebo from robot_description topic
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'arduinobot', '-topic', 'robot_description'],
        output='screen'
    )

    # Controller spawners (Do not include ros2_control_node! Gazebo loads it)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )


    return LaunchDescription([
        model_arg,
        env_var,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])
