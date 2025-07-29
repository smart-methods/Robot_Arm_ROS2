'''
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arduinobot", package_name="arduinobot_mc").to_moveit_configs()
    return generate_demo_launch(moveit_config)

'''








from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    is_sim = LaunchConfiguration('is_sim')

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='True'
    )

    # Load MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("arduinobot", package_name="arduinobot_mc")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
                #
    )

    # Additional planning parameters (cartesian limits + scaling factors)
    robot_description_planning = {
        'robot_description_planning': {
            'cartesian_limits': {
                'max_trans_vel': 1.5,
                'max_trans_acc': 1.5,
                'max_trans_dec': 1.5,
                'max_rot_vel': 3.14
            },
            'default_velocity_scaling_factor': 0.5,
            'default_acceleration_scaling_factor': 0.5
        }
    }


    # Static transform publisher
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen"
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics
        ],
        output="screen"
    )



    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {'use_sim_time': True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            robot_description_planning

        ]
    )

    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("arduinobot_mc"),
            "config",
            "moveit.rviz"
        ])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            robot_description_planning
        ],
        output="screen"
    )

    return LaunchDescription([
        is_sim_arg,
        static_tf,
        robot_state_publisher,
        move_group_node,
        rviz_node
    ])




'''


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml
from launch.actions import TimerAction
from launch_ros.actions import Node as ROSNode


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    abs_file_path = os.path.join(package_path, file_path)
    with open(abs_file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    is_sim = LaunchConfiguration('is_sim')

    is_sim_arg = DeclareLaunchArgument('is_sim', default_value='True')

    # Load MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("arduinobot", package_name="arduinobot_mc")
        .to_moveit_configs()
    )

    # Load extra config files
    ompl_planning_yaml = load_yaml("arduinobot_mc", "config/ompl_planning.yaml")
    trajectory_execution_yaml = load_yaml("arduinobot_mc", "config/trajectory_execution.yaml")
    moveit_controllers_yaml = load_yaml("arduinobot_mc", "config/moveit_controllers.yaml")

    # Additional robot planning limits
    robot_description_planning = {
        'robot_description_planning': {
            'cartesian_limits': {
                'max_trans_vel': 1.5,
                'max_trans_acc': 1.5,
                'max_trans_dec': 1.5,
                'max_rot_vel': 3.14
            },
            'default_velocity_scaling_factor': 0.5,
            'default_acceleration_scaling_factor': 0.5
        }
    }

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen"
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "initial_joint_positions": {
                    "joint_1": 0.0,
                    "joint_2": 0.0,
                    "joint_3": 0.0,
                    "joint_4": 0.0
                }
            }
        ],
        output="screen"
    )


    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {'use_sim_time': True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            robot_description_planning,
            ompl_planning_yaml,
            trajectory_execution_yaml,
            moveit_controllers_yaml,
            {
                "planning_pipelines": ["ompl"],
                "move_group": {
                    "default_planning_pipeline": "ompl",
                    "planning_pipelines": ["ompl"]
                }
            }
        ]
    )


    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("arduinobot_mc"),
            "config",
            "moveit.rviz"
        ])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            robot_description_planning
        ],
        output="screen"
    )


    time_param_node = ROSNode(
        package="arduinobot_mc",
        executable="time_param_node",
        name="time_param_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ]
    )
    

    return LaunchDescription([
        is_sim_arg,
        static_tf,
        robot_state_publisher,
        move_group_node,
        rviz_node,
        time_param_node
    ])
'''