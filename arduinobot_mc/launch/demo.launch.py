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
