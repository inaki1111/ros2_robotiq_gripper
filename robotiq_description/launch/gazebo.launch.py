import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable, IfElseSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    description_file = LaunchConfiguration("description_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", "robot",
            "-allow_renaming", "true",
        ],
    )

    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[" -r -v 1 --physics-engine gz-physics-bullet-featherstone-plugin ", world_file],
                else_value=[" -s -r -v 1 --physics-engine gz-physics-bullet-featherstone-plugin ", world_file],
            )
        }.items(),
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model]",
        ],
        output="screen",
    )

    return [
        robot_state_publisher_node,
        gz_spawn_entity,
        gz_launch_description,
        gz_sim_bridge,
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [get_package_share_directory("robotiq_description"), "urdf", "robotiq_2f_85_gripper.urdf.xacro"]
            ),
            description="Robot description file",
        ),
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Launch Gazebo GUI"
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])