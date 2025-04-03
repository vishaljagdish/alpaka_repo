from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ml_model",
            executable="ml_model_node",
            output="screen",
            emulate_tty=True,
            prefix="gnome-terminal --"
        )
    ])
