from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    prefix = LaunchConfiguration("prefix")
    clamp_angle = LaunchConfiguration("clamp_angle")
    tourch_tilt = LaunchConfiguration("tourch_tilt")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("alpaka_welding_gun_description"), "urdf", "welding_gun.urdf.xacro"]),
            " ",
            "name:=",
            "welding_gun",
            " ",
            "prefix:=",
            prefix,
            " ",
            "clamp_angle:=",
            clamp_angle,
            " ", 
            "tourch_tilt:=",
            tourch_tilt,
            " ",            
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("alpaka_welding_gun_description"), "rviz", "view_welding_gun.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "clamp_angle",
            default_value="0",
            description="rotation of the tourch clamp",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tourch_tilt",
            default_value="0",
            description="rotation of the tourch",
        )
    )

    return LaunchDescription(declared_arguments + nodes_to_start)
