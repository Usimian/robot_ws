from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_qwiic_bme280",
                executable="ros_qwiic_bme280",
                name="ros_qwiic_bme280",
                output="screen",
            ),
        ]
    )
