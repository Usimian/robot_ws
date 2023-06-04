"""Launch drive system with joystick packages."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Launch Description."""
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    # Publish /joy messages from connected joystick controller
    # joystick_node = Node(
    #     name='joy_node',
    #     package='joy',
    #     executable='joy_node',
    #     output='screen',
    #     parameters=[
    #         {'dev_name': 'wireless_controller', 'deadzone': 0.3, 'autorepeat_rate': 0.0, 'use_sim_time': use_sim_time}
    #     ],
    # )
    # Convert /joy message to /cmd_vel message and publish
    # teleop_joy_node = Node(
    #     name='teleop_twist_joy_node',
    #     package='teleop_twist_joy',
    #     executable='teleop_node',
    #     parameters=[
    #         {
    #             'axis_linear.x': 1,
    #             'axis_angular.yaw': 0,
    #             'scale_linear.x': 1.0,
    #             'scale_angular.yaw': 0.5,
    #             'enable_button': 0,
    #             'require_enable_button': False,
    #             'use_sim_time': use_sim_time,
    #         }
    #     ],
    # )
    # # Wheels odometry node
    wheels_odom_node = Node(
        package='my_drive_pkg',
        executable='wheels_odom_node',
        name='wheels_odom',
        parameters=[{'odom_frame': 'odom', 'child_frame': 'base_link', 'use_sim_time': use_sim_time}],
        output='screen',
    )
    # Lidar node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[
            {
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Boost',
                'auto_standby': True,
            }
        ],
    )
    # Differential drive interface node
    drive_node = Node(
        name='drive',
        package='my_drive_pkg',
        executable='drive_node',
        parameters=[{'serial_port': '/dev/ttyACM0', 'base_width': 0.170, 'use_sim_time': use_sim_time}],
        output='screen',
    )
    # LCD display node
    lcd_driver_node = Node(
        package='pet_ros2_lcd_pkg',
        executable='pet_lcd_driver_node',
        name='lcd_display',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)

    # ld.add_action(joystick_node)
    # ld.add_action(teleop_joy_node)
    ld.add_action(wheels_odom_node)
    ld.add_action(rplidar_node)
    ld.add_action(drive_node)
    ld.add_action(lcd_driver_node)

    return ld
