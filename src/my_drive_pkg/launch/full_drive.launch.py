"""Launch complete system without joystick packages."""
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch Description."""
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    share_path = get_package_share_path("my_drive_pkg")

    urdf_default_file = str(share_path / "urdf/my_slam_robot.urdf")
    model_arg = DeclareLaunchArgument(
        name="model", default_value=urdf_default_file, description="Absolute path to robot urdf file"
    )

    robot_localization_file = str(share_path / "config/ekf.yaml")

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    # Slam toolbox node
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[str(share_path / "config/mapper_params_online_async.yaml"), {"use_sim_time": use_sim_time}],
    )
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        output="screen",
    )
    # Joint state publisher node
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    # Odom -> base_footprint static transform node
    # odom_base_link_static_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )
    # Robot localization node using an Extended Kalman filter
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[robot_localization_file],
    )
    # Lidar node
    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[
            {
                "serial_port": "/dev/ttyUSB0",
                "frame_id": "laser",
                "angle_compensate": True,
                "scan_mode": "Standard",
                "auto_standby": True,
            }
        ],
    )
    # Connect to joystick, publish /joy messages
    # joystick_node = Node(
    #     name='joy_node',
    #     package='joy',
    #     executable='joy_node',
    #     output='screen',
    #     parameters=[
    #         {
    #             'dev_name': 'wireless_controller',
    #             'deadzone': 0.3,
    #             'autorepeat_rate': 0.0,
    #             'use_sim_time': use_sim_time,
    #         }
    #     ],
    # )
    # Convert /joy messages to /cmd_vel messages and publish
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
    # Move drive wheels per twist msg node
    drive_node = Node(
        name="drive",
        package="my_drive_pkg",
        executable="drive_node",
        parameters=[{"serial_port": "/dev/ttyACM0", "base_width": 0.170, "use_sim_time": use_sim_time}],
        output="screen",
    )
    # Wheels odometry node
    wheels_odom_node = Node(
        package="my_drive_pkg",
        executable="wheels_odom_node",
        name="wheels_odom",
        parameters=[{"odom_frame": "odom", "child_frame": "base_link", "use_sim_time": use_sim_time}],
        output="screen",
    )
    # LCD driver node
    lcd_driver_node = Node(
        package="pet_ros2_lcd_pkg",
        executable="pet_lcd_driver_node",
        name="lcd_display",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    # Initial/goal pose node
    click_2d_node = Node(
        package="my_drive_pkg",
        executable="click_2d_node",
        name="click_2d",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    # ICM-20948 9-dof IMU node
    imu_node = Node(
        package="ros_qwiic_icm_20948",
        executable="ros_qwiic_icm_20948",
        name="ros_qwiic_icm_20948",
        output="screen",
        parameters=[{"topicImu": "/imu/data", "use_sim_time": use_sim_time}],
    )
    # Madgwick filter node
    imu_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="screen",
        parameters=[str(share_path / "config/imu_filter.yaml"), {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            # IncludeLaunchDescription(PythonLaunchDescriptionSource([sllidar_launch_path])),
            rplidar_node,
            model_arg,
            slam_toolbox_node,
            robot_state_publisher_node,
            joint_state_publisher_node,
            # odom_base_link_static_node,
            robot_localization_node,
            wheels_odom_node,
            lcd_driver_node,
            click_2d_node,
            drive_node,
            # joystick_node,
            # teleop_joy_node,
            imu_node,
            imu_madgwick_node,
        ]
    )
