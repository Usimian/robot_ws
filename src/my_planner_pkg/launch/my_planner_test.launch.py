from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    share_path = get_package_share_path("my_planner_pkg")

    # controller_yaml = str(share_path / "config/controller.yaml")
    # planner_yaml = str(share_path / "config/planner_server.yaml")
    # recovery_yaml = str(share_path / "config/recovery.yaml")
    # bt_navigator_yaml = str(share_path / "config/bt_navigator.yaml")
    # costmap_yaml = str(share_path / "config/costmap.yaml")
    # collision_monitor = str(share_path / "config/collision_monitor.yaml")
    params_yaml = str(share_path / "config/params.yaml")
    map_file_yaml = str(share_path / "maps/my_drive.yaml")

    nav2_controller_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        # respawn=use_respawn,
        # respawn_delay=2.0,
        parameters=[params_yaml],
        remappings=[('cmd_vel', 'cmd_vel_raw')],
    )

    nav2_planner_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_yaml],
    )

    nav2_recoveries_node = Node(
        package="nav2_recoveries",
        executable="recoveries_server",
        name="recoveries_server",
        output="screen",
        parameters=[params_yaml],
    )

    nav2_bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_yaml],
    )

    # nav2_costmap_2d_node = Node(
    #     package="nav2_costmap_2d",
    #     executable="nav2_costmap_2d",
    #     name="nav2_costmap_2d",
    #     output="screen",
    #     parameters=[params_yaml],
    # )

    # map_server_node = Node(
    #     package="nav2_map_server",
    #     executable="map_server",
    #     name="map_server",
    #     output="screen",
    #     parameters=[{"use_sim_time": use_sim_time}, {"yaml_filename": map_file_yaml}],
    # )

    nav2_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_pathplanner",
        output="screen",
        parameters=[
            {"autostart": True},
            {
                "node_names": [
                    "controller_server",
                    "planner_server",
                    # 'nav2_costmap_2d',
                    "recoveries_server",
                    # "map_server",
                    "bt_navigator",
                ]
            },
        ],
    )

    return LaunchDescription(
        [
            nav2_controller_node,
            nav2_planner_node,
            nav2_recoveries_node,
            # nav2_costmap_2d_node,
            # map_server_node,
            nav2_bt_navigator_node,
            nav2_lifecycle_manager_node,
        ]
    )
