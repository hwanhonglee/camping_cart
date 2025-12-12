# HH_251203: Global Planner Launch File (Refactored for camping_cart_planning)
# 
# Refactored from camping_cart_navigation.
# Now launches global planner with simplified dependencies.
# Integration with camping_cart_map (map subsystem) is TODO.
#
# See: /home/hong/camping_cart_ws/src/camping_cart_planning/README.md

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    planning_pkg_share = get_package_share_directory('camping_cart_planning')
    rviz_config = os.path.join(planning_pkg_share, 'config', 'rviz_planner.rviz')

    # Declare launch arguments
    declare_map_path = DeclareLaunchArgument(
        "map_path",
        default_value="/home/hong/camping_cart_ws/src/lanelet2_map.osm",
        description="Path to Lanelet2 OSM map file"
    )

    declare_origin_lat = DeclareLaunchArgument(
        "origin_lat",
        default_value="36.12491471",
        description="WGS84 reference latitude"
    )

    declare_origin_lon = DeclareLaunchArgument(
        "origin_lon",
        default_value="126.77473954",
        description="WGS84 reference longitude"
    )

    # HH_251203: Global Planner Node (standalone)
    # - Subscribes to /goal_pose (RViz 2D Nav Goal clicks)
    # - Publishes /planning/global_path
    # - Uses lanelet2 routing to compute paths
    planner_node = Node(
        package='camping_cart_planning',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[
            {
                'map_path': LaunchConfiguration('map_path'),
                'origin_lat': LaunchConfiguration('origin_lat'),
                'origin_lon': LaunchConfiguration('origin_lon'),
                'goal_lane_id': -1,
            }
        ]
    )

    # RViz2 for visualization (disabled due to symbol lookup error in environment)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config]
    # )

    return LaunchDescription([
        declare_map_path,
        declare_origin_lat,
        declare_origin_lon,
        planner_node,
        # rviz_node,
    ])
