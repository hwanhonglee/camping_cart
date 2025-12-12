from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='camping_cart_common',
            executable='robot_info_node',
            name='robot_info_node',
            parameters=[
                {'use_sim_time': False},
                '$(find camping_cart_common)/config/robot_params.yaml'
            ],
            output='screen'
        )
    ])
