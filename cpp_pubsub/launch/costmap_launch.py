import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = True

    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='custom_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': '/home/asd/gg_ws/src/cpp_pubsub/params/my_custom_costmap.yaml',
        }],
        remappings=[
            ('/custom_costmap/costmap', '/custom_costmap/costmap_raw'),
            ('/custom_costmap/costmap_updates',
                '/custom_costmap/costmap_raw_updates'),
        ],
    )
    ld = LaunchDescription()
    ld.add_action(costmap_node)
    # ld.add_action(map_server_node)
    # ld.add_action(lifecycle_manager_node)
    # ld.add_action(rviz_node)
    return ld
