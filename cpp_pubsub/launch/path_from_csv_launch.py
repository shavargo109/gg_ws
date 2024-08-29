import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    prefix = '/home/asd/gg_ws/src/cpp_pubsub/'

    # change all params here

    declare_save_path_filename = prefix+'path/' + '123.csv'
    declare_yaml_path = prefix + 'maps/' + 'map.yaml'
    declare_isNavThroughPoses = bool(True)
    path_from_csv_node = Node(
        package='cpp_pubsub',
        executable='path_from_csv.py',
        parameters=[
            {'save_path_filename': declare_save_path_filename},
            {'isNavThroughPoses': declare_isNavThroughPoses},
        ],
        output='screen',
        emulate_tty=True,  # show print() in terminal
    )
    map_server_node = Node(
        package='nav2_map_server',
        namespace='',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': declare_yaml_path,
        }]
    )
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d'+prefix+'rviz/'+'test.rviz']
    )
    ld = LaunchDescription()
    ld.add_action(path_from_csv_node)
    # ld.add_action(map_server_node)
    # ld.add_action(lifecycle_manager_node)
    # ld.add_action(rviz_node)
    return ld
