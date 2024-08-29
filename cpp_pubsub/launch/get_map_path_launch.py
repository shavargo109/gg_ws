import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    prefix = '/home/asd/gg_ws/src/cpp_pubsub/'

    # change all params here
    declare_image_path = prefix + 'maps/' + 'map3.png'
    declare_yaml_path = prefix + 'maps/' + 'turtlebot3_world.yaml'
    declare_savepath_filename = prefix + 'path/' + 'map3.csv'
    declare_first_pixels = [160, 194]

    # declare_image_path = prefix+'maps/' + \
    #     'map2.png'
    # declare_yaml_path = prefix+'maps/'+'map.yaml'
    # declare_savepath_filename = prefix+'path/' + \
    #     'asd.csv'
    # declare_first_pixels = [233, 166]

    get_map_path_node = Node(
        package='cpp_pubsub',
        executable='get_map_path.py',
        parameters=[{'image_path': declare_image_path},
                    {'yaml_path': declare_yaml_path},
                    {'savepath_filename': declare_savepath_filename},
                    {'first_pixels': declare_first_pixels}
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
        arguments=['-d', prefix + 'rviz/' + 'test.rviz'],
        parameters=[{
            'fixed_frame': 'map',  # Set the fixed frame
        }]
    )
    ld = LaunchDescription()
    ld.add_action(get_map_path_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)
    return ld
