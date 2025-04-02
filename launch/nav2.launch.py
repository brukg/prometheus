import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_prometheus = get_package_share_directory('prometheus')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_prometheus, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(pkg_prometheus, 'config', 'small_house.yaml'),
            'params_file': os.path.join(pkg_prometheus, 'config', 'nav2_params.yaml'),
            'package_path': pkg_prometheus, 
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + os.path.join(
                pkg_prometheus,
                'rviz',
                'nav2_default_view.rviz'
            )
        ]
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    remapper_node = Node(
        package='prometheus',
        executable='remapper.py',
        name='remapper',
        output='screen',
    )

    set_initial_pose_node = Node(
        package='prometheus',
        executable='set_initial_pose.py',
        name='set_initial_pose',
        output='screen',
    )

    ld = LaunchDescription()

    # ld.add_action(set_initial_pose_node)

    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    
    # ld.add_action(static_transform_publisher_node)
    # ld.add_action(remapper_node)

    return ld

