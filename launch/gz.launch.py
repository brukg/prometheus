#!/usr/bin/python3

from launch.substitutions import LaunchConfiguration,PythonExpression
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import AppendEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    namespace = LaunchConfiguration("namespace", default="prometheus")
    headless = LaunchConfiguration("headless", default="true")

    prometheus_pkg = get_package_share_directory("prometheus")
    aws_small_house_pkg = get_package_share_directory("aws_robomaker_small_house_world")
    husarion_gz_worlds_pkg = get_package_share_directory("husarion_gz_worlds")
    # world_file = LaunchConfiguration("world_file", default = join(husarion_gz_worlds_pkg, "worlds", "husarion_world.sdf"))
    world_file = LaunchConfiguration("world_file", default = join(aws_small_house_pkg, "worlds", "small_house.world"))
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression([
                "'", world_file, " -r'",
                " + (' -s' if '", headless, "' == 'true' else '')"
            ])
        }.items()
    )

    spawn_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(prometheus_pkg, "launch", "spawn.launch.py")),
        launch_arguments={
            'namespace': namespace,
        }.items()
    )

    return LaunchDescription([

        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(prometheus_pkg, "worlds")),

        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(prometheus_pkg, "models")),

        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(prometheus_pkg, "materials")),

        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(aws_small_house_pkg, "worlds")),

        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(aws_small_house_pkg, "models")),

        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("world_file", default_value=world_file),
        DeclareLaunchArgument("namespace", default_value=namespace),
        DeclareLaunchArgument("headless", default_value=headless,
            description="Run Gazebo server only (no GUI) if true"),
        
        gz_sim, spawn_node
    ])