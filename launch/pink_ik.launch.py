#!/usr/bin/python3

from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    prometheus_path = get_package_share_directory("prometheus")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    run_tests = LaunchConfiguration("run_tests", default="false")

    pink_ik_params = join(prometheus_path, "config", "pink_ik_params.yaml")

    # Load and activate arm controllers
    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "joint_state_broadcaster", "--set-state", "active"],
        output="screen",
    )

    load_jtc = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "joint_trajectory_controller", "--set-state", "active"],
        output="screen",
    )

    pink_ik_node = Node(
        package="prometheus",
        executable="pink_ik_node.py",
        name="pink_ik_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            pink_ik_params,
        ],
    )

    test_poses_node = Node(
        package="prometheus",
        executable="pink_ik_test_poses.py",
        name="pink_ik_test_poses",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(run_tests),
    )

    # Chain: load_jsb -> load_jtc -> pink_ik_node
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("run_tests", default_value="false"),
            load_jsb,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_jsb,
                    on_exit=[load_jtc],
                ),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_jtc,
                    on_exit=[pink_ik_node, test_poses_node],
                ),
            ),
        ]
    )
