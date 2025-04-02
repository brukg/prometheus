#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
    prometheus_path = get_package_share_directory("prometheus")
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=True)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source")
    robot_desc = Command(
        [
            "xacro ",
            join(prometheus_path, "models", "prometheus", "urdf", "prometheus.xacro"),
            " camera_enabled:=", camera_enabled,
            " stereo_camera_enabled:=", stereo_camera_enabled,
            " two_d_lidar_enabled:=", two_d_lidar_enabled,
            " odometry_source:=", odometry_source,
            " sim_gz:=", "true"
        ]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                    {'robot_description': robot_desc}
                    ],
        remappings=[
            ('/joint_states', 'prometheus/joint_states'),
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "prometheus",
            "-allow_renaming", "true",
            "-z", "0.28",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/world/default/model/prometheus/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        remappings=[
            ('/world/default/model/prometheus/joint_state', 'prometheus/joint_states'),
            ('/odom', 'prometheus/odom'),
            ('/scan', 'prometheus/scan'),
            ('/kinect_camera', 'prometheus/kinect_camera'),
            ('/stereo_camera/left/image_raw', 'prometheus/stereo_camera/left/image_raw'),
            ('/stereo_camera/right/image_raw', 'prometheus/stereo_camera/right/image_raw'),
            ('/imu', 'prometheus/imu'),
            ('/cmd_vel', 'prometheus/cmd_vel'),
            ('kinect_camera/camera_info', 'prometheus/kinect_camera/camera_info'),
            ('stereo_camera/left/camera_info', 'prometheus/stereo_camera/left/camera_info'),
            ('stereo_camera/right/camera_info', 'prometheus/stereo_camera/right/camera_info'),
            ('/kinect_camera/points', 'prometheus/kinect_camera/points'),
        ]
    )

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--yaw", "0.0",
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "kinect_camera",
                    "--child-frame-id", "prometheus/base_footprint/kinect_camera"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value="world"),
        robot_state_publisher,
        gz_spawn_entity, transform_publisher, 
        gz_ros2_bridge
    ])