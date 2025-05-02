from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():
    ScanRegistration = Node(
        package="lio_livox",
        executable="ScanRegistration",
        parameters=[[FindPackageShare("lio_livox"), "/config", "/mid360.yaml"]],
        output="screen",
    )
    PoseEstimation = Node(
        package="lio_livox",
        executable="PoseEstimation",
        parameters=[[FindPackageShare("lio_livox"), "/config", "/mid360.yaml"]],
        output="screen",
    )

    launch = LaunchDescription()
    launch.add_action(ScanRegistration)
    launch.add_action(PoseEstimation)

    return launch
