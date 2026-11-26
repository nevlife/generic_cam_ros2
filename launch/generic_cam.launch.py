import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def camera_remappings(config_file):
    with open(config_file) as stream:
        parameters = next(iter(yaml.safe_load(stream).values()))["ros__parameters"]
    remappings = []
    for name in parameters["cameras"]:
        for topic in ("image_raw", "camera_info"):
            remappings.append((f"~/output/{name}/{topic}", f"/camera/{name}/{topic}"))
    return remappings


def launch_setup(context):
    share = get_package_share_directory("generic_cam")
    name = LaunchConfiguration("config").perform(context)
    config_file = os.path.join(share, "config", f"{name}.param.yaml")
    node = ComposableNode(
        package="generic_cam",
        plugin="generic_cam::GenericCam",
        name="generic_cam",
        parameters=[config_file],
        remappings=camera_remappings(config_file),
    )
    container = LaunchConfiguration("container").perform(context)
    if container:
        return [
            LoadComposableNodes(target_container=container, composable_node_descriptions=[node])
        ]
    return [
        ComposableNodeContainer(
            name="generic_cam_container",
            namespace="",
            package="capa_ros",
            executable="capa_component_container",
            composable_node_descriptions=[node],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value="generic_cam", description="config/<config>.param.yaml"
            ),
            DeclareLaunchArgument(
                "container", default_value="", description="a running container to load into"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
