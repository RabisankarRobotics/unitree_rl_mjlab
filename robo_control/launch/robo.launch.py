import os
import yaml
import shlex
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="./config/yaml",
        description="Path to configuration YAML directory",
    )

    # Load service configuration
    config_path = os.path.join("./config/yaml/services.yaml")
    with open(config_path, "r") as f:
        service_config = yaml.safe_load(f)

    # Extract parameters
    service_names = service_config["service_names"]

    nodes = []

    for name in service_names:
        params = service_config[name]

        package = params["package"]
        executable = params["executable"]
        command_params = params["command_params"]

        # Parse command params into arguments list
        arguments = shlex.split(command_params)

        node = Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            arguments=arguments,
        )

        nodes.append(node)

    return LaunchDescription([config_dir_arg] + nodes)
