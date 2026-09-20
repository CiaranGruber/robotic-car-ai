"""Start only the policy: shipped settings first, optional overlay second."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def policy_node(context):
    share = Path(get_package_share_directory("ai4r_policy"))
    parameters = [str(share / "config" / "ai4r_policy.yaml")]
    overlay = LaunchConfiguration("params_file").perform(context)
    if overlay:
        if not Path(overlay).is_file():
            raise ValueError("params_file must name a readable policy YAML file")
        parameters.append(overlay)
    return [Node(package="ai4r_policy", executable="policy_node.py", name="ai4r_policy",
                 namespace=LaunchConfiguration("namespace"), parameters=parameters,
                 output="screen", respawn=False)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="", description="Robot ROS namespace"),
        DeclareLaunchArgument("params_file", default_value="", description="Optional policy YAML overlay"),
        OpaqueFunction(function=policy_node),
    ])
