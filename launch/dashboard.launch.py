from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription(
    [
      Node(
        package="ss_depth",
        executable="dashboard_node",
        name="dashboard_node",
        output="screen",
      ),
    ]
  )
