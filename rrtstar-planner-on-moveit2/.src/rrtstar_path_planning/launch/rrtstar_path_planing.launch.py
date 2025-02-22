from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import  OpaqueFunction
from launch_ros.actions import Node


def setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="rrtstar_path_planning",
            executable="rrtstar_path_planning",
        )
    )

    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=setup))

    return ld