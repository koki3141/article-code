from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import  OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.moveit import LBRMoveGroupMixin

def setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    model = LaunchConfiguration("model").perform(context)
    use_sim_time = False

    moveit_configs = LBRMoveGroupMixin.moveit_configs_builder(
        robot_name=model,
        package_name=f"{model}_moveit_config",
    )

    ld.add_action(
        Node(
            package="robot_state_server",
            executable="robot_state_server",
            parameters=[
                moveit_configs.to_dict(),
                {"use_sim_time": use_sim_time},
                LBRDescriptionMixin.param_robot_name(),
            ],
        )
    )

    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    # デフォルトで，iiwa7を選択
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_mode())

    ld.add_action(
        IncludeLaunchDescription(
            FindPackageShare("lbr_bringup").find("lbr_bringup")
            + "/launch/mock.launch.py",
            launch_arguments={"model": LaunchConfiguration("model")}.items(),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            FindPackageShare("lbr_bringup").find("lbr_bringup")
            + "/launch/move_group.launch.py",
            launch_arguments={
                "mode": "mock",
                "rviz": "true",
                "model": LaunchConfiguration("model"),
            }.items(),
        )
    )
    # ノードを実行
    ld.add_action(OpaqueFunction(function=setup))

    return ld