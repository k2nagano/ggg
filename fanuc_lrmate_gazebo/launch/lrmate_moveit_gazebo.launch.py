import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("fanuc_lrmate_gazebo")

    robot_xacro = os.path.join(
        pkg_gazebo,
        "urdf",
        "lrmate200id7l_gazebo.urdf.xacro",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_lrmate_gazebo"),
                    "launch",
                    "lrmate_gazebo.launch.py",
                ]
            )
        )
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "lrmate200id7l",
            package_name="fanuc_lrmate_moveit_config",
        )
        .robot_description(file_path=robot_xacro)
        .robot_description_semantic(file_path="config/lrmate200id7l.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("fanuc_moveit_config"),
            "rviz",
            "view_robot.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        arguments=["--display-config", rviz_config],
    )

    delayed_moveit = TimerAction(
        period=6.0,
        actions=[
            move_group_node,
            rviz_node,
        ],
    )

    return LaunchDescription(
        [
            gazebo_launch,
            delayed_moveit,
        ]
    )
