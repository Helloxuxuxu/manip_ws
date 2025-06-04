# launch_example.py

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    """加载YAML配置文件"""
    absolute_path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(absolute_path, "r") as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"[ERROR] Failed to load YAML: {absolute_path}\n{e}")
        return None


def launch_setup(context, *args, **kwargs):
    """基于 context 和 LaunchConfiguration 构建节点和参数"""
    # 读取启动参数（示例：use_fake_hardware）
    use_fake = LaunchConfiguration("use_fake_hardware").perform(context)

    # 配置路径与变量
    robot_name = "rb210"
    pkg_name = "rb210_moveit_config"
    planning_pkg = "rb210_path_planning"

    joint_limits_path = os.path.join(
        get_package_share_directory(pkg_name), "config", "joint_limits.yaml"
    )

    # MoveIt 配置构建
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=pkg_name)
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl"
        )
        .to_moveit_configs()
    )

    # 修改某些参数
    moveit_config.planning_pipelines["ompl"]["planning_group"]["enforce_constrained_state_space"] = True

    move_group_params = {
        "robot_description": moveit_config.robot_description,
        "robot_description_semantic": moveit_config.robot_description_semantic,
        "publish_robot_description_semantic": True,
        "monitor_dynamics": False,
        "use_sim_time": True,
        "joint_limits": joint_limits_path,
        "use_fake_hardware": use_fake.lower() == "true"
    }

    # 创建节点
    path_planning_node = Node(
        package=planning_pkg,
        executable="path_planning_server",
        name="path_planning_node",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_params]
    )

    # 返回所有动作
    return [
        SetParameter(name="use_sim_time", value=True),
        path_planning_node
    ]


def generate_launch_description():
    """Launch 文件入口函数，使用 OpaqueFunction 延迟执行"""
    declared_args = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware for mirroring commands to states."
        )
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
