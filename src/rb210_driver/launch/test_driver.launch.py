# launch_example.py

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # 读取启动参数（示例：use_fake_hardware）
    use_fake = LaunchConfiguration("use_fake_hardware").perform(context)

    # 配置路径与变量
    robot_name = "rb210"
    driver_pkg = "rb210_driver"

    traj_param = PathJoinSubstitution(
        [FindPackageShare(driver_pkg), "config", "joints_config_for_test.yaml"]
    )

    # 创建节点
    rb210_driver_bode = Node(
        package=driver_pkg,
        executable="rb210_driver",
        name="rb210_driver",
        output="screen",
        parameters=[]
    )

    test_driver_node = Node(
        package=driver_pkg,
        executable="test_driver_node",
        name="test_driver_node",
        output="screen",
        parameters=[{"config_path": traj_param}]
    )

    # 返回所有动作
    return [
        SetParameter(name="use_sim_time", value=True),
        rb210_driver_bode,
        test_driver_node
    ]

def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware for mirroring commands to states."
        )
    ]
    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
