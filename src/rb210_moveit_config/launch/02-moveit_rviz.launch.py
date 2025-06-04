from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from moveit_configs_utils.launch_utils import add_debuggable_node, DeclareBooleanLaunchArg
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import launch_ros
import re,os,yaml

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    joint_limits_file_path = os.path.join(
            get_package_share_directory("rb210_moveit_config"),
            "config",
            "joint_limits.yaml"
    )
    # MoveIt 配置
    moveit_config = (
    MoveItConfigsBuilder("rb210", package_name="rb210_moveit_config")
    .joint_limits(file_path=joint_limits_file_path)
    .planning_pipelines(
        pipelines=["ompl"],
        default_planning_pipeline="ompl"  # 默认使用 STOMP
    )
    .to_moveit_configs() # 修改这里
    )
    ld = LaunchDescription()
    ld.add_action(launch_ros.actions.SetParameter(name='use_sim_time', value=True))

    other_launch_path = PathJoinSubstitution([
        FindPackageShare('rb210_moveit_config'),  # 替换为被启动Launch文件所在的包名
        'launch',                               # 通常Launch文件存放在`launch`目录下
        '01-start_gazebo.launch.py'           # 替换为被启动的Launch文件名
    ])
    
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([other_launch_path])))

    # 启动 MoveIt 中的 move_group 节点
    my_generate_move_group_launch(ld, moveit_config)
 
    # 启动 RViz 并加载 MoveIt 配置
    my_generate_moveit_rviz_launch(ld, moveit_config)

    
    return ld
 
def my_generate_move_group_launch(ld, moveit_config):
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
 
    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))
    
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    joint_limits_path = os.path.join(
            get_package_share_directory("rb210_moveit_config"),
            "config",
            "joint_limits.yaml"
        )
        # 加载 OMPL 配置文件
    ompl_planning_yaml = os.path.join(
        get_package_share_directory("rb210_moveit_config"),
        "config",
        "ompl_planning.yaml"
    )
    with open(ompl_planning_yaml, 'r') as f:
        ompl_params = yaml.safe_load(f)
    
    move_group_configuration = {
        "robot_description": moveit_config.robot_description,
        "robot_description_semantic": moveit_config.robot_description_semantic,
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),  # 使用 capabilities 参数
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
  
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time": True,
        "joint_limits": joint_limits_path,
        # "ompl": ompl_params  # 直接加载 OMPL 配置
  }
    move_group_params = [moveit_config.to_dict(), move_group_configuration]
 
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=move_group_params,
        output="screen",
        extra_debug_args=["--debug"] if LaunchConfiguration("debug") == "true" else []
    )
 
def my_generate_moveit_rviz_launch(ld, moveit_config):
   
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )
   
    rviz_parameters = [
        # moveit_config.planning_pipelines,
        # moveit_config.robot_description_kinematics,
        {"use_sim_time": True}
    ]
    
    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        arguments=['-d', LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
        output="log"
    )

def generate_rsp_launch(ld, moveit_config):
    """Launch file for robot state publisher (rsp)"""

    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
    )
    ld.add_action(rsp_node)

    return ld