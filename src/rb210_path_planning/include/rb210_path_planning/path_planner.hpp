#ifndef SIMPLE_PATH_PLANNING_H
#define SIMPLE_PATH_PLANNING_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_msgs/msg/constraints.h>
#include <moveit_msgs/msg/orientation_constraint.h>
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>

#include <map>
//
#include "rb210_planning_interfaces/action/pick_place.hpp"
#include "rb210_planning_interfaces/srv/pose_req.hpp"
#include "rb210_planning_interfaces/action/move_target.hpp"

#include "path_interpolator.hpp"

using vector6d_t = std::array<double, 6>;
using vector3d_t = std::array<double, 3>;
using MovePlan = moveit::planning_interface::MoveGroupInterface::Plan;
using MovePlans = std::vector<std::pair<std::string, MovePlan>>;
using MovePlansMap = std::map<std::string, MovePlan>;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rb210_path_planning"); // 创建日志记录器

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using MoveTarget = rb210_planning_interfaces::action::MoveTarget;
using GoalHandleMoveTarget = rclcpp_action::ServerGoalHandle<MoveTarget>;

namespace ErrorCode
{
  constexpr int SUCCESS = 1;
  constexpr int FAILURE = -9999; // 通用失败
  constexpr int PLANNING_FAILED = -1;// 规划失败
  constexpr int INVALID_MOTION_PLAN = -2;
  constexpr int START_STATE_IN_COLLISION = -4;
  constexpr int GOAL_IN_COLLISION = -5;
  constexpr int GOAL_CONSTRAINTS_VIOLATED = -6;
  constexpr int INVALID_TARGET_NAME = -10;
  constexpr int EXECUTE_FAILED = -11;
}


class ManipPlanner
{
public:
  explicit ManipPlanner(const rclcpp::NodeOptions &options);
  ~ManipPlanner();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_;

  moveit_visual_tools::MoveItVisualTools moveit_visual_tools_; // 声明为成员变量
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
  
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  const std::string PLANNING_GROUP_ = "planning_group";
  const std::string end_effector_name_ = "hand";
  const double height_to_floor = 0.686;
  std::string stick_id_ = "stick";
  std::vector<double> home_;
  std::vector<double> last_traj_joints_; // 上一规划路径的最后路径点

  // 路径规划
  std::map<std::string, geometry_msgs::msg::Pose> poses_map_; // 运动途径目标点
  MovePlansMap plan_map_; // 运动路径
  MovePlans move_plans_;

public:
  std::map<std::string, geometry_msgs::msg::Pose> getPosesMap(){return poses_map_;}
  MovePlansMap getMovePlansMap(){return plan_map_;}

  // transform监听
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  // TF缓冲区
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;  // TF监听器
  geometry_msgs::msg::TransformStamped Teb_;  //TransformStamped of end effector(gripper centor)
  Eigen::Isometry3d Toc_;                     //Transform of object->camera
  Eigen::Isometry3d Tcg_;                     //Transform of camera->gripper centor
  Eigen::Isometry3d Tgb_;                     //Transform of gripper centor->base

  bool use_fake_hardware_ = true;

  // 内部规划运动接口
  void constructPickPlacePoses(const geometry_msgs::msg::Pose& pick_pose, const geometry_msgs::msg::Pose& place_pose);// 构建抓取放置位姿系列
  int moveP(const std::string target_id);      // 规划至指定位置的api接口
  int moveL(const std::string target_id);  // 笛卡尔方式规划至指定位置的api接口
  int moveJ(const std::string target_id, const std::vector<double> joints);  // 笛卡尔方式规划至指定位置的api接口
  // 先规划后运动的接口
  int planP(const std::string target_id);      // 规划至指定位置的api接口
  int planL(const std::string target_id);  // 笛卡尔方式规划至指定位置的api接口
  int planJ(const std::string target_id, const std::vector<double> joints);  // 笛卡尔方式规划至指定位置的api接口
  int combinePaths(moveit_msgs::msg::RobotTrajectory& traj1,moveit_msgs::msg::RobotTrajectory& traj2);

  // 执行路径接口
  // bool executePlans(const MovePlans& move_plans);
public:
  bool executePlans(MovePlansMap move_plans);
  bool executePlans() {return executePlans(plan_map_);};
  void Next(std::string message = "Press 'next' in the RvizVisualToolsGui window to continue.");

public:
  geometry_msgs::msg::TransformStamped LookupTransform_EE(const std::chrono::milliseconds timeout);
  geometry_msgs::msg::TransformStamped LookupTransform(const std::string& target_frame, const std::string& source_frame, const std::chrono::milliseconds timeout);

  // 外部调用
  int pickAndPlace(std::string stick_id, const geometry_msgs::msg::Pose pick_pose, const geometry_msgs::msg::Pose place_pose);// 规划并执行抓取放置任务
  int pickAndPlace2(std::string stick_id, const geometry_msgs::msg::Pose pick_pose, const geometry_msgs::msg::Pose place_pose);// 规划并执行抓取放置任务
  
  // 路径规划
  bool planToPose(const std::string pose_name, geometry_msgs::msg::Pose pos_target);
  bool planToJoints(const std::string pose_name, const std::vector<double>& joints_target);
  bool planCartPath(const std::string cart_name, const geometry_msgs::msg::Pose& pos_target);
  bool checkMotion(const std::vector<double>& q1,const std::vector<double>& q2);
  bool pathSimplify(moveit_msgs::msg::RobotTrajectory& path,int maxSteps = 0, int maxEmptySteps = 0);
  bool checkCollision(const std::vector<double>& joints_value);   // 检查给定关节下的碰撞
  bool checkPathOrientConstraint(const moveit_msgs::msg::RobotTrajectory& path);
  bool checkOrientConstraint(const std::vector<double>& joints_value);  // 检查规划出的路径是否满足方向约束
  // 轨迹后处理
  void scale_trajectory_speed(moveit_msgs::msg::RobotTrajectory& traj, double scale);
  void draw_all_trajectory();



  // 环境
  bool initPlanningScene();
  bool AddCollisionObject(moveit_msgs::msg::CollisionObject &collision_object);
  bool AddBoxObject(const std::string& id, const std::vector<double>& params);
  bool AddCylinderObject(const std::string& id, const std::vector<double>& params);
  bool AddCylinderByPose(const std::string& id, const geometry_msgs::msg::Pose& pose, double height, double radius);
  bool RemoveCollisionById(const std::string& id);
  bool IsCollisionObjectAdded(const std::string& id);
  bool clearAllCollisionObjects();
  void PrintCollisionObj();
  bool attachObject(const std::string& id);
  bool detatchObject(const std::string& id);
  void printAttach();

  void join(){executor_thread_.join();}
  void spin(){rclcpp::spin(node_);}
  // 提供节点以便 spin
  rclcpp::Node::SharedPtr get_node() const
  {
    return node_;
  }
};

#endif // SIMPLE_PATH_PLANNING_H