/************************************************************************************
 * @file         path_planner.cpp
 * @brief        rb210的控制
 * @author       文娱
 * @date         2025-04-15
 * @version      V0.1
 * @copyright    中煤科工重庆研究院智能化协同创新中心机器人所. 2025 All rights reserved.
 *************************************************************************************
 */

#include "rb210_path_planning/path_planner.hpp"

ManipPlanner::ManipPlanner(const rclcpp::NodeOptions &options)
    : node_(rclcpp::Node::make_shared("path_planning_node", options)),
      move_group_interface_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "planning_group")),
      executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
      moveit_visual_tools_{node_, "arm_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface_->getRobotModel()}
{
  auto declare_parameter = [this](const std::string &name, const auto &default_value, const std::string &description = "")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;

    if (!node_->has_parameter(name))
    {
      node_->declare_parameter(name, default_value, descriptor);
    }
  };

  auto namedHome = move_group_interface_->getNamedTargetValues("home");
  for (const auto &[joint_name, position] : namedHome)
  {
    home_.push_back(position);
  }
  last_traj_joints_ = home_;

  move_group_interface_->setPlannerId("RRTconnect");
  move_group_interface_->setPlanningTime(10);

  // 初始化参数
  if (!node_->has_parameter("use_fake_hardware"))
  {
    node_->declare_parameter("use_fake_hardware", true);
  }
  use_fake_hardware_ = node_->get_parameter("use_fake_hardware").as_bool();
  RCLCPP_INFO(LOGGER, "use_fake_hardware = %s", use_fake_hardware_ ? "true" : "false");

  RCLCPP_INFO(LOGGER, "Planning frame = %s", move_group_interface_->getPlanningFrame().c_str());

  // 设置可视化目标位置
  moveit_visual_tools_.deleteAllMarkers();
  moveit_visual_tools_.loadRemoteControl();

  // 初始化场景
  robot_model_loader_.reset(
      new robot_model_loader::RobotModelLoader(node_, "robot_description"));
  psm_.reset(
      new planning_scene_monitor::PlanningSceneMonitor(node_, robot_model_loader_));
  psm_->startSceneMonitor();
  psm_->startWorldGeometryMonitor();
  psm_->startStateMonitor();
  auto robot_model_ = robot_model_loader_->getModel();
  planning_pipeline_.reset(
      new planning_pipeline::PlanningPipeline(robot_model_, node_, "ompl"));

  // 初始化规划场景
  rclcpp::sleep_for(std::chrono::nanoseconds(500));
  initPlanningScene();

  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  this->transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  executor_->add_node(node_);
  executor_thread_ = std::thread([this]()
                                 { this->executor_->spin(); });
  executor_thread_.detach();
}

ManipPlanner::~ManipPlanner()
{
  RCLCPP_INFO(LOGGER, "Node clear");
}

// 监听机器人末端执行器TF
geometry_msgs::msg::TransformStamped ManipPlanner::LookupTransform_EE(const std::chrono::milliseconds timeout = std::chrono::milliseconds(50))
{
  std::string target_frame = "arm_base_link";
  std::string source_frame = "hand";
  auto max_wait_time = std::chrono::seconds(3); // 设置最大等待时间为3秒
  auto start_time = std::chrono::steady_clock::now();
  bool timed_out = false;
  geometry_msgs::msg::TransformStamped T;
  while (rclcpp::ok() && timed_out == false)
  {
    try
    {
      Teb_ = tf_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(), timeout);
      RCLCPP_INFO(LOGGER, "%s->%s coordinate ----"
                          "x: %f y: %f z: %f qw: %f qx: %f qy: %f qz: %f",
                  target_frame.c_str(), source_frame.c_str(),
                  Teb_.transform.translation.x, Teb_.transform.translation.y, Teb_.transform.translation.z,
                  Teb_.transform.rotation.w, Teb_.transform.rotation.x, Teb_.transform.rotation.y, Teb_.transform.rotation.z);
      return Teb_;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(LOGGER, "%s->%s coordinate listen failed: %s",
                  target_frame.c_str(), source_frame.c_str(), ex.what());
      auto current_time = std::chrono::steady_clock::now();
      if (current_time - start_time > max_wait_time)
      {
        timed_out = true;
        RCLCPP_WARN(LOGGER, "%s->%s coordinate listen failed: %s within the maximum wait time of 3 seconds.",
                    target_frame.c_str(), source_frame.c_str(), ex.what());
      }
    }
  }
  if (timed_out)
  {
    throw std::runtime_error("Transform lookup failed after timeout in MoveIt_Control::LookupTransform_EE.");
  }
  return Teb_;
}
// 监听机器人TF
geometry_msgs::msg::TransformStamped ManipPlanner::LookupTransform(const std::string &target_frame, const std::string &source_frame, const std::chrono::milliseconds timeout = std::chrono::milliseconds(50))
{
  auto max_wait_time = std::chrono::seconds(3); // 设置最大等待时间为3秒
  auto start_time = std::chrono::steady_clock::now();
  geometry_msgs::msg::TransformStamped T;
  bool timed_out = false;
  while (rclcpp::ok() && timed_out == false)
  {
    try
    {
      T = tf_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(), timeout);
      RCLCPP_INFO(LOGGER, "%s->%s coordinate ----"
                          "x: %f y: %f z: %f qw: %f qx: %f qy: %f qz: %f",
                  target_frame.c_str(), source_frame.c_str(),
                  T.transform.translation.x, T.transform.translation.y, T.transform.translation.z,
                  T.transform.rotation.w, T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z);
      return T;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(LOGGER, "%s->%s coordinate listen failed: %s",
                  target_frame.c_str(), source_frame.c_str(), ex.what());
      auto current_time = std::chrono::steady_clock::now();
      if (current_time - start_time > max_wait_time)
      {
        timed_out = true;
        RCLCPP_WARN(LOGGER, "%s->%s coordinate listen failed: %s within the maximum wait time of 3 seconds.",
                    target_frame.c_str(), source_frame.c_str(), ex.what());
      }
    }
  }
  if (timed_out)
  {
    throw std::runtime_error("Transform lookup failed after timeout in MoveIt_Control::LookupTransform.");
  }
  return T;
}

// 构造抓取位姿
void ManipPlanner::constructPickPlacePoses(const geometry_msgs::msg::Pose &pick_pose, const geometry_msgs::msg::Pose &place_pose)
{
  poses_map_.clear();
  geometry_msgs::msg::Pose home_pose;
  home_pose.position.x = 1.2;
  home_pose.position.y = 0;
  home_pose.position.z = 0.65;
  home_pose.orientation.x = -0.7071;
  home_pose.orientation.y = 0;
  home_pose.orientation.z = 0;
  home_pose.orientation.w = 0.7071;
  poses_map_["home"] = home_pose;

  double pick_height = 0.3;

  auto pre_pick = pick_pose;
  auto post_pick = pick_pose;
  pre_pick.position.z = pick_pose.position.z + pick_height;
  post_pick.position.z = pick_pose.position.z + pick_height;

  poses_map_["pre_pick"] = pre_pick;
  poses_map_["pick"] = pick_pose;
  poses_map_["post_pick"] = post_pick;

  auto pre_place = place_pose;
  auto post_place = place_pose;
  pre_place.position.z = place_pose.position.z + pick_height;
  post_place.position.z = place_pose.position.z + pick_height;
  poses_map_["pre_place"] = pre_place;
  poses_map_["place"] = place_pose;
  poses_map_["post_place"] = post_place;
}

// 规划路径到目标，并填充plan_map_
int ManipPlanner::moveP(const std::string target_id)
{
  geometry_msgs::msg::Pose target_pos;
  if (poses_map_.find(target_id) != poses_map_.end())
  {
    target_pos = poses_map_[target_id];
    RCLCPP_INFO(LOGGER, "planning to pose target: %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f.",
                target_pos.position.x, target_pos.position.y, target_pos.position.z,
                target_pos.orientation.x, target_pos.orientation.y, target_pos.orientation.z, target_pos.orientation.w);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "target id is not find in the pose map.");
    return ErrorCode::INVALID_TARGET_NAME;
  }
  bool plan_success = planToPose(target_id, target_pos);
  if (!plan_success)
  {
    RCLCPP_ERROR(LOGGER, "Plan to target %s fail.", target_id.c_str());
    return ErrorCode::PLANNING_FAILED;
  }

  // 执行机器人轨迹的代码
  RCLCPP_INFO(LOGGER, "Executing moveP: %s", target_id.c_str());
  moveit::core::MoveItErrorCode error_id = move_group_interface_->execute(plan_map_[target_id].trajectory_);

  if (error_id != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Execute path fail. ");
    return ErrorCode::EXECUTE_FAILED;
  }
  return ErrorCode::SUCCESS;
}

int ManipPlanner::moveL(const std::string target_id)
{
  geometry_msgs::msg::Pose target_pos;
  if (poses_map_.find(target_id) != poses_map_.end())
  {
    target_pos = poses_map_[target_id];
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "target id is not find in the pose map.");
    return ErrorCode::INVALID_TARGET_NAME;
  }
  // 将当前位置设置为上一规划后位置
  last_traj_joints_ = move_group_interface_->getCurrentJointValues();

  bool plan_success = planCartPath(target_id, target_pos);
  if (!plan_success)
  {
    RCLCPP_ERROR(LOGGER, "Plan to target %s fail.", target_id.c_str());
    return ErrorCode::PLANNING_FAILED;
  }

  // 执行机器人轨迹的代码
  RCLCPP_INFO(LOGGER, "Executing moveL: %s", target_id.c_str());
  moveit::core::MoveItErrorCode error_id = move_group_interface_->execute(plan_map_[target_id].trajectory_);

  if (error_id != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Execute path fail. ");
    return ErrorCode::EXECUTE_FAILED;
  }
  return ErrorCode::SUCCESS;
}

int ManipPlanner::moveJ(const std::string target_id, const std::vector<double> joints)
{
  if (joints.size() != 6)
  {
    RCLCPP_ERROR(LOGGER, "target id is not find in the pose map.");
    return ErrorCode::INVALID_TARGET_NAME;
  }

  bool plan_success = planToJoints(target_id, joints);
  if (!plan_success)
  {
    RCLCPP_ERROR(LOGGER, "Plan to target %s fail.", target_id.c_str());
    return ErrorCode::PLANNING_FAILED;
  }

  // 执行机器人轨迹的代码
  RCLCPP_INFO(LOGGER, "Executing moveJ: %s", target_id.c_str());
  moveit::core::MoveItErrorCode error_id = move_group_interface_->execute(plan_map_[target_id].trajectory_);

  if (error_id != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Execute path fail. ");
    return ErrorCode::EXECUTE_FAILED;
  }
  return ErrorCode::SUCCESS;
}

// 规划路径到目标，并填充plan_map_
int ManipPlanner::planP(const std::string target_id)
{
  geometry_msgs::msg::Pose target_pos;
  if (poses_map_.find(target_id) != poses_map_.end())
  {
    target_pos = poses_map_[target_id];
    RCLCPP_INFO(LOGGER, "planning to pose target: %s, position: %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f.", target_id.c_str(),
                target_pos.position.x, target_pos.position.y, target_pos.position.z,
                target_pos.orientation.x, target_pos.orientation.y, target_pos.orientation.z, target_pos.orientation.w);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "target id is not find in the pose map.");
    return ErrorCode::INVALID_TARGET_NAME;
  }
  bool plan_success = planToPose(target_id, target_pos);
  if (!plan_success)
  {
    RCLCPP_ERROR(LOGGER, "Plan to target %s fail.", target_id.c_str());
    return ErrorCode::PLANNING_FAILED;
  }
  return ErrorCode::SUCCESS;
}

int ManipPlanner::planL(const std::string target_id)
{
  geometry_msgs::msg::Pose target_pos;
  if (poses_map_.find(target_id) != poses_map_.end())
  {
    target_pos = poses_map_[target_id];
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "target id is not find in the pose map.");
    return ErrorCode::INVALID_TARGET_NAME;
  }
  bool plan_success = planCartPath(target_id, target_pos);
  if (!plan_success)
  {
    RCLCPP_ERROR(LOGGER, "Plan to target %s fail.", target_id.c_str());
    return ErrorCode::PLANNING_FAILED;
  }

  return ErrorCode::SUCCESS;
}

int ManipPlanner::planJ(const std::string target_id, const std::vector<double> joints)
{
  if (joints.size() != 6)
  {
    RCLCPP_ERROR(LOGGER, "target id is not find in the pose map.");
    return ErrorCode::INVALID_TARGET_NAME;
  }

  bool plan_success = planToJoints(target_id, joints);
  if (!plan_success)
  {
    RCLCPP_ERROR(LOGGER, "Plan to target %s fail.", target_id.c_str());
    return ErrorCode::PLANNING_FAILED;
  }

  return ErrorCode::SUCCESS;
}

int ManipPlanner::combinePaths(moveit_msgs::msg::RobotTrajectory &traj1, moveit_msgs::msg::RobotTrajectory &traj2)
{
  for (int i = 0; i < 4; i++)
  {
    traj1.joint_trajectory.points.erase(traj1.joint_trajectory.points.end() - 1);
    traj2.joint_trajectory.points.erase(traj2.joint_trajectory.points.begin());
  }

  robot_trajectory::RobotTrajectory rt1(move_group_interface_->getCurrentState()->getRobotModel(), PLANNING_GROUP_);
  rt1.setRobotTrajectoryMsg(*move_group_interface_->getCurrentState(), traj1);

  robot_trajectory::RobotTrajectory rt2(move_group_interface_->getCurrentState()->getRobotModel(), PLANNING_GROUP_);
  rt2.setRobotTrajectoryMsg(*move_group_interface_->getCurrentState(), traj2);

  rt1.append(rt2, /*dt*/ 0.0); // dt 是两段轨迹间的时间间隔，可以设为 0 表示无停顿

  trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
  auto success = time_param.computeTimeStamps(rt1);
  if (!success)
  {
    RCLCPP_ERROR(LOGGER, "time_param computeTimeStamps fails.");
    return 0;
  }

  rt1.getRobotTrajectoryMsg(traj1);
  if (traj1.joint_trajectory.points.empty())
  {
    return 0;
  }
  scale_trajectory_speed(traj1, 2.0);
  return 1;
}

int ManipPlanner::pickAndPlace(std::string stick_id, const geometry_msgs::msg::Pose pick_pose, const geometry_msgs::msg::Pose place_pose)
{
  move_plans_.clear();
  // 环境中添加钻杆
  //  RemoveCollisionById(stick_id);
  AddCylinderByPose(stick_id, pick_pose, 1.0, 0.05);

  constructPickPlacePoses(pick_pose, place_pose);

  if (moveP("pre_pick") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "pre_pick fail.");
    return ErrorCode::EXECUTE_FAILED;
  }
  if (moveL("pick") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "pick fail.");
    return ErrorCode::EXECUTE_FAILED;
  }
  attachObject(stick_id);
  if (moveL("post_pick") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "post_pick fail.");
    return ErrorCode::EXECUTE_FAILED;
  }

  // if(moveJ("home", home_)!=PlanErrorCode::SUCCESS){
  //   RCLCPP_INFO(LOGGER,"home fail.");
  //   return ExecuteErrorCode::EXECUTE_FAILED;
  // }

  if (moveP("pre_place") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "pre_place fail.");
    return ErrorCode::EXECUTE_FAILED;
  }
  if (moveL("place") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "place fail.");
    return ErrorCode::EXECUTE_FAILED;
  }
  detatchObject(stick_id);
  if (moveL("post_place") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "post_place fail.");
    return ErrorCode::EXECUTE_FAILED;
  }

  if (moveJ("home", home_) != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "home fail.");
    return ErrorCode::EXECUTE_FAILED;
  }

  return ErrorCode::SUCCESS;
}

void ManipPlanner::printAttach()
{
  auto planning_scene = psm_->getPlanningScene();
  RCLCPP_INFO(LOGGER, "---Printing attach.---");
  // 输出Object是否存在
  moveit::core::RobotState &current_state = planning_scene->getCurrentStateNonConst();
  current_state.update(); // 更新状态后再检查附着体
  std::vector<const moveit::core::AttachedBody *> attached_bodies;
  current_state.getAttachedBodies(attached_bodies);
  for (auto body : attached_bodies)
  {
    RCLCPP_INFO(LOGGER, "Object [%s] attached.", body->getName().c_str());
  }
}

int ManipPlanner::pickAndPlace2(std::string stick_id, const geometry_msgs::msg::Pose pick_pose, const geometry_msgs::msg::Pose place_pose)
{
  move_plans_.clear();
  plan_map_.clear();
  last_traj_joints_ = home_;
  // 环境中添加钻杆
  stick_id_ = stick_id;
  AddCylinderByPose(stick_id, pick_pose, 1.0, 0.05);

  constructPickPlacePoses(pick_pose, place_pose);

  // 统计时间
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  rclcpp::Time start_time = clock.now();
  rclcpp::Time end_time = clock.now();
  double duration = 0;
  // -------------------TO PICK--------------------------
  if (planP("pre_pick") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "pre_pick fail.");
    return ErrorCode::PLANNING_FAILED;
  }
  duration = (clock.now() - end_time).seconds();
  end_time = clock.now();
  RCLCPP_INFO(LOGGER, "All pre_pick took %.4f seconds", duration);
  if (planL("pick") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "pick fail.");
    return ErrorCode::PLANNING_FAILED;
  }
  duration = (clock.now() - end_time).seconds();
  end_time = clock.now();
  RCLCPP_INFO(LOGGER, "All pick took %.4f seconds", duration);
  if (!attachObject(stick_id))
  {
    RCLCPP_INFO(LOGGER, "attach body fail.");
    return ErrorCode::PLANNING_FAILED;
  }
  printAttach();
  duration = (clock.now() - end_time).seconds();
  end_time = clock.now();
  RCLCPP_INFO(LOGGER, "Attach took %.4f seconds", duration);
  if (planL("post_pick") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "post_pick fail.");
    return ErrorCode::PLANNING_FAILED;
  }
  printAttach();
  duration = (clock.now() - end_time).seconds();
  end_time = clock.now();
  RCLCPP_INFO(LOGGER, "All post_pick took %.4f seconds", duration);

  // --------------------TO PLACE----------------------
  if (planJ("home", home_) != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "home fail.");
    return ErrorCode::PLANNING_FAILED;
  }
  duration = (clock.now() - end_time).seconds();
  end_time = clock.now();
  RCLCPP_INFO(LOGGER, "All home took %.4f seconds", duration);

  if (planP("pre_place") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "pre_place fail.");
    return ErrorCode::PLANNING_FAILED;
  }
  printAttach();
  duration = (clock.now() - end_time).seconds();
  end_time = clock.now();
  RCLCPP_INFO(LOGGER, "All pre_place took %.4f seconds", duration);
  // 连接home和pre_pick
  combinePaths(move_plans_[move_plans_.size() - 2].second.trajectory_, move_plans_[move_plans_.size() - 1].second.trajectory_);
  move_plans_.pop_back();
  plan_map_["pre_place"] = move_plans_.back().second;
  // Next();

  if (planL("place") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "place fail.");
    return ErrorCode::PLANNING_FAILED;
  }
  printAttach();
  duration = (clock.now() - end_time).seconds();
  end_time = clock.now();
  RCLCPP_INFO(LOGGER, "All place took %.4f seconds", duration);
  if (!detatchObject(stick_id))
  {
    RCLCPP_INFO(LOGGER, "detach body fail.");
    return ErrorCode::PLANNING_FAILED;
  }
  printAttach();
  duration = (clock.now() - end_time).seconds();
  end_time = clock.now();
  RCLCPP_INFO(LOGGER, "Detach took %.4f seconds", duration);
  if (planL("post_place") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "post_place fail.");
    return ErrorCode::PLANNING_FAILED;
  }
  duration = (clock.now() - end_time).seconds();
  end_time = clock.now();
  RCLCPP_INFO(LOGGER, "All post_place took %.4f seconds", duration);
  // -------------------TO HOME-----------------------
  if (planJ("home", home_) != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "home fail.");
    return ErrorCode::PLANNING_FAILED;
  }

  duration = (clock.now() - start_time).seconds();
  end_time = clock.now();
  RCLCPP_INFO(LOGGER, "Planning took %.4f seconds", duration);

  // 规划完毕，更新钻杆
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  PrintCollisionObj();
  detatchObject(stick_id);
  RemoveCollisionById(stick_id);
  AddCylinderByPose(stick_id, pick_pose, 1.0, 0.05);

  // // 路径简化
  // for(auto& plan:move_plans_){
  //   pathSimplify(plan.second.trajectory_);
  // }

  // 规划完毕绘制路径
  draw_all_trajectory();

  // 检查路径是否满足垂直约束
  for (auto &plan : move_plans_)
  {
    if (!checkPathOrientConstraint(plan.second.trajectory_))
    {
      return ErrorCode::PLANNING_FAILED;
    }
  }
  RCLCPP_INFO(LOGGER, "Plan process is successful.");
  // 执行前先确认路径
  moveit_visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to execute");
  RCLCPP_INFO(LOGGER, "'next' pressed");
  return ErrorCode::SUCCESS;
}

bool ManipPlanner::planToPose(const std::string pose_name, geometry_msgs::msg::Pose pos_target)
{
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP_);
  if (!move_plans_.empty())
  {
    robot_state->setJointGroupPositions(joint_model_group, last_traj_joints_);
  }
  // 从位姿逆解为关节
  if (!robot_state->setFromIK(joint_model_group, pos_target, 0.1))
  {
    RCLCPP_INFO(LOGGER, "SetFromIK fail");
    return false;
  }
  std::vector<double> joints_target;
  robot_state->copyJointGroupPositions(joint_model_group, joints_target);

  return planToJoints(pose_name, joints_target);
}

bool ManipPlanner::planToJoints(const std::string pose_name, const std::vector<double> &joints_target)
{
  RCLCPP_INFO(LOGGER, "plan name: %s", pose_name.c_str());
  RCLCPP_INFO(LOGGER, "Planning Joints start from:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f.", last_traj_joints_[0], last_traj_joints_[1], last_traj_joints_[2],
              last_traj_joints_[3], last_traj_joints_[4], last_traj_joints_[5]);
  RCLCPP_INFO(LOGGER, "Planning Joints to:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f.", joints_target[0], joints_target[1], joints_target[2],
              joints_target[3], joints_target[4], joints_target[5]);
  auto planning_scene = psm_->getPlanningScene();
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP_);
  if (!move_plans_.empty())
  {
    robot_state->setJointGroupPositions(joint_model_group, last_traj_joints_);
  }
  planning_scene->setCurrentState(*robot_state.get());
  // Now, setup a joint space goal
  moveit::core::RobotState goal_state(*robot_state);
  std::vector<double> joint_values = joints_target;
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::msg::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  planning_interface::MotionPlanRequest req;
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
  req.pipeline_id = "ompl";
  req.planner_id = "RRTConnectkConfigDefault";
  req.group_name = "planning_group";
  req.allowed_planning_time = 1.0;
  req.num_planning_attempts = 25; // 多次尝试不同随机种子
  req.max_velocity_scaling_factor = 1.0;
  req.max_acceleration_scaling_factor = 1.0;
  planning_interface::MotionPlanResponse res;

  // 路径规划
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  rclcpp::Time start_time = clock.now();

  // 循环尝试
  int max_attempts = 1;
  int attempt_num = 0;
  std::vector<planning_interface::MotionPlanResponse> reses;
  std::vector<double> scores;
  while (attempt_num < max_attempts)
  {
    planning_scene_monitor::LockedPlanningSceneRW lscene(psm_);
    // lscene->getCollisionEnvNonConst()->setPadding(0.05);
    planning_pipeline_->generatePlan(lscene, req, res);
    attempt_num++;
    if (res.error_code_.val != res.error_code_.SUCCESS)
    { // 规划失败的情况
      RCLCPP_INFO(LOGGER, "Could not compute plan to %s successfully at attempt %d. ", pose_name.c_str(), attempt_num);
    }
    else
    { // 规划成功的情况
      reses.push_back(res);
      scores.push_back(robot_trajectory::path_length(*res.trajectory_));
      break; // 注释这一行以从多个结果中选取最优
    }
  }
  if (!reses.empty())
  {
    auto best_it = std::min_element(scores.begin(), scores.end());
    res = reses[std::distance(scores.begin(), best_it)];
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Could not compute plan to %s successfully after %d attempts. ", pose_name.c_str(), max_attempts);
    return false;
  }

  rclcpp::Time end_time = clock.now();
  double duration = (end_time - start_time).seconds();
  RCLCPP_INFO(LOGGER, "Planning %s took %.4f seconds", pose_name.c_str(), duration);
  /* Now, call the pipeline and check whether planning was successful. */
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Could not compute plan successfully");
    return false;
  }
  moveit_msgs::msg::MotionPlanResponse response;
  res.getMessage(response);

  // // Visualize the result
  // // ^^^^^^^^^^^^^^^^^^^^
  // rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =
  //     node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
  // moveit_msgs::msg::DisplayTrajectory display_trajectory;
  // display_trajectory.trajectory_start = response.trajectory_start;
  // display_trajectory.trajectory.push_back(response.trajectory);
  // display_publisher->publish(display_trajectory);

  // 时间参数化（必须步骤）
  robot_trajectory::RobotTrajectory rt(
      move_group_interface_->getRobotModel(), "planning_group");

  // 应用时间最优参数化
  moveit::core::RobotState start_state(*move_group_interface_->getCurrentState());
  rt.setRobotTrajectoryMsg(start_state, response.trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
  // 创建时间参数化器
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  // auto success = time_param.computeTimeStamps(rt);
  auto success = iptp.computeTimeStamps(rt);

  if (!success)
  {
    RCLCPP_ERROR(LOGGER, "time_param computeTimeStamps fails.");
    return false;
  }

  rt.getRobotTrajectoryMsg(response.trajectory);

  scale_trajectory_speed(response.trajectory, 2.0);
  // const auto& joint_names = response.trajectory.joint_trajectory.joint_names;
  // const auto& points = response.trajectory.joint_trajectory.points;

  // for (size_t i = 0; i < points.size(); ++i)
  // {
  //   const auto& point = points[i];
  //   RCLCPP_INFO(LOGGER, "Trajectory Point %zu (time_from_start: %.3f s)", i,
  //               point.time_from_start);

  //   for (size_t j = 0; j < point.velocities.size(); ++j)
  //   {
  //     const std::string& joint_name = joint_names[j];
  //     double velocity = point.velocities[j];
  //     RCLCPP_INFO(LOGGER, "  Joint %s: velocity = %.4f", joint_name.c_str(), velocity);
  //   }
  // }
  /* Wait for user input */
  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  plan1.planning_time_ = response.planning_time;
  plan1.start_state_ = response.trajectory_start;
  plan1.trajectory_ = response.trajectory;
  last_traj_joints_ = response.trajectory.joint_trajectory.points.back().positions; // 记录上一次规划的最后轨迹

  plan_map_[pose_name] = plan1;
  move_plans_.push_back({pose_name, plan1});
  return true;
}

// rviz按键到下一步，调试用
void ManipPlanner::Next(std::string message)
{
  /*工具轨迹*/
  auto const draw_trajectory_tool_path = [this](auto const trajectory)
  {
    // jm = move_group_interface_.getRobotModel()->getLinkModel("gripper_fake_center_link");
    // jmg = move_group_interface_.getRobotModel()->getJointModelGroup("ur_manipulator");
    // 清除之前的可视化轨迹
    moveit_visual_tools_.deleteAllMarkers();
    moveit_visual_tools_.publishTrajectoryLine(trajectory, 
    move_group_interface_->getRobotModel()->getLinkModel(end_effector_name_), 
    move_group_interface_->getRobotModel()->getJointModelGroup(PLANNING_GROUP_), 
    rviz_visual_tools::LIME_GREEN); };
  /* 执行规划 */
  draw_trajectory_tool_path(move_plans_.back().second.trajectory_);
  moveit_visual_tools_.trigger();
  moveit_visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to execute");
}

void ManipPlanner::scale_trajectory_speed(moveit_msgs::msg::RobotTrajectory &traj, double scale)
{
  int n_joints = traj.joint_trajectory.joint_names.size(); // 获取关节个数

  for (int i = 0; i < traj.joint_trajectory.points.size(); i++) // 通过for循环对plan中所有的轨迹点作一个遍历
  {
    auto &point = traj.joint_trajectory.points[i];
    // 获取原始时间（秒）
    double original_time = static_cast<double>(point.time_from_start.sec) +
                           static_cast<double>(point.time_from_start.nanosec) / 1e9;
    // 计算缩放后的时间
    double scaled_time = original_time / scale;
    point.time_from_start.sec = static_cast<int32_t>(scaled_time);
    point.time_from_start.nanosec = static_cast<uint32_t>(
        (scaled_time - point.time_from_start.sec) * 1e9);

    for (int j = 0; j < n_joints; j++) // 遍历各个关节，每一个关节的速度和加速度数据都要作一个尺度的变化
    {
      traj.joint_trajectory.points[i].velocities[j] *= scale;            // 速度变化为原来的 scale
      traj.joint_trajectory.points[i].accelerations[j] *= scale * scale; // 加速度变化为原来的 scale * scale
    }
  }
}

void ManipPlanner::draw_all_trajectory()
{
  // 清除之前的可视化轨迹
  moveit_visual_tools_.deleteAllMarkers();
  moveit_msgs::msg::DisplayTrajectory display_trajectory;
  auto robot_state = move_group_interface_->getCurrentState();
  moveit::core::robotStateToRobotStateMsg(*robot_state, display_trajectory.trajectory_start);
  /*工具轨迹*/
  auto const draw_trajectory_tool_path = [this](auto const trajectory)
  { moveit_visual_tools_.publishTrajectoryLine(trajectory,
                                               move_group_interface_->getRobotModel()->getLinkModel(end_effector_name_),
                                               move_group_interface_->getRobotModel()->getJointModelGroup(PLANNING_GROUP_),
                                               rviz_visual_tools::LIME_GREEN); };
  /* 执行规划 */
  for (auto plan : move_plans_)
  {
    display_trajectory.trajectory.push_back(plan.second.trajectory_);
    draw_trajectory_tool_path(plan.second.trajectory_);
  }
  // draw_trajectory_tool_path(move_plans_.back().second.trajectory_);
  moveit_visual_tools_.trigger();
  // moveit_visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to execute");

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =
      node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
  display_publisher->publish(display_trajectory);
}

bool ManipPlanner::planCartPath(const std::string plan_id, const geometry_msgs::msg::Pose &pos_target)
{
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  rclcpp::Time start_time = clock.now();
  // moveit::core::RobotStatePtr start_state(
  //   new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  moveit::core::RobotStatePtr start_state = move_group_interface_->getCurrentState();
  const moveit::core::JointModelGroup *joint_model_group = start_state->getJointModelGroup(PLANNING_GROUP_);
  if (!move_plans_.empty())
  {
    start_state->setJointGroupPositions(joint_model_group, last_traj_joints_);
    move_group_interface_->setStartState(*start_state.get());
  }
  else
  {
    move_group_interface_->setStartStateToCurrentState();
  }

  geometry_msgs::msg::Pose target_pose = pos_target;
  // RCLCPP_INFO(LOGGER,"trying plan to cart target:%s. x:%.3f,y:%.3f,z:%.3f.",plan_id.c_str(), target_pose.position.x,target_pose.position.y,target_pose.position.z);

  // 路径计算
  moveit_msgs::msg::RobotTrajectory trajectory;

  // 使用PILZ进行规划
  move_group_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface_->setPlannerId("LIN");
  move_group_interface_->setPoseTarget(target_pose);

  int planning_attempts = 0;
  const int max_attempts = 3;
  while (planning_attempts < max_attempts)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
    auto const success = static_cast<bool>(move_group_interface_->plan(plan_msg));

    if (success)
    {
      trajectory = plan_msg.trajectory_;
      break; // 成功后跳出循环
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "PILZ for %s, Planning attempt %d  failed.", plan_id.c_str(), planning_attempts);
      planning_attempts++;
      std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 等待
    }
  }
  if (planning_attempts == max_attempts)
  {
    RCLCPP_ERROR(LOGGER, "Planning failed for goal_pos after 3 attempts");
    return false;
  }

  rclcpp::Time end_time = clock.now();
  double duration = (end_time - start_time).seconds();
  RCLCPP_INFO(LOGGER, "planCartPath %s took %.4f seconds", plan_id.c_str(), duration);

  // 添加轨迹到计划中
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  last_traj_joints_ = trajectory.joint_trajectory.points.back().positions; // 记录上一次规划的最后轨迹
  plan_map_[plan_id] = plan;
  move_plans_.push_back({plan_id, plan});
  return true;
}

// 检查q1 q2之间是否存在碰撞
bool ManipPlanner::checkMotion(const std::vector<double> &q1, const std::vector<double> &q2)
{
  interpolate::Path path(q1, q2);
  auto interpolated_path = path.InterpolateWithStepSize(0.02);
  if (interpolated_path.empty())
  {
    RCLCPP_WARN(LOGGER, "interpolate path fail.");
    return false;
  }
  for (int i = 0; i < interpolated_path.size(); i++)
  {
    auto q = interpolated_path[i];
    if (checkCollision(q))
    {
      return false;
    }
  }
  return true;
}

double euclideanDistance(const std::vector<double> &a, const std::vector<double> &b)
{
  if (a.size() != b.size())
    throw std::runtime_error("Dimension mismatch in distance computation");

  double dist = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    dist += (a[i] - b[i]) * (a[i] - b[i]);

  return std::sqrt(dist);
}

bool ManipPlanner::pathSimplify(moveit_msgs::msg::RobotTrajectory &path, int maxSteps, int maxEmptySteps)
{
  auto &path_tmp = path.joint_trajectory.points;
  RCLCPP_INFO(LOGGER, "path size before simplified: %d.", path_tmp.size());
  if (path_tmp.size() < 3)
    return false;
  if (maxSteps == 0)
    maxSteps = path_tmp.size();
  if (maxEmptySteps == 0)
    maxEmptySteps = path_tmp.size();

  // 自定义比较器用于 map 中的 pair<vector<double>, vector<double>> 作为 key
  struct VectorPairLess
  {
    bool operator()(const std::pair<std::vector<double>, std::vector<double>> &a,
                    const std::pair<std::vector<double>, std::vector<double>> &b) const
    {
      return a < b; // 字典序比较
    }
  };
  // 距离缓存表
  std::map<std::pair<std::vector<double>, std::vector<double>>, double, VectorPairLess> distances;
  // 预先计算所有不相邻点对的距离
  for (size_t i = 0; i < path_tmp.size(); ++i)
  {
    for (size_t j = i + 2; j < path_tmp.size(); ++j)
    {
      distances[{path_tmp[i].positions, path_tmp[j].positions}] = euclideanDistance(path_tmp[i].positions, path_tmp[j].positions);
    }
  }
  bool result = false;
  unsigned int nochange = 0;
  for (unsigned int s = 0; s < maxSteps && nochange < maxEmptySteps; ++s, ++nochange)
  {
    double minDist = std::numeric_limits<double>::infinity();
    int p1 = -1, p2 = -1;
    for (size_t i = 0; i < path_tmp.size(); ++i)
    {
      for (size_t j = i + 2; j < path_tmp.size(); ++j)
      {
        double d = distances[{path_tmp[i].positions, path_tmp[j].positions}];
        if (d < minDist)
        {
          minDist = d;
          p1 = i;
          p2 = j;
        }
      }
    }
    if (p1 >= 0 && p2 >= 0)
    {
      if (checkMotion(path_tmp[p1].positions, path_tmp[p2].positions))
      {
        // 删除中间无效点
        path_tmp.erase(path_tmp.begin() + p1 + 1, path_tmp.begin() + p2);
        result = true;
        nochange = 0; // 成功压缩路径，重置 nochange 计数器
      }
      else
      {
        // 将不可直连的节点对标记为不可用
        distances[{path_tmp[p1].positions, path_tmp[p2].positions}] = std::numeric_limits<double>::infinity();
      }
    }
    else
    {
      break; // 没有可简化的路径段，提前结束
    }
  }
  RCLCPP_INFO(LOGGER, "path simplified to size: %d.", path_tmp.size());
  return result;
}

bool ManipPlanner::checkCollision(const std::vector<double> &joints_value)
{
  if (joints_value.size() != 6)
  {
    RCLCPP_WARN(LOGGER, "Dimension Error.");
    return false;
  }
  auto planning_scene = psm_->getPlanningScene();
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP_);
  robot_state->setJointGroupPositions(joint_model_group, joints_value);
  planning_scene->setCurrentState(*robot_state.get());
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000; // 最大碰撞对数量
  collision_request.max_contacts_per_pair = 1;
  planning_scene->checkCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(LOGGER, "Test 1: Current state is " << (collision_result.collision ? "in" : "not in")
                                                         << " collision");
  RCLCPP_INFO(LOGGER, "joints_value:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f.", joints_value[0], joints_value[1], joints_value[2], joints_value[3],
              joints_value[4], joints_value[5]);
  if (collision_result.collision)
  {
    RCLCPP_INFO(LOGGER, "Collision detected.");
    for (const auto &contact_pair : collision_result.contacts)
    {
      const std::string &link1 = contact_pair.first.first;
      const std::string &link2 = contact_pair.first.second;
      RCLCPP_INFO(LOGGER, "Collision between link [%s] and link [%s]", link1.c_str(), link2.c_str());
    }
  }
  return collision_result.collision;
}

bool ManipPlanner::checkPathOrientConstraint(const moveit_msgs::msg::RobotTrajectory &path)
{
  for (auto q : path.joint_trajectory.points)
  {
    if (!checkOrientConstraint(q.positions))
    {
      RCLCPP_INFO(LOGGER, "constraint is not satisfied at this joints.");
      return false;
    }
  }
  return true;
}

bool ManipPlanner::checkOrientConstraint(const std::vector<double> &joints_value)
{
  if (joints_value.size() != 6)
  {
    RCLCPP_WARN(LOGGER, "Dimension Error.");
    return false;
  }
  auto planning_scene = psm_->getPlanningScene();
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP_);
  robot_state->setJointGroupPositions(joint_model_group, joints_value);
  robot_state->update();
  auto end_tf = robot_state->getGlobalLinkTransform(end_effector_name_);
  Eigen::Quaterniond q_curr(end_tf.rotation()); // 四元数
  // RCLCPP_INFO(LOGGER, "End-effector orientation (q_curr): x=%.4f, y=%.4f, z=%.4f, w=%.4f",
  //   q_curr.x(), q_curr.y(), q_curr.z(), q_curr.w());

  Eigen::Quaterniond q_init(0.7071, -0.7071, 0, 0); // 初始姿态（绕z轴90度）
  q_init.normalize();
  // RCLCPP_INFO(LOGGER, "q_init orientation: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
  //   q_init.x(), q_init.y(), q_init.z(), q_init.w());

  // 计算相对旋转 q_rel = q_init.inverse() * q_curr
  Eigen::Quaterniond q_rel = q_init.inverse() * q_curr;
  // RCLCPP_INFO(LOGGER, "q_rel orientation: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
  //   q_rel.x(), q_rel.y(), q_rel.z(), q_rel.w());
  Eigen::Quaterniond q_norm = q_rel.normalized();
  double tollerance = 0.4;
  if ((std::abs(q_norm.x()) < tollerance) && (std::abs(q_norm.z()) < tollerance))
  {
    // RCLCPP_INFO(LOGGER,"State is constrained.");
    return true;
  }
  else
  {
    // RCLCPP_INFO(LOGGER,"State is not constrained. q_norm.x: %.4f,q_norm.y: %.4f.",q_norm.x(),q_norm.y());
    return false;
  }

  return true;
}

// 路径执行
bool ManipPlanner::executePlans(MovePlansMap move_plans)
{
  moveit_msgs::msg::RobotTrajectory current_traj;
  /* 1. To pre_pick */
  RCLCPP_INFO(LOGGER, "Start execute plan: pre_pick.");
  if (move_plans.find("pre_pick") != move_plans.end())
  {
    current_traj = move_plans["pre_pick"].trajectory_;
  }
  else
  {
    return false;
  }
  // 执行路径
  RCLCPP_INFO(LOGGER, "Start executing path.");
  auto success = move_group_interface_->execute(current_traj);
  if (success != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Plan execution fail, Error code is %d.", success);
    return false;
  }

  // // 眼在手上时的物体识别
  // camera_pos_client_->wait_for_service();
  // // 请求目标位姿
  // auto request = std::make_shared<rb210_planning_interfaces::srv::PoseReq::Request>();
  // request->object_id = 0;
  // auto future = camera_pos_client_->async_send_request(request);
  // // 阻塞等待 future 完成（建议设置超时）
  // if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
  // {
  //   auto response = future.get();
  //   auto pos = response->pose;

  //   RCLCPP_INFO(LOGGER, "Received EyeInHand pos: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
  //               pos.position.x, pos.position.y, pos.position.z,
  //               pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w);
  // }
  // else
  // {
  //   RCLCPP_ERROR(LOGGER, "Timeout while waiting for service response.");
  //   return false;
  // }

  /* 2. To pick */
  RCLCPP_INFO(LOGGER, "Start execute plan: pick.");
  if (moveL("pick") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Plan execution fail.");
    return false;
  };

  move_group_interface_->attachObject(stick_id_, move_group_interface_->getEndEffectorLink());
  RCLCPP_INFO(LOGGER, "Start execute plan: post_pick.");
  if (moveL("post_pick") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Plan execution fail.");
    return false;
  };

  /* 3. To pre_place */
  RCLCPP_INFO(LOGGER, "Start execute plan: pre_place.");
  if (move_plans.find("pre_place") != move_plans.end())
  {
    current_traj = move_plans["pre_place"].trajectory_;
  }
  else
  {
    return false;
  }
  // 执行路径
  RCLCPP_INFO(LOGGER, "Start executing path.");
  success = move_group_interface_->execute(current_traj);
  if (success != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Plan execution fail, Error code is %d.", success);
    return false;
  }

  /* 4. To place */
  RCLCPP_INFO(LOGGER, "Start execute plan: place.");
  if (moveL("place") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Plan execution fail.");
    return false;
  };

  move_group_interface_->detachObject(stick_id_);
  RCLCPP_INFO(LOGGER, "Start execute plan: post_place.");
  if (moveL("post_place") != ErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Plan execution fail.");
    return false;
  };

  /* 5. To home */
  RCLCPP_INFO(LOGGER, "Start execute plan: home.");
  if (move_plans.find("home") != move_plans.end())
  {
    current_traj = move_plans["home"].trajectory_;
  }
  else
  {
    return false;
  }
  // 执行路径
  RCLCPP_INFO(LOGGER, "Start executing path.");
  success = move_group_interface_->execute(current_traj);
  if (success != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(LOGGER, "Plan execution fail, Error code is %d.", success);
    return false;
  }

  return true;
}

bool ManipPlanner::initPlanningScene()
{
  RCLCPP_INFO(LOGGER, "planning scene initializing...");
  clearAllCollisionObjects();
  // table
  std::vector<double> params = {0.5, 0.0, -0.011 - height_to_floor, 1.0, 0.0, 0.0, 0.0, 4.0, 5.0, 0.01};
  AddBoxObject("table", params);
  // celling
  std::vector<double> params_celling = {0.5, 0.0, 2.4 - height_to_floor, 1.0, 0.0, 0.0, 0.0, 4.0, 5.0, 0.01};
  AddBoxObject("celling", params_celling);
  // wall1
  std::vector<double> params_wall1 = {-1.0, 0.0, 1.2 - height_to_floor, 0.7071, 0.0, 0.7071, 0.0, 2.4, 5.0, 0.01};
  AddBoxObject("wall1", params_wall1);
  // wall2
  std::vector<double> params_wall2 = {1.8, 0.0, 1.2 - height_to_floor, 0.7071, 0.0, 0.7071, 0.0, 2.4, 5.0, 0.01};
  AddBoxObject("wall2", params_wall2);
  // // wall3
  // std::vector<double> params_wall3 = {0.0, 2.5, 1.15, 0.7071, 0.7071, 0.0, 0.0, 5.0, 2.3, 0.01};
  // AddBoxObject("wall3", params_wall3);
  // wall4
  std::vector<double> params_wall4 = {0.5, -2.5, 1.2 - height_to_floor, 0.7071, 0.7071, 0.0, 0.0, 4.0, 2.4, 0.01};
  AddBoxObject("wall4", params_wall4);

  // stick
  // std::vector<double> params0 = {-0.3, -0.35, 0.5, 0, 0, 0, 1, 0.5, 0.03}; //竖

  // //Beam 横梁1
  // std::vector<double> params1 = {2.0, 0, 0.87, 0.7071068, 0, 0, 0.7071068, 0.51, 0.025};
  // AddCylinderObject("Beam1",params1);
  // //Beam 横梁2
  // std::vector<double> params2 = {2.0, 0, 0.995, 0.7071068, 0, 0, 0.7071068, 0.51, 0.025};
  // AddCylinderObject("Beam2",params2);
  // //Stanchion 支柱1
  // std::vector<double> params3 = {2.0, 0.245, 0.5, 0, 0, 0, 1, 1.0, 0.025};
  // AddCylinderObject("Stanchion1",params3);
  // //Stanchion 支柱2
  // std::vector<double> params4 = {2.0, -0.245, 0.5, 0, 0, 0, 1, 1.0, 0.025};
  // AddCylinderObject("Stanchion2",params4);
  // //Brace 角撑1
  // std::vector<double> params5 = {1.92, 0.245, 0.08, 0, 0.3826834, 0, 0.9238795, 0.2263, 0.02};
  // AddCylinderObject("Brace1",params5);
  // //Brace 角撑2
  // std::vector<double> params6 = {1.92, -0.245, 0.08, 0, 0.3826834, 0, 0.9238795, 0.2263, 0.02};
  // AddCylinderObject("Brace2",params6);
  PrintCollisionObj();

  RCLCPP_INFO(LOGGER, "planning scene initialized.");
  return true;
}

// 打印出场景中的碰撞物体
void ManipPlanner::PrintCollisionObj()
{
  RCLCPP_INFO(LOGGER, "Print Collision Objects");
  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<std::string> object_names;
  RCLCPP_INFO(LOGGER, "Debug Print1");
  object_names = psi.getKnownObjectNames();
  RCLCPP_INFO(LOGGER, "Debug Print2");
  for (const auto &name : object_names)
  {
    RCLCPP_INFO(LOGGER, "Collision object name: %s", name.c_str());
  }
  RCLCPP_INFO(LOGGER, "Debug Print3");
  auto attached_object_names = psi.getAttachedObjects();
  RCLCPP_INFO(LOGGER, "Debug Print4");
  for (auto name : attached_object_names)
  {
    RCLCPP_INFO(LOGGER, "Attach objects name in space:%s", name.first.c_str());
  }
  RCLCPP_INFO(LOGGER, "Debug Print5");
}

// 添加障碍物（通用）
bool ManipPlanner::AddCollisionObject(moveit_msgs::msg::CollisionObject &collision_object)
{
  moveit::planning_interface::PlanningSceneInterface psi;
  RCLCPP_INFO(LOGGER, "Add object %s into the world", collision_object.id.c_str());
  // planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
  // scene->processCollisionObjectMsg(collision_object);
  // psm_->triggerSceneUpdateEvent(
  //   planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  // while(!IsCollisionObjectAdded(collision_object.id)){
  //   psi.addCollisionObjects(collision_objects_);
  // }
  psi.addCollisionObjects({collision_object});
  return true;
}

bool ManipPlanner::AddBoxObject(const std::string &id, const std::vector<double> &params)
{
  RCLCPP_INFO(LOGGER, "Adding box...");
  moveit_msgs::msg::CollisionObject box_object;
  box_object.header.frame_id = move_group_interface_->getPlanningFrame();
  box_object.id = id;
  box_object.primitives.resize(1);  //简单物体只有一个primitive，可用多个primitive组合复杂物体
  box_object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  box_object.primitives[0].dimensions.resize(3);
  box_object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = params[7];
  box_object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = params[8];
  box_object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = params[9];

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = params[0];
  box_pose.position.y = params[1];
  box_pose.position.z = params[2];
  box_pose.orientation.w = params[3];
  box_pose.orientation.x = params[4];
  box_pose.orientation.y = params[5];
  box_pose.orientation.z = params[6];

  box_object.primitive_poses.push_back(box_pose);
  box_object.operation = moveit_msgs::msg::CollisionObject::ADD;

  AddCollisionObject(box_object);
  return true;
}
bool ManipPlanner::AddCylinderObject(const std::string &id, const std::vector<double> &params)
{
  moveit_msgs::msg::CollisionObject cylinder_obj;
  cylinder_obj.id = id;
  cylinder_obj.header.frame_id = move_group_interface_->getPlanningFrame();
  cylinder_obj.primitives.resize(1);
  cylinder_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder_obj.primitives[0].dimensions.resize(2);
  cylinder_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = params[7];
  cylinder_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = params[8];

  geometry_msgs::msg::Pose cylinder_pose;
  cylinder_pose.position.x = params[0];
  cylinder_pose.position.y = params[1];
  cylinder_pose.position.z = params[2];
  cylinder_pose.orientation.x = params[3];
  cylinder_pose.orientation.y = params[4];
  cylinder_pose.orientation.z = params[5];
  cylinder_pose.orientation.w = params[6];

  cylinder_obj.primitive_poses.push_back(cylinder_pose);
  cylinder_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  AddCollisionObject(cylinder_obj);
  return true;
}
bool ManipPlanner::AddCylinderByPose(const std::string &id, const geometry_msgs::msg::Pose &pose, double height, double radius)
{
  moveit_msgs::msg::CollisionObject cylinder_obj;
  cylinder_obj.id = id;
  cylinder_obj.header.frame_id = move_group_interface_->getPlanningFrame();

  cylinder_obj.primitives.resize(1);
  cylinder_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder_obj.primitives[0].dimensions.resize(2);
  cylinder_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = height;
  cylinder_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = radius;

  cylinder_obj.primitive_poses.push_back(pose);
  cylinder_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  // planning_scene_interface_.addCollisionObjects({cylinder_obj});
  // rclcpp::sleep_for(std::chrono::milliseconds(500));  // 简单方式

  AddCollisionObject(cylinder_obj);
  // PrintCollisionObj();
  return true;
}
bool ManipPlanner::RemoveCollisionById(const std::string &id)
{
  RCLCPP_INFO(LOGGER, "Before getAttachedObjects.");
  moveit::planning_interface::PlanningSceneInterface psi;
  // 如果障碍物附着先去除附着
  auto attached_objects = psi.getAttachedObjects();
  std::vector<std::string> attached_ids;
  for (const auto &[ida, obj] : attached_objects)
  {
    if (ida == id)
    {
      RCLCPP_INFO(rclcpp::get_logger("scene"), "Detaching object: %s", id.c_str());
      move_group_interface_->detachObject(id);
    }
  }
  RCLCPP_INFO(LOGGER, "Before removeCollisionObjects.");
  // 从规划场景中移除
  std::vector<std::string> ids_to_remove = {id};
  psi.removeCollisionObjects(ids_to_remove);
  RCLCPP_INFO(LOGGER, "After removeCollisionObjects.");
  // 确认对象已被移除
  while (IsCollisionObjectAdded(id))
  {
    psi.removeCollisionObjects(ids_to_remove);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(LOGGER, "Removed object %s from the planning scene", id.c_str());
  return true;
}

// 判断物体id是否在已知物体中
bool ManipPlanner::IsCollisionObjectAdded(const std::string &id)
{
  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<std::string> object_names = psi.getKnownObjectNames();
  for (const auto &name : object_names)
  {
    if (name == id)
    {
      return true;
    }
  }
  return false;
}

// 初始化时清空障碍物
bool ManipPlanner::clearAllCollisionObjects()
{
  RCLCPP_INFO(LOGGER, "Clear collision.");
  moveit::planning_interface::PlanningSceneInterface psi;
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  PrintCollisionObj();
  // 去除附着
  auto attached_objects = psi.getAttachedObjects();
  std::vector<std::string> attached_ids;
  for (const auto &[id, obj] : attached_objects)
    attached_ids.push_back(id);
  for (const auto &id : attached_ids)
  {
    RCLCPP_INFO(rclcpp::get_logger("scene"), "Detaching object: %s", id.c_str());
    move_group_interface_->detachObject(id);
  }
  // 获取所有已知的碰撞对象的ID列表
  int MAX_ATTEMPTS = 10;
  std::vector<std::string> known_object_ids;
  known_object_ids = psi.getKnownObjectNames();

  // move_group_interface_.detachObject("stick");
  // 移除所有已知的碰撞对象
  psi.removeCollisionObjects(known_object_ids);
  // 确认所有对象已被移除
  int attempts = 0;
  while (known_object_ids.size() > 0 && attempts < MAX_ATTEMPTS)
  {
    psi.removeCollisionObjects(known_object_ids);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    known_object_ids = psi.getKnownObjectNames();
    attempts++;
  }

  if (attempts >= MAX_ATTEMPTS)
  {
    RCLCPP_ERROR(LOGGER, "Failed to clear all objects after %d attempts.", MAX_ATTEMPTS);
    return false;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "All objects have been successfully removed from the planning scene.");
    return true;
  }
}

bool ManipPlanner::attachObject(const std::string &id)
{
  auto planning_scene = psm_->getPlanningScene();
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP_);
  robot_state->setJointGroupPositions(joint_model_group, move_plans_.back().second.trajectory_.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());
  // 创建 AttachedCollisionObject 消息
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = end_effector_name_;
  attached_object.object.id = id;
  attached_object.object.header.frame_id = end_effector_name_;
  attached_object.object.operation = attached_object.object.ADD;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.1, 0.1, 0.2}; // 与原始一致

  attached_object.object.primitives.push_back(primitive);
  geometry_msgs::msg::Pose stick_pose;
  stick_pose.position.x = 0;
  stick_pose.position.y = 0;
  stick_pose.position.z = 0.0;
  stick_pose.orientation.x = 0;
  stick_pose.orientation.y = 0;
  stick_pose.orientation.z = 0;
  stick_pose.orientation.w = 1.0;
  attached_object.object.primitive_poses.push_back(stick_pose);
  attached_object.object.primitives.resize(1);
  attached_object.object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  attached_object.object.primitives[0].dimensions.resize(2);
  attached_object.object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 1.0;
  attached_object.object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.05;
  RCLCPP_INFO(LOGGER, "attatching id:%s", id.c_str());

  const int max_attempts = 10;
  int attempt = 0;

  while (attempt < max_attempts)
  {
    if (!planning_scene->processAttachedCollisionObjectMsg(attached_object))
    {
      RCLCPP_INFO(LOGGER, "attatch fail");
      rclcpp::sleep_for(std::chrono::milliseconds(50)); // 避免过快重试
      continue;
    }
    moveit::core::RobotState &current_state = planning_scene->getCurrentStateNonConst();
    current_state.update(); // 更新状态后再检查附着体
    std::vector<const moveit::core::AttachedBody *> attached_bodies;
    current_state.getAttachedBodies(attached_bodies);
    bool attached = std::any_of(attached_bodies.begin(), attached_bodies.end(),
                                [&id](const moveit::core::AttachedBody *body)
                                {
                                  return body->getName() == id;
                                });
    if (attached)
    {
      RCLCPP_INFO(LOGGER, "Object [%s] successfully attached on attempt %d.", id.c_str(), attempt + 1);
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(50)); // 避免过快重试
    attempt++;
  }

  psm_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  return true;
}

bool ManipPlanner::detatchObject(const std::string &id)
{
  auto planning_scene = psm_->getPlanningScene();
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP_);
  robot_state->setJointGroupPositions(joint_model_group, move_plans_.back().second.trajectory_.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = end_effector_name_;
  attached_object.object.id = id;
  attached_object.object.operation = attached_object.object.REMOVE;
  RCLCPP_INFO(LOGGER, "detaching object");

  // 输出Object是否存在
  moveit::core::RobotState &current_state = planning_scene->getCurrentStateNonConst();
  current_state.update(); // 更新状态后再检查附着体
  std::vector<const moveit::core::AttachedBody *> attached_bodies;
  current_state.getAttachedBodies(attached_bodies);
  for (auto body : attached_bodies)
  {
    RCLCPP_INFO(LOGGER, "Object [%s] attached.", body->getName().c_str());
  }
  if (attached_bodies.empty())
  {
    RCLCPP_WARN(LOGGER, "Detach fail, empty attached body in hand.");
    return false;
  }

  try
  {
    planning_scene->processAttachedCollisionObjectMsg(attached_object);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(LOGGER, "Exception during detatchObject: %s", e.what());
  }

  RemoveCollisionById(id);
  psm_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  return true;
}