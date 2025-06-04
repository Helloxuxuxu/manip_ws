/************************************************************************************
* @file         path_planning_node.cpp
* @brief        rb210的动作server，接收动作指令并规划执行。
* @author       文娱  
* @date         2025-04-15
* @version      V0.1
* @copyright    中煤科工重庆研究院智能化协同创新中心机器人所. 2025 All rights reserved.
*************************************************************************************
*/

#include "rb210_path_planning/path_planner.hpp"
#include "rb210_planning_interfaces/action/pick_place.hpp"


using PickPlace = rb210_planning_interfaces::action::PickPlace;
using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlace>;

class TaskManager: public rclcpp::Node
{
public:
  TaskManager() : Node("path_planning_client")
  {
    // 初始化参数
    if (!this->has_parameter("use_fake_hardware"))
    {
      this->declare_parameter("use_fake_hardware", true);
    }
    use_fake_hardware_ = this->get_parameter("use_fake_hardware").as_bool();
    RCLCPP_INFO(LOGGER, "use_fake_hardware = %s", use_fake_hardware_ ? "true" : "false");
    // move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "planning_group");
    using namespace std::placeholders;
    // 初始化action Server
    plan_action_server_ = rclcpp_action::create_server<PickPlace>(
      this,
      "pick_place",
      std::bind(&TaskManager::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TaskManager::handle_cancel, this, std::placeholders::_1),
      std::bind(&TaskManager::handle_accepted, this, std::placeholders::_1)
    );
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    manip_planner = std::make_shared<ManipPlanner>(node_options);

    // 使用实物硬件
    if(!use_fake_hardware_){
      traj_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/follow_joint_trajectory_rb210");
      traj_action_client_->wait_for_action_server();
      move_action_client_ = rclcpp_action::create_client<MoveTarget>(
        this, "/move_to_target_rb210");
      move_action_client_->wait_for_action_server();
    }

  }
  ~TaskManager(){}
  

private:
  std::shared_ptr<ManipPlanner> manip_planner;
  // moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_;
  bool use_fake_hardware_ = true;
  // 动作接口
  rclcpp_action::Server<PickPlace>::SharedPtr plan_action_server_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr traj_action_client_;
  rclcpp_action::Client<MoveTarget>::SharedPtr move_action_client_;
  bool action_running_flag = false;
  bool stop_flag = false;
  // bool executePlans(std::string stick_id,MovePlansMap move_plans, std::map<std::string, geometry_msgs::msg::Pose> poses_map_);
  // 动作接口
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const PickPlace::Goal> goal){
    if(!action_running_flag){
      RCLCPP_INFO(LOGGER, "Received pick and place poses, ready to run.");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }else{
      RCLCPP_INFO(LOGGER, "Received pick and place poses, but an action is running.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePickPlace> goal_handle){
    stop_flag = true;
    RCLCPP_INFO(LOGGER, "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle){
    std::thread{std::bind(&TaskManager::execute, this, goal_handle)}.detach();
  }
  void execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle){
    RCLCPP_INFO(LOGGER,"executing...");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PickPlace::Result>();
    auto feedback = std::make_shared<PickPlace::Feedback>();
  
    auto pick_pose = goal->pick_pose;
    auto place_pose = goal->place_pose;
    auto stick_id = goal->object_id;
    action_running_flag = true;
    stop_flag = false;
    // 添加钻杆
    manip_planner->AddCylinderByPose(stick_id, pick_pose,1.0,0.05);
    
    // 规划任务
    auto code = manip_planner->pickAndPlace2(stick_id, pick_pose, place_pose);
    // 处理取消情况
    if (goal_handle->is_canceling()) {
      result->success = code;
      result->message = "Cancelled Pick and Place.";
      goal_handle->canceled(result);
      RCLCPP_INFO(LOGGER, "Goal canceled");
      action_running_flag = false;
      return;
    }
  
    if(code != ErrorCode::SUCCESS){
      result->success = code;
      result->message = "Fail to plan.";
      goal_handle->abort(result);
      RCLCPP_INFO(LOGGER, "Server Goal fail");
      action_running_flag = false;
      return;
    }
  
    // 规划成功则执行任务
    RCLCPP_INFO(LOGGER, "Ready to executePlans");
    bool execute_ok = false;
    if(use_fake_hardware_){
      execute_ok = manip_planner->executePlans();
    }else{
      execute_ok = executePlans(stick_id, manip_planner->getMovePlansMap(),manip_planner->getPosesMap());
    }

    if(!execute_ok){
      result->success = ErrorCode::EXECUTE_FAILED;
      result->message = "Fail to execute.";
      goal_handle->abort(result);
      RCLCPP_INFO(LOGGER, "Server Goal fail");
      action_running_flag = false;
      return;
    }
  
    result->success = ErrorCode::SUCCESS;
    result->message = "Finish Pick and Place.";
    goal_handle->succeed(result);
    RCLCPP_INFO(LOGGER, "Server Goal succeeded");
  
    action_running_flag = false;
  }

  bool sendPath(const moveit_msgs::msg::RobotTrajectory& traj){
    // ---发送给实物执行---
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory = traj.joint_trajectory;
  
    auto future_goal_handle = traj_action_client_->async_send_goal(goal_msg);
  
    if (future_goal_handle.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_ERROR(LOGGER, "Goal response timeout.");
      return false;
    }
    
    auto goal_handle = future_goal_handle.get();  // 获取 GoalHandle
    if (!goal_handle) {
      RCLCPP_ERROR(LOGGER, "Goal was rejected by action server.");
      return false;
    }
    // 等待结果
    auto result_future = traj_action_client_->async_get_result(goal_handle);
    RCLCPP_INFO(LOGGER, "Waiting for result...");
    if (result_future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
      auto result = result_future.get();
      RCLCPP_INFO(LOGGER, "Goal completed. Result code: %d", static_cast<int>(result.code));
    } else {
      RCLCPP_ERROR(LOGGER, "Timed out while waiting for result.");
      return false;
    }
    return true;
  }
  
  bool sendMoveL(const geometry_msgs::msg::Pose& dcart_target){
    // ---发送给实物执行---
    auto goal_msg = MoveTarget::Goal();
    goal_msg.pos_target.resize(6);
    goal_msg.pos_target[0] = dcart_target.position.x;
    goal_msg.pos_target[1] = dcart_target.position.y;
    goal_msg.pos_target[2] = dcart_target.position.z;
    auto q = dcart_target.orientation;
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    // (modify) 需要与机械臂实际坐标对齐
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    goal_msg.pos_target[3] = roll;
    goal_msg.pos_target[4] = pitch;
    goal_msg.pos_target[5] = yaw;
  
    goal_msg.type = 2;
  
    auto future_goal_handle = move_action_client_->async_send_goal(goal_msg);
    if (future_goal_handle.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_ERROR(LOGGER, "Goal response timeout.");
      return false;
    }
    auto goal_handle = future_goal_handle.get();  // 获取 GoalHandle
    if (!goal_handle) {
      RCLCPP_ERROR(LOGGER, "Goal was rejected by action server.");
      return false; 
    }
    // 等待结果
    auto result_future = move_action_client_->async_get_result(goal_handle);
    RCLCPP_INFO(LOGGER, "Waiting for result...");
    if (result_future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
      auto result = result_future.get();
      RCLCPP_INFO(LOGGER, "Goal completed. Result code: %d", static_cast<int>(result.code));
    } else {
      RCLCPP_ERROR(LOGGER, "Timed out while waiting for result.");
      return false;
    }
    return true;
  }
  
  bool sendMoveJ(const std::vector<double>& joint_target){
    if(joint_target.size()!=6){return false;}
    // ---发送给实物执行---
    auto goal_msg = MoveTarget::Goal();
    goal_msg.joints_target = joint_target;
    goal_msg.type = 0;
    auto future_goal_handle = move_action_client_->async_send_goal(goal_msg);
    if (future_goal_handle.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_ERROR(LOGGER, "Goal response timeout.");
      return false;
    }
    auto goal_handle = future_goal_handle.get();  // 获取 GoalHandle
    if (!goal_handle) {
      RCLCPP_ERROR(LOGGER, "Goal was rejected by action server.");
      return false;
    }
    // 等待结果
    auto result_future = move_action_client_->async_get_result(goal_handle);
    RCLCPP_INFO(LOGGER, "Waiting for result...");
    if (result_future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
      auto result = result_future.get();
      RCLCPP_INFO(LOGGER, "Goal completed. Result code: %d", static_cast<int>(result.code));
    } else {
      RCLCPP_ERROR(LOGGER, "Timed out while waiting for result.");
      return false;
    }
    return true;
  }

  // 真实执行任务
  bool executePlans(std::string stick_id,MovePlansMap move_plans, std::map<std::string, geometry_msgs::msg::Pose> poses_map_){
    moveit_msgs::msg::RobotTrajectory current_traj;
    /* 1. To pre_pick */
    RCLCPP_INFO(LOGGER,"Start execute plan: pre_pick.");
    if(move_plans.find("pre_pick")!=move_plans.end()){
      current_traj = move_plans["pre_pick"].trajectory_;
    }else{
      return false;
    }
    // 执行路径
    RCLCPP_INFO(LOGGER,"Start executing path.");
    if(!sendPath(current_traj)){
      RCLCPP_INFO(LOGGER,"sendPath execution fail.");
      return false;
    }

    /* 2. To pick */
    RCLCPP_INFO(LOGGER,"Start execute plan: pick.");

    geometry_msgs::msg::Pose target_pos; // (modify) 如果用EyeInHand数据需要做修改
    if(poses_map_.find("pick") != poses_map_.end()){
      target_pos = poses_map_["pick"];
    }else{return false;}
    if(!sendMoveL(target_pos)){
      RCLCPP_INFO(LOGGER,"Plan execution fail.");
      return false;
    };

    RCLCPP_INFO(LOGGER,"Start execute plan: post_pick.");
    if(poses_map_.find("post_pick") != poses_map_.end()){
      target_pos = poses_map_["post_pick"];
    }else{return false;}
    if(!sendMoveL(target_pos)){
      RCLCPP_INFO(LOGGER,"Plan execution fail.");
      return false;
    };

    /* 3. To pre_place */
    RCLCPP_INFO(LOGGER,"Start execute plan: pre_place.");
    if(move_plans.find("pre_place")!=move_plans.end()){
      current_traj = move_plans["pre_place"].trajectory_;
    }else{
      return false;
    }
    // 执行路径
    RCLCPP_INFO(LOGGER,"Start executing path.");
    if(!sendPath(current_traj)){
      RCLCPP_INFO(LOGGER,"sendPath execution fail.");
      return false;
    }

    /* 4. To place */
    RCLCPP_INFO(LOGGER,"Start execute plan: place.");
    if(poses_map_.find("place") != poses_map_.end()){
      target_pos = poses_map_["place"];
    }else{return false;}
    if(!sendMoveL(target_pos)){
      RCLCPP_INFO(LOGGER,"Plan execution fail.");
      return false;
    };

    RCLCPP_INFO(LOGGER,"Start execute plan: post_place.");
    if(poses_map_.find("post_place") != poses_map_.end()){
      target_pos = poses_map_["post_place"];
    }else{return false;}
    if(!sendMoveL(target_pos)){
      RCLCPP_INFO(LOGGER,"Plan execution fail.");
      return false;
    };
    

    /* 5. To home */
    RCLCPP_INFO(LOGGER,"Start execute plan: home.");
    if(move_plans.find("home")!=move_plans.end()){
      current_traj = move_plans["home"].trajectory_;
    }else{
      return false;
    }
    // 执行路径
    RCLCPP_INFO(LOGGER,"Start executing path.");
    if(!sendPath(current_traj)){
      RCLCPP_INFO(LOGGER,"sendPath execution fail.");
      return false;
    }

    return true;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto task_manager = std::make_shared<TaskManager>();
  rclcpp::spin(task_manager);
  return 0;
}