#include "rb210_driver/rb210_driver.hpp"

DriverNode::DriverNode() : Node("rb210_driver")
{
  joint_positions_ = std::vector<double>(6, 0);
  dcart_positions_ = std::vector<double>(6, 0);
  // 获取机器人状态
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  dcart_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("dcart_states", 10);
  key_input_sub_ = this->create_subscription<std_msgs::msg::UInt8>("key_input_cmd", 10,
                                                                   std::bind(&DriverNode::handleCmd, this, std::placeholders::_1));

  // timer_ = this->create_wall_timer(
  //   std::chrono::milliseconds(20),  // 50Hz
  //   std::bind(&DriverNode::readData, this));

  // 执行机器人轨迹
  traj_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "follow_joint_trajectory_rb210",
      std::bind(&DriverNode::traj_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DriverNode::traj_handle_cancel, this, std::placeholders::_1),
      std::bind(&DriverNode::traj_handle_accepted, this, std::placeholders::_1));
  // 执行目标位姿
  move_action_server_ = rclcpp_action::create_server<MoveTarget>(
      this,
      "move_to_target_rb210",
      std::bind(&DriverNode::move_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DriverNode::move_handle_cancel, this, std::placeholders::_1),
      std::bind(&DriverNode::move_handle_accepted, this, std::placeholders::_1));

  // 声明参数，后续在launch文件中添加
  declare_parameter<double>("moveJ_vel", 15);
  declare_parameter<short>("zone", 2);
  // 初始化参数
  get_parameter("moveJ_vel", moveJ_vel);
  get_parameter("zone", zone);
}

void DriverNode::readData()
{
  // 读取当前关节
  int count = 0;
  double joints[6] = {0};
  if (GRC01_GetJointActPos(cliSocket_, true, count, joints) != 0)
  {
    RCLCPP_WARN(this->get_logger(), "GRC01_GetDcarActPos fail.");
    return;
  }
  if (count != 6)
  {
    RCLCPP_WARN(get_logger(), "Get current joints fail.");
    return;
  }
  for (int i = 0; i < 6; i++)
  {
    joint_positions_[i] = joints[i];
  }
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->get_clock()->now();
  msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  msg.position = joint_positions_;
  joint_state_pub_->publish(msg);
  rclcpp::Parameter traj_param;
  // 读取当前笛卡尔坐标
  std::vector<double> dcart_positions(6, 0.0); // 六自由度位姿
  count = 0;
  double dcart[6] = {0};
  if (GRC01_GetDcarActPos(cliSocket_, true, count, dcart) != 0)
  {
    RCLCPP_WARN(this->get_logger(), "GRC01_GetDcarActPos fail.");
    return;
  }
  if (count != 6)
  {
    RCLCPP_WARN(get_logger(), "GRC01_GetDcarActPos fail.");
    return;
  }
  for (int i = 0; i < 6; i++)
  {
    dcart_positions_[i] = joints[i];
  }
  tf2::Quaternion q;
  q.setRPY(dcart_positions_[5], dcart_positions_[4], dcart_positions_[3]);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = "base_link";
  pose.pose.position.x = dcart_positions_[0];
  pose.pose.position.y = dcart_positions_[1];
  pose.pose.position.z = dcart_positions_[2];
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  dcart_state_pub_->publish(pose);

  // 获取当前状态
  if (GRC01_GetSysRunState(cliSocket_, true, running_state_) != 0)
  {
    RCLCPP_WARN(this->get_logger(), "GetSysRunState fail.");
    return;
  }
  if (GRC01_GetintCanUse(cliSocket_, true, init_can_use_) != 0)
  {
    RCLCPP_WARN(this->get_logger(), "GRC01_GetintCanUse fail.");
    return;
  }
}

// 收到新的 goal 请求
rclcpp_action::GoalResponse DriverNode::traj_handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  if (running_flag_)
  {
    RCLCPP_INFO(get_logger(), "running flag is true.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Recieve traj request, there are %zu points", goal->trajectory.points.size());
  // 打印所有点
  for (auto point : goal->trajectory.points)
  {
    if (point.positions.size() != 6)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "received traj point: %.4f,%.4f,%.4f,%.4f,%.4f,%.4f.", point.positions[0], point.positions[1], point.positions[2],
                point.positions[3], point.positions[4], point.positions[5]);
  }

  // 这里可根据需求检查 joint_names、points.size() 等，决定是否接受
  // 检查点数
  auto traj = goal->trajectory.points;
  if (traj.size() <= 1 || traj.size() > 100)
  {
    RCLCPP_INFO(get_logger(), "Empty trajctory.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // 检查起始位置和当前位置的差异，避免较大的初始值跳跃
  std::vector<double> start_pose = goal->trajectory.points[0].positions;
  const double tolerance = 0.02; // 初始状态的容差
  double difference = 0;
  for (int i = 0; i < 6; i++)
  {
    difference += (start_pose[i] - joint_positions_[i]);
  }
  if (difference > tolerance)
  {
    RCLCPP_INFO(get_logger(), "Start position differ too much.");
    RCLCPP_INFO(get_logger(), "Start position: %.5f,%.5f,%.5f,%.5f,%.5f,%.5f.", start_pose[0], start_pose[1], start_pose[2],
                start_pose[3], start_pose[4], start_pose[5]);
    RCLCPP_INFO(get_logger(), "Current position: %.5f,%.5f,%.5f,%.5f,%.5f,%.5f.", joint_positions_[0], joint_positions_[1], joint_positions_[2],
                joint_positions_[3], joint_positions_[4], joint_positions_[5]);
    // return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 收到 cancel 请求
rclcpp_action::CancelResponse DriverNode::traj_handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  RCLCPP_WARN(get_logger(), "Traj execute cancelled.");
  running_flag_ = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 接受 goal 后的回调（启动异步执行）
void DriverNode::traj_handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  // 使用新线程执行轨迹发送，避免阻塞 executor
  std::thread{std::bind(&DriverNode::traj_execute, this, std::placeholders::_1), goal_handle}.detach();
}

// 真正的轨迹发送逻辑
void DriverNode::traj_execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  running_flag_ = true;
  RCLCPP_INFO(get_logger(), "Start traj_execute.");
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  goal->trajectory.points;
  get_parameter("moveJ_vel", moveJ_vel);
  get_parameter("zone", zone);

  // ---发送轨迹的逻辑---
  RCLCPP_INFO(get_logger(), "preparing traj.");
  GRC01_ClearPath(cliSocket_);
  for (int i = 0; i < goal->trajectory.points.size(); i++)
  {
    auto joints = goal->trajectory.points[i].positions;
    if (i == 0 || i == goal->trajectory.points.size() - 1)
    { // 首末位置zone=0
      GRC01_AddMovePath(cliSocket_, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], 0, 0, moveJ_vel, 0, 0);
    }
    else
    {
      GRC01_AddMovePath(cliSocket_, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], 0, 0, moveJ_vel, zone, 0);
    }
  }
  // 执行开始
  RCLCPP_INFO(get_logger(), "start executing traj.");
  GRC01_RunPath(cliSocket_);
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  rclcpp::Rate rate(5);
  short run_state = 0;
  GRC01_GetRunPathState(cliSocket_, run_state);
  if (run_state == 0)
  {                                                                                          // 轨迹未启动成功
    result->error_code = control_msgs::action::FollowJointTrajectory_Result::INVALID_JOINTS; // 也可以自定义，比如设置一个 CANCELLED 错误码
    result->error_string = "RB210 execute path initialize fail.";
    RCLCPP_INFO(get_logger(), "launch traj fail.");
    goal_handle->abort(result);
    RCLCPP_INFO(get_logger(), "Goal abort.");
    running_flag_ = false;
    return;
  }

  // 循环等待执行成功与否
  while (1)
  {
    RCLCPP_INFO(get_logger(), "waiting for success.");
    GRC01_GetRunPathState(cliSocket_, run_state);
    if (run_state == 1)
    { // 空闲状态则退出
      break;
    }
    // 反馈当前状态
    feedback->actual.positions = std::vector<double>(6, 0);
    for (int i = 0; i < 6; i++)
    {
      feedback->actual.positions[i] = joint_positions_[i];
    }
    goal_handle->publish_feedback(feedback);
    if (goal_handle->is_canceling())
    {
      RCLCPP_WARN(get_logger(), "Recieve cancelling request.");

      // === 1. 发送停止命令 ===

      // === 2. 填充取消结果并返回 ===
      result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL; // 也可以自定义，比如设置一个 CANCELLED 错误码
      result->error_string = "RB210 execute path cancelled.";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Traj execute cancelled.");
      running_flag_ = false;
      return; // 直接退出execute()
    }

    rate.sleep();
  }

  // 轨迹发送完成
  result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
  result->error_string = "RB210 successfully execute path.";
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(), "RB210 successfully execute path.");
  running_flag_ = false;
}

// 运动至一个目标
rclcpp_action::GoalResponse DriverNode::move_handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MoveTarget::Goal> goal)
{
  if (running_flag_)
  {
    RCLCPP_INFO(get_logger(), "running flag is true.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  (void)uuid;
  if (goal->type == 0)
  {
    if (goal->joints_target.size() != 6)
    {
      RCLCPP_INFO(get_logger(), "goal demension is not right.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "Recieve goal target: %.3f,%.3f,%.3f,%.3f,%.3f,%.3f.",
                goal->joints_target[0], goal->joints_target[1], goal->joints_target[2],
                goal->joints_target[3], goal->joints_target[4], goal->joints_target[5]);
    // (TODO) 关节目标范围校验
  }
  else if (goal->type == 2)
  {
    if (goal->pos_target.size() != 6)
    {
      RCLCPP_INFO(get_logger(), "goal demension is not right.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "Recieve goal target: %.3f,%.3f,%.3f,%.3f,%.3f,%.3f.",
                goal->pos_target[0], goal->pos_target[1], goal->pos_target[2],
                goal->pos_target[3], goal->pos_target[4], goal->pos_target[5]);
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 收到 cancel 请求
rclcpp_action::CancelResponse DriverNode::move_handle_cancel(
    const std::shared_ptr<GoalHandleMoveTarget> goal_handle)
{
  RCLCPP_WARN(get_logger(), "Traj execute cancelled.");
  running_flag_ = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 接受 goal 后的回调（启动异步执行）
void DriverNode::move_handle_accepted(const std::shared_ptr<GoalHandleMoveTarget> goal_handle)
{
  // 使用新线程执行轨迹发送，避免阻塞 executor
  std::thread{std::bind(&DriverNode::move_execute, this, std::placeholders::_1), goal_handle}.detach();
}

// 真正的轨迹发送逻辑
void DriverNode::move_execute(const std::shared_ptr<GoalHandleMoveTarget> goal_handle)
{
  running_flag_ = true;
  RCLCPP_INFO(get_logger(), "Start moving To target.");
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveTarget::Result>();
  auto feedback = std::make_shared<MoveTarget::Feedback>();

  if (goal->type == 0)
  {
    // MOVJ
    get_parameter("moveJ_vel", moveJ_vel);
    RCLCPP_INFO(get_logger(), "moveJ_vel is: %.3f.", moveJ_vel);
    GRC01_SetJointSpeedRatio(cliSocket_, moveJ_vel);
    int success = GRC01_MoveJ(cliSocket_, goal->joints_target[0], goal->joints_target[1], goal->joints_target[2],
                              goal->joints_target[3], goal->joints_target[4], goal->joints_target[5], 0, 0, 1); // 阻塞运行
    if (success == 0)
    {
      // 轨迹执行完成
      result->error_code = 0;
      result->error_string = "RB210 successfully execute moveJ.";
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "RB210 successfully execute moveJ.");
    }
    else
    {
      // 轨迹执行完成
      result->error_code = 1;
      result->error_string = "RB210 fail to execute moveJ.";
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "RB210 fail to execute moveJ.");
    }
  }
  else if (goal->type == 2)
  {
    // MOVL
    int success = GRC01_MoveL(cliSocket_, goal->pos_target[0], goal->pos_target[1], goal->pos_target[2],
                              goal->pos_target[3], goal->pos_target[4], goal->pos_target[5], 0, 0, 1); // 阻塞运行
    if (success == 0)
    {
      // 轨迹执行完成
      result->error_code = 0;
      result->error_string = "RB210 successfully execute moveL.";
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "RB210 successfully execute moveL.");
    }
    else
    {
      // 轨迹执行完成
      result->error_code = 1;
      result->error_string = "RB210 fail to execute moveL.";
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "RB210 fail to execute moveL.");
    }
  }
  running_flag_ = false;
}

void DriverNode::handleCmd(const std_msgs::msg::UInt8::SharedPtr msg)
{
  unsigned char cmd_value = msg->data;
  RCLCPP_INFO(this->get_logger(), "Received cmd byte: %u (char: %c)", cmd_value, cmd_value);
  if (GRC01_SimulateKeyInput(cliSocket_, cmd_value) != 0)
  {
    RCLCPP_WARN(this->get_logger(), "GRC01_SimulateKeyInput fail.");
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}