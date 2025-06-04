/**
 * @file rb210_driver.hpp
 * @author wenyu (785895681@qq.com)
 * @brief 机械臂的运动控制和状态反馈,实现状态的读取和控制指令的下发，可以参考ur
 * @version 0.1
 * @date 2025-04-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "GRC01.h"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rb210_planning_interfaces/action/move_target.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/msg/u_int8.hpp"

// 状态反馈枚举
enum class RunningState : uint8_t {
  STOP        = 0,
  PAUSE       = 1,
  EME_STOP    = 2,
  RUNNING     = 3
};

// 执行错误代码
enum class ExecuteErrorCode : uint8_t {
  SUCCESS    = 0,
  FAILURE    = 1,
  PAUSE      = 2,
  INIT_FAIL  = 3
};

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using MoveTarget = rb210_planning_interfaces::action::MoveTarget;
using GoalHandleMoveTarget = rclcpp_action::ServerGoalHandle<MoveTarget>;

class DriverNode : public rclcpp::Node
{
public:
  DriverNode();
  ~DriverNode(){}
private:
  void readData();

  // 轨迹action回调函数
  rclcpp_action::GoalResponse traj_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse traj_handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
  void traj_handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
  void traj_execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

  // 目标运动回调函数
  rclcpp_action::GoalResponse move_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveTarget::Goal> goal);
  rclcpp_action::CancelResponse move_handle_cancel(
    const std::shared_ptr<GoalHandleMoveTarget> goal_handle);
  void move_handle_accepted(const std::shared_ptr<GoalHandleMoveTarget> goal_handle);
  void move_execute(const std::shared_ptr<GoalHandleMoveTarget> goal_handle);
  
  // 处理键值输入cmd
  void handleCmd(const std_msgs::msg::UInt8::SharedPtr msg);

private:
  // 动作列表，实现路径控制和目标控制
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr traj_action_server_;
  rclcpp_action::Server<MoveTarget>::SharedPtr move_action_server_;
  // 消息发布列表
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dcart_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // 按键控制指令下发
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr key_input_sub_;

  // 通信
  const int cliSocket_ = 100;

  // 状态信息：关节状态，末端位姿，状态码，使能状态
  std::vector<double> joint_positions_;
  std::vector<double> dcart_positions_;
  ushort running_state_ = 0; // 默认当前空闲状态
  ushort init_can_use_ = 0; // 使能状态

  // 控制参数：控制过程参数和
  double moveJ_vel = 10;
  short zone = 2;

  // 标志
  bool running_flag_ = false;

};
