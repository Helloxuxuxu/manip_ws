#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rb210_planning_interfaces/action/move_target.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "yaml-cpp/yaml.h"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using MoveTarget = rb210_planning_interfaces::action::MoveTarget;
using GoalHandleMoveTarget = rclcpp_action::ClientGoalHandle<MoveTarget>;

class TestNode : public rclcpp::Node
{
public:
  TestNode() : Node("test_node")
  {
    // 发布话题
    publisher_ = this->create_publisher<std_msgs::msg::UInt8>("test_topic", 10);

    // 创建定时器，每 2 秒发布一次话题
    // timer_ = this->create_wall_timer(
    //   std::chrono::seconds(2),
    //   std::bind(&TestNode::publish_message, this));

    // 创建动作客户端
    traj_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, "follow_joint_trajectory_rb210");
    move_action_client_ = rclcpp_action::create_client<MoveTarget>(this, "move_to_target_rb210");

    // 执行指令
    if(!readConfig())return;
    sendMoveGoal();
    sendTrajGoal();

  }

private:
  void publish_message()
  {
    auto msg = std_msgs::msg::UInt8();
    msg.data = 1;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published message");
  }
  bool readConfig(){
      traj_goal_.clear();
      // 通过yaml读取配置文件
      this->declare_parameter<std::string>("config_path", "");
      std::string config_path = this->get_parameter("config_path").as_string();
      RCLCPP_INFO(get_logger(),"config_path:%s",config_path.c_str());
      YAML::Node config = YAML::LoadFile(config_path);
      // 读入目标
      std::string name = config["name"].as<std::string>();
      joint_goal_ = config["joint_target"].as<std::vector<double>>();
      dcart_goal_ = config["dcart_target"].as<std::vector<double>>();

      RCLCPP_INFO(this->get_logger(), "joint_target: %f,%f,%f,%f,%f,%f.", 
      joint_goal_[0],joint_goal_[1],joint_goal_[2],
      joint_goal_[3],joint_goal_[4],joint_goal_[5]);
      std::vector<std::vector<double>> waypoints;
      for (const auto &node : config["waypoints"]) {
        auto joint_angles = node["joint_angles"].as<std::vector<double>>();
        waypoints.push_back(joint_angles);
      }
      RCLCPP_INFO(this->get_logger(), "Task name: %s", name.c_str());
      for (size_t i = 0; i < waypoints.size(); ++i) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = waypoints[i];
        traj_goal_.push_back(point);
        RCLCPP_INFO(this->get_logger(), "Waypoint %zu: joints point = %f,%f,%f,%f,%f,%f.", i, 
        waypoints[i][0],waypoints[i][1],waypoints[i][2],
        waypoints[i][3],waypoints[i][4],waypoints[i][5]);
      }
      return true;
  }
  void sendTrajGoal()
  {
    if (!traj_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Action server not available");
      return;
    }
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.points = traj_goal_;
    auto future_goal_handle = traj_action_client_->async_send_goal(goal_msg);

    // 同步等待结果
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Goal was not accepted.");
      return;
    }
    
    auto goal_handle = future_goal_handle.get();  // 阻塞获取 GoalHandle
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server.");
      return;
    }
    // 等待结果
    auto result_future = traj_action_client_->async_get_result(goal_handle);
    RCLCPP_INFO(get_logger(), "traj_action_client: Client Waiting for result...");
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = result_future.get();
      RCLCPP_INFO(get_logger(), "traj_action_client: Goal completed. Result code: %d", static_cast<int>(result.result->error_code));
    }
    else{
      RCLCPP_ERROR(get_logger(), "Failed to get result");
      return;
    }
  }

  void sendMoveGoal()
  {
    if (!move_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Action server not available");
      return;
    }
    if(joint_goal_.size()!=6){
      RCLCPP_WARN(this->get_logger(), "joint_goal demension error.");
      return;
    }
    auto goal_msg = MoveTarget::Goal();
    goal_msg.joints_target = joint_goal_;
    goal_msg.type = 0;
    goal_msg.pos_target = dcart_goal_;
    goal_msg.type = 2;

    // 设置运动速度
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "rb210_driver");
    // 等待服务可用
    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "waiting parameter server...");
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    rclcpp::Parameter new_param("moveJ_vel", 0.32);
    parameters_client->set_parameters({new_param});

    auto future_goal_handle = move_action_client_->async_send_goal(goal_msg);

    // 同步等待结果
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "move_action_client: Goal was not accepted.");
      return;
    }
    
    auto goal_handle = future_goal_handle.get();  // 获取 GoalHandle
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "move_action_client: Goal was rejected by the action server.");
      return;
    }
    // 等待结果
    auto result_future = move_action_client_->async_get_result(goal_handle);
    RCLCPP_INFO(get_logger(), "move_action_client: Waiting for result...");
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = result_future.get();
      RCLCPP_INFO(get_logger(), "move_action_client: Goal completed. Result code: %d", static_cast<int>(result.result->error_code));
    }
    else{
      RCLCPP_ERROR(get_logger(), "Failed to get result");
      return;
    }
  }


  void traj_result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "traj_result_callback: Result received:");
      RCLCPP_INFO(this->get_logger(), "traj_result_callback: error_code: %d", result.result->error_code);
    } else {
      RCLCPP_WARN(this->get_logger(), "traj_result_callback: Goal failed or was canceled.");
    }
  }

  void move_result_callback(const GoalHandleMoveTarget::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "move_result_callback: Result received:");
      RCLCPP_INFO(this->get_logger(), "move_result_callback: error_code: %d", result.result->error_code);
    } else {
      RCLCPP_WARN(this->get_logger(), "move_result_callback: Goal failed or was canceled.");
    }
  }

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr traj_action_client_;
  rclcpp_action::Client<MoveTarget>::SharedPtr move_action_client_;

  // 目标
  std::vector<double> joint_goal_;
  std::vector<double> dcart_goal_;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> traj_goal_;

};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>();
  RCLCPP_INFO(node->get_logger(), "\033[34mNode started!!!\033[30m");
  // 创建多线程执行器并添加节点
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(node);
  // executor.spin();

  // 创建单线程执行器并添加节点
  rclcpp::executors::SingleThreadedExecutor single_thread_executor;
  single_thread_executor.add_node(node);
  single_thread_executor.spin();

  rclcpp::shutdown();
  return 0;
}