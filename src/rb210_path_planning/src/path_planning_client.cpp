/************************************************************************************
 * @file         path_planning_node.cpp
 * @brief        rb210的动作server，接收动作指令并规划执行。
 * @author       文娱
 * @date         2025-04-15
 * @version      V0.1
 * @copyright    中煤科工重庆研究院智能化协同创新中心机器人所. 2025 All rights reserved.
 *************************************************************************************
 */

#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rb210_planning_interfaces/action/pick_place.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/msg/int32.hpp>
#include "rb210_planning_interfaces/srv/pose_req.hpp"

using PickPlace = rb210_planning_interfaces::action::PickPlace;
using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlace>;

// 状态反馈枚举
enum class ServerState : uint8_t
{
  SUCCESS = 0,
  ABORTED = 1,
  EME_STOP = 2,
  RUNNING = 3
};

static const rclcpp::Logger LOGGER = rclcpp::get_logger("path_planning_client"); // 创建日志记录器

class PathPlanningClient : public rclcpp::Node
{
public:
  PathPlanningClient() : Node("path_planning_client")
  {
    // 创建动作客户端
    client_ptr_ = rclcpp_action::create_client<PickPlace>(this, "pick_place");
    declare_parameter<int>("stick_num_id", 0); // 声明一个当前stick的全局变量
    stick_num_id_ = 0;
    stick_req_pub_ = this->create_publisher<std_msgs::msg::Int32>("stick_req", 10);
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "int_topic", 10, std::bind(&PathPlanningClient::topic_callback, this, std::placeholders::_1));
    pos_client_ = this->create_client<rb210_planning_interfaces::srv::PoseReq>("get_stick_pose");
    pos_client_->wait_for_service();

    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "sim_visual_detection_node");
    parameters_client_->wait_for_service();
  }

  void sendGoal()
  {
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "waiting for action server");
    if (!this->client_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }
    goal_done_ = false;
    goal_success_ = false;
    RCLCPP_INFO(this->get_logger(), "Sending goal stick: %d", stick_num_id_);
    if (place_poses_.empty())
    {
      RCLCPP_INFO(LOGGER, "No more place_poses to place stick.");
      return;
    }
    auto goal_msg = PickPlace::Goal();
    goal_msg.pick_pose = current_stick_pose_;
    goal_msg.place_pose = place_poses_.front();
    goal_msg.object_id = stick_ids_.front();

    // 发送目标并设置回调
    auto send_goal_options = rclcpp_action::Client<PickPlace>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&PathPlanningClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&PathPlanningClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&PathPlanningClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  bool is_goal_done() { return goal_done_; }
  bool is_goal_success() { return goal_success_; }
  // 如果orientation不合法，则没有要规划的pick位姿了。
  bool is_stick_exist()
  {
    double a = (current_stick_pose_.orientation.x * current_stick_pose_.orientation.x + current_stick_pose_.orientation.y * current_stick_pose_.orientation.y +
                current_stick_pose_.orientation.z * current_stick_pose_.orientation.z + current_stick_pose_.orientation.w * current_stick_pose_.orientation.w);
    RCLCPP_INFO(LOGGER, "current_stick_pose_ orientation sum is %.2f.", a);
    return a > 0.9 || stick_num_id_ >= (place_poses_.size() - 1);
  }

private:
  // 话题和动作
  rclcpp_action::Client<PickPlace>::SharedPtr client_ptr_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr stick_req_pub_;
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Client<rb210_planning_interfaces::srv::PoseReq>::SharedPtr pos_client_;

  geometry_msgs::msg::Pose current_stick_pose_;

  bool goal_done_ = false;
  bool goal_success_ = false;

  void goal_response_callback(rclcpp_action::ClientGoalHandle<PickPlace>::SharedPtr goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      rclcpp_action::ClientGoalHandle<PickPlace>::SharedPtr,
      const std::shared_ptr<const PickPlace::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->current_state)
    {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<PickPlace>::WrappedResult &result)
  {
    goal_done_ = true;
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
    {
      if (result.result->success)
      {
        goal_success_ = true;
        place_poses_.pop_front();
        stick_ids_.pop_front();
        stick_num_id_++;
        RCLCPP_INFO(this->get_logger(), "Goal execute success");
      }
      else
      {
        goal_success_ = false;
        RCLCPP_INFO(this->get_logger(), "Goal execute fail");
      }
    }
    break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->message)
    {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  int stick_num_id_ = 1;
  // std::list<geometry_msgs::msg::Pose> pick_poses_;
  std::list<geometry_msgs::msg::Pose> place_poses_;
  std::list<std::string> stick_ids_;

  // 场景处理
  bool AddCylinderByPose(const std::string &id, const geometry_msgs::msg::Pose &pose, double height, double radius)
  {
    if (!this->client_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(LOGGER, "Client add stick.");
    moveit::planning_interface::PlanningSceneInterface psi; // moveit封装后的规划场景接口
    moveit_msgs::msg::CollisionObject cylinder_obj;
    cylinder_obj.id = id;
    cylinder_obj.header.frame_id = "world";

    cylinder_obj.primitives.resize(1);
    cylinder_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder_obj.primitives[0].dimensions.resize(2);
    cylinder_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = height;
    cylinder_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = radius;

    cylinder_obj.primitive_poses.push_back(pose);
    cylinder_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.addCollisionObjects({cylinder_obj});
    // AddCollisionObject(cylinder_obj);
    return true;
  }

  // ------------------------
public:
  /**
   * @brief 在stick_poses中选择合适的poses，
   *
   * @param stick_poses 场景中识别到的stick位姿集合
   * @param stick_pose 提取的stick位姿
   * @param target_pose 放置的目标位姿
   * @return std::string
   */
  void arrangePickPlace()
  {
    stick_ids_.clear();
    place_poses_.clear();
    RCLCPP_INFO(LOGGER, "Arrange task");

    // 放置位姿应该是固定的序列
    std::vector<geometry_msgs::msg::Pose> place_poses;
    geometry_msgs::msg::Pose place_pose1;
    place_pose1.orientation.x = -0.5;
    place_pose1.orientation.y = 0.5;
    place_pose1.orientation.z = -0.5;
    place_pose1.orientation.w = 0.5;
    place_pose1.position.x = 0.9;
    place_pose1.position.y = -1.7;
    place_pose1.position.z = 0.2 - 0.686;
    geometry_msgs::msg::Pose place_pose2 = place_pose1;
    place_pose2.position.y = -1.5;
    geometry_msgs::msg::Pose place_pose3 = place_pose1;
    place_pose3.position.y = -1.3;
    place_poses_.push_back(place_pose1);
    place_poses_.push_back(place_pose2);
    place_poses_.push_back(place_pose3);

    for (int i = 0; i < place_poses_.size(); i++)
    {
      std::string stick_id = "stick_" + std::to_string(stick_num_id_ + i);
      stick_ids_.push_back(stick_id);

      // 添加到场景中
      // AddCylinderByPose(stick_id,stick_poses[i],1.0,0.05);
    }
  }

  // 接收到钻杆位姿信息则执行
  void topic_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    current_stick_pose_ = *msg;
    if (!is_stick_exist())
    {
      RCLCPP_INFO(LOGGER, "no stick exist.");
      return; // 没有钻杆则停止
    }
    RCLCPP_INFO(LOGGER, "Client sending goal.");
    // 发送场景中第一个stick
    sendGoal();
    const int max_attempts = 3;
    int attempts_time = 0;
    while (attempts_time < max_attempts)
    {
      rclcpp::spin_some(shared_from_this());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      if (is_goal_done() && !is_goal_success())
      { // 目标完成,但是目标失败，进行重试。
        RCLCPP_INFO(LOGGER, "Client resending goal, attempts:%d.", attempts_time);
        attempts_time++;
        sendGoal();
      }
      else if (is_goal_success())
      {
        RCLCPP_INFO(LOGGER, "After success Stick num id is:%d", stick_num_id_);
        break;
      }
    }
  };

  void doPickPlace()
  {
    arrangePickPlace();
    bool visual_available = false;

    while (1)
    {
      if (!is_stick_exist())
      {
        RCLCPP_INFO(LOGGER, "no stick exist.");
        break;
      }
      // 请求目标位姿
      auto request = std::make_shared<rb210_planning_interfaces::srv::PoseReq::Request>();
      request->object_id = stick_num_id_;
      auto future = pos_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS)
      {
        auto pos = future.get()->pose;
        current_stick_pose_ = pos;
        RCLCPP_INFO(this->get_logger(), "结果: [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", pos.position.x, pos.position.x, pos.position.x,
                    pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败");
        return;
      }
      RCLCPP_INFO(LOGGER,"Start planning.");
      // 根据位姿规划
      
      if (!is_stick_exist())
      {
        RCLCPP_INFO(LOGGER, "no stick exist.");
        return; // 没有钻杆则停止
      }
      RCLCPP_INFO(LOGGER, "Client sending goal.");
      sendGoal();
      // 执行与重试机制
      const int max_attempts = 3;
      int attempts_time = 0;
      while (attempts_time < max_attempts)
      {
        rclcpp::spin_some(shared_from_this());
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        if (is_goal_done() && !is_goal_success())
        { // 目标完成,但是目标失败，进行重试。
          attempts_time++;
          if(attempts_time>=max_attempts){
            RCLCPP_INFO(LOGGER, "Goal is done but not success, attempts:%d.", attempts_time);
            break;
          }
          RCLCPP_INFO(LOGGER, "Client resending goal, attempts:%d.", attempts_time);
          sendGoal();
        }
        else if (is_goal_success())
        {
          RCLCPP_INFO(LOGGER, "After success Stick num id is:%d", stick_num_id_);
          break;
        }
      }
      if(attempts_time == max_attempts){
        break;
      }
    }
  }
};

int main(char argc, char *argv[])
{

  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<PathPlanningClient>();

  action_client->doPickPlace();

  rclcpp::shutdown();

  return 0;
}
