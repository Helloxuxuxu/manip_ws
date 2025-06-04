/**
 * @file sim_visual_detaction_node.cpp
 * @author wenyu (you@domain.com)
 * @brief 用于模拟视觉检测模块提供检测位姿
 * @version 0.1
 * @date 2025-04-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include <std_msgs/msg/int32.hpp>
#include "rb210_planning_interfaces/srv/pose_req.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class PosePublisherNode : public rclcpp::Node
{
public:
  PosePublisherNode()
      : Node("sim_visual_detaction_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("stick_pose_detected", 10);
    declare_parameter<bool>("visual_available", false); // 声明一个当前stick的全局变量
    declare_parameter<int>("stick_num_id", 0); // 声明一个当前stick的全局变量
    RCLCPP_INFO(get_logger(),"parameters_client started.");
    subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
      "stick_req", 10, std::bind(&PosePublisherNode::publish_pose, this, std::placeholders::_1));
    stick_srv_ = this->create_service<rb210_planning_interfaces::srv::PoseReq>(
      "get_stick_pose", std::bind(&PosePublisherNode::handle_service, this, _1, _2));
    stick_srv_EyeInHand_ = this->create_service<rb210_planning_interfaces::srv::PoseReq>(
      "get_stick_pose_EyeInHand", std::bind(&PosePublisherNode::handle_service_EyeInHand, this, _1, _2));
    
      // 创建目标请求
    geometry_msgs::msg::Pose pose1;
    pose1.orientation.x = -0.7071;
    pose1.orientation.y = 0;
    pose1.orientation.z = 0;
    pose1.orientation.w = 0.7071;
    pose1.position.x = 1.2;
    pose1.position.y = 1.2;
    pose1.position.z = 0.1 - 0.686;
    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.x = 1.4;
    pose2.orientation.x = -0.58779;
    pose2.orientation.y = -0.39311;
    pose2.orientation.z = 0.39299;
    pose2.orientation.w = 0.58782;
    pose2.position.x = 0.6;
    pose2.position.y = 1.6;
    pose2.position.z = 0.1 - 0.686;
    geometry_msgs::msg::Pose pose3 = pose1;
    pose3.position.x = 1.2;
    pose3.orientation.x = -0.5318;
    pose3.orientation.y = -0.466;
    pose3.orientation.z = 0.46597;
    pose3.orientation.w = 0.53189;
    pose3.position.x = 0.6;
    pose3.position.y = 1.3;
    pose3.position.z = 0.1 - 0.686;

    pick_poses.push_back(pose1);
    pick_poses.push_back(pose2);
    pick_poses.push_back(pose3);
    set_parameter(rclcpp::Parameter("visual_available", true));
  }
  ~PosePublisherNode(){
    set_parameter(rclcpp::Parameter("visual_available", false));
  }

private:
  std::vector<geometry_msgs::msg::Pose> pick_poses;
  int stick_num_id_ = 0;

  void publish_pose(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int stick_id = msg->data;
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 0.0;
    if(stick_id < pick_poses.size()){
      pose = pick_poses[stick_id];
    }
    publisher_->publish(pose);
    // RCLCPP_INFO(this->get_logger(), "Published pose: [%.2f, %.2f, %.2f]",
    //             pose.position.x, pose.position.y, pose.position.z);

  }

  void handle_service(
    const std::shared_ptr<rb210_planning_interfaces::srv::PoseReq::Request> request,
    std::shared_ptr<rb210_planning_interfaces::srv::PoseReq::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request for target ID: %d", request->object_id);
    stick_id_ = request->object_id;
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 0.0;
    if(stick_id_ < pick_poses.size()){
      pose = pick_poses[stick_id_];
    }
    response->pose = pose;
  }

  void handle_service_EyeInHand(
    const std::shared_ptr<rb210_planning_interfaces::srv::PoseReq::Request> request,
    std::shared_ptr<rb210_planning_interfaces::srv::PoseReq::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request EyeInHand, return id: %d", stick_id_);
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 0.0;
    if(stick_id_ < pick_poses.size()){
      pose = pick_poses[stick_id_];
    }
    response->pose = pose;
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
  rclcpp::Service<rb210_planning_interfaces::srv::PoseReq>::SharedPtr stick_srv_;
  rclcpp::Service<rb210_planning_interfaces::srv::PoseReq>::SharedPtr stick_srv_EyeInHand_;
  rclcpp::TimerBase::SharedPtr timer_;
  int stick_id_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
