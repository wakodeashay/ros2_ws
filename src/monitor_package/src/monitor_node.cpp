#include <memory>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode() : Node("goal_setter_node")
  { 
    this->declare_parameter("csv_filename", "odom_data.csv");
    csv_filename_ = this->get_parameter("csv_filename").as_string();

    csv_file_.open(csv_filename_, std::ios::out | std::ios::app);
    
    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_filename_.c_str());
    } else {
      csv_file_ << "lin_x,lin_y,lin_x,ang_x,ang_y,ang_z\n";
    }

    set_init_pose_flag_ = false;
    goal_reached_flag_ = true;

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

    goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("monitor_input_goal", 1, 
      std::bind(&MonitorNode::goal_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, 
      std::bind(&MonitorNode::odom_callback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    set_initial_pose(0.0, 0.0, 0.0);  
  }

~MonitorNode()
{
  if (csv_file_.is_open()) {
    csv_file_.close();
  }
}

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;

  bool set_init_pose_flag_;
  bool goal_reached_flag_;

  std::string csv_filename_;
  std::ofstream csv_file_;

  void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    double x = msg->position.x;
    double y = msg->position.y;

    double orientation_z = msg->orientation.z;
    double orientation_w = msg->orientation.w;

    double heading_rad = atan2(2 * (orientation_w * orientation_z), 1 - 2 * (orientation_z * orientation_z));

    RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f)", x, y);
    send_goal(x, y, heading_rad);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    csv_file_ << msg->twist.twist.linear.x << "," << msg->twist.twist.linear.y << ","  << msg->twist.twist.linear.z << "," 
    << msg->twist.twist.angular.x << "," << msg->twist.twist.angular.y << "," << msg->twist.twist.angular.z << "\n";
  }

  void set_initial_pose(double x, double y, double theta)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = this->now();

    initial_pose.pose.pose.position.x = x;
    initial_pose.pose.pose.position.y = y;
    initial_pose.pose.pose.orientation.w = cos(theta / 2);
    initial_pose.pose.pose.orientation.z = sin(theta / 2);

    initial_pose_pub_->publish(initial_pose);
    RCLCPP_INFO(this->get_logger(), "Initial pose set.");
    // Set init pose setup to true
    set_init_pose_flag_ = true;
  }

  void send_goal(double x, double y, double heading_rad)
  {
    if (!nav_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Navigation server not available");
      return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = cos(heading_rad/2);
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = sin(heading_rad/2);

    show_marker(x, y, heading_rad);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&MonitorNode::result_callback, this, std::placeholders::_1);

    nav_client_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(this->get_logger(), "Goal sent: (%.2f, %.2f)", x, y);
    goal_reached_flag_ = false;
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Reached goal!");
      RCLCPP_INFO(this->get_logger(), "Put Another Goal!");
      goal_reached_flag_ = true;
      // prompt_goal();
    } else {
      RCLCPP_WARN(this->get_logger(), "Goal failed or was canceled.");
    }
  }

  void show_marker(double x, double y, double heading_rad)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.w = cos(heading_rad/2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = sin(heading_rad/2);
    marker.scale.x = 0.5;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = rclcpp::Duration(0, 0);

    marker_pub_->publish(marker);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}