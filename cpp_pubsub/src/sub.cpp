#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "talker", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
// class OdomSubscriber : public rclcpp::Node
// {
// public:
//   OdomSubscriber()
//       : Node("odom_subscriber")
//   {
//     subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//         "velodyne_points", 10, std::bind(&OdomSubscriber::odom_callback, this, std::placeholders::_1));
//     // adwq_sad_AS_ = this->create_subscription<geometry_msgs::msg::Twist>(
//     //     "velodyne_points", 10, std::bind(&OdomSubscriber::asd_callback, this, std::placeholders::_1));
//   }

// private:
//   void odom_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
//   {
//     RCLCPP_INFO(this->get_logger(), "PointClouds: %ld, %ld", msg->header.stamp.sec, msg->header.stamp.nanosec);
//     // RCLCPP_INFO(this->get_logger(), "Received Odom: [Position: (%.2f, %.2f, %.2f), Orientation: (%.2f, %.2f, %.2f, %.2f)]",
//     //             msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
//     //             msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
//     //             msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//   }
//   void asd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
//   {
//     RCLCPP_INFO(this->get_logger(), "PointClouds: %f, ", msg->linear.x);
//     // RCLCPP_INFO(this->get_logger(), "Received Odom: [Position: (%.2f, %.2f, %.2f), Orientation: (%.2f, %.2f, %.2f, %.2f)]",
//     //             msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
//     //             msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
//     //             msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//   }

//   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr adwq_sad_AS_;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   std::cout << "qwrfreg" << std::endl;
//   rclcpp::spin(std::make_shared<OdomSubscriber>());
//   rclcpp::shutdown();
//   return 0;
// }
