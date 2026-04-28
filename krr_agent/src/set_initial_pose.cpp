#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <chrono>

using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node
{
public:
  InitialPosePublisher()
  : Node("initial_pose_publisher")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);

    timer_ = this->create_wall_timer(
      2s, std::bind(&InitialPosePublisher::publish_pose, this));
  }

private:
  void publish_pose()
  {
    if (published_) return;

    geometry_msgs::msg::PoseWithCovarianceStamped msg;

    msg.header.frame_id = "map";
    msg.header.stamp = now();

    // POSITION (Hardcoded: x=0.0, y=0.0, z=0.0)
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;

    // ORIENTATION (Hardcoded: x=0.0, y=0.0, z=0.0, w=1.0)
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;

    // COVARIANCE (same as RViz default)
    msg.pose.covariance = {
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
    };

    pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Initial pose published: [-1.0, 0.0, 0.0] with orientation [0.0, 0.0, 0.0, 1.0]");

    published_ = true;
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool published_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitialPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}