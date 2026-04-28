#ifndef KRR_AGENT__ACTION_PLACE_BACK_HPP_
#define KRR_AGENT__ACTION_PLACE_BACK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "krr_mirte_skills_msgs/srv/place_object.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros_typedb_msgs/srv/query.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace krr_agent
{

class PlaceBack : public plansys2::ActionExecutorClient
{
public:
  PlaceBack(const std::string & node_name,
    const std::chrono::nanoseconds & rate);

  virtual ~PlaceBack();

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_place_client_;
  rclcpp::Client<krr_mirte_skills_msgs::srv::PlaceObject>::SharedPtr place_cli_;
  
  // Turning variables
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  bool moved_aside_;
  bool spin_started_;
  rclcpp::Time start_spin_time_;

  // TF2 variables
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  void do_work();
};

}  // namespace krr_agent

#endif // KRR_AGENT__ACTION_PLACE_BACK_HPP_