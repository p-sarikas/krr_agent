#ifndef KRR_AGENT__ACTION_MOVE_TO_OBJECT_HPP_
#define KRR_AGENT__ACTION_MOVE_TO_OBJECT_HPP_

#include <rclcpp/rclcpp.hpp>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "ros_typedb_msgs/srv/query.hpp"
#include "ros_typedb_msgs/msg/query_result.hpp"
#include "ros_typedb_msgs/msg/result_tree.hpp"

namespace krr_agent
{

using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
using NavigationFeedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

class MoveToObject : public plansys2::ActionExecutorClient
{
public:
  MoveToObject(const std::string & node_name,
    const std::chrono::nanoseconds & rate);

  virtual ~MoveToObject();

private:
  geometry_msgs::msg::Pose current_pos_;

  rclcpp::CallbackGroup::SharedPtr callback_group_action_client_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_cli_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;

  double dist_to_move_;
  bool goal_sent_;

  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2);
 
  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  void do_work();
};

}  // namespace krr_agent

#endif