#ifndef KRR_AGENT__ACTION_PICK_HPP_
#define KRR_AGENT__ACTION_PICK_HPP_

#include <rclcpp/rclcpp.hpp>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "krr_mirte_skills_msgs/srv/pick_object.hpp"
#include "krr_mirte_skills_msgs/srv/get_object_info.hpp"
#include "ros_typedb_msgs/srv/query.hpp"

#include <memory>
#include <string>
#include <chrono>

namespace krr_agent
{

class Pick : public plansys2::ActionExecutorClient
{
public:
  Pick(const std::string & node_name,
    const std::chrono::nanoseconds & rate);
  virtual ~Pick();

private:
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