#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

namespace krr_agent {

class TaskController : public rclcpp::Node {

public:

  TaskController(const std::string & node_name);
  virtual ~TaskController();

protected:

  rclcpp::CallbackGroup::SharedPtr step_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  
  bool start_planning_ = false;
  bool is_executing_ = false;

  void trigger_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void execute_plan();
  void step();
  void finish_controlling();

};

}
