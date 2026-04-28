#include <ctime>
#include <std_srvs/srv/trigger.hpp>
#include "krr_agent/task_controller.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace krr_agent
{

  TaskController::TaskController(const std::string & node_name)
  : rclcpp::Node(node_name)
{
  domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  executor_client_ = std::make_shared<plansys2::ExecutorClient>("task_executor");

  trigger_server_ = this->create_service<std_srvs::srv::Trigger>(
    "/start_task_planning",
    std::bind(&TaskController::trigger_callback, this, _1, _2));
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/task_status", 10);

  step_timer_cb_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  // TODO: create parameter for timer rate?
  step_timer_ = this->create_wall_timer(
    1s, std::bind(&TaskController::step, this), step_timer_cb_group_);
  
  RCLCPP_INFO(this->get_logger(), "TaskController initialized. Waiting for trigger...");
}

TaskController::~TaskController()
{
}

void TaskController::trigger_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!is_executing_) {
    start_planning_ = true;
    response->success = true;
    response->message = "Planning triggered successfully.";
    RCLCPP_INFO(this->get_logger(), "Trigger received from Python!");
  } else {
    response->success = false;
    response->message = "Already planning or executing.";
    RCLCPP_WARN(this->get_logger(), "Trigger ignored. Currently executing.");
  }
}

void TaskController::execute_plan(){
  // Compute the plan
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto plan = planner_client_->getPlan(domain, problem);

  if (!plan.has_value()) {
    std::cout << "Could not find plan to reach goal " << std::endl;
    is_executing_ = false;

    std_msgs::msg::String msg;
    msg.data = "FAILED";
    status_pub_->publish(msg);
    
    return;
  }

  std::cout << "Selected plan: " << std::endl;
  for (auto item : plan->items){
    RCLCPP_INFO(this->get_logger(), "  Action: '%s'", item.action.c_str());
  }
  // Execute the plan
  executor_client_->start_plan_execution(plan.value());
}

void TaskController::finish_controlling(){
  this->executor_client_->cancel_plan_execution();
  this->is_executing_ = false;
  this->start_planning_ = false;
}

void TaskController::step(){
  if (start_planning_){
    start_planning_ = false;
    is_executing_ = true;
    this->execute_plan();
    return;
  }

  if (!is_executing_) {
    return; 
  }

  if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
    if (executor_client_->getResult().value().success) {
      RCLCPP_INFO(this->get_logger(), "Plan execution finished with success!");

      // Publish success status
      std_msgs::msg::String msg;
      msg.data = "SUCCESS";
      status_pub_->publish(msg);

      this->finish_controlling();
      return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Replanning!");
        this->execute_plan();
        return;
    }
  }

  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      std::string error_str_ = "[" + action_feedback.action + "] finished with error: " + action_feedback.message_status;
      RCLCPP_ERROR(this->get_logger(), error_str_.c_str());
      break;
    }

    std::string arguments_str_ = " ";
    for (const auto & arguments: action_feedback.arguments){
      arguments_str_ += arguments + " ";
    }
    std::string feedback_str_ = "[" + action_feedback.action + arguments_str_ +
      std::to_string(action_feedback.completion * 100.0) + "%]";
    RCLCPP_INFO(this->get_logger(), feedback_str_.c_str());
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<krr_agent::TaskController>(
    "task_controller");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
}