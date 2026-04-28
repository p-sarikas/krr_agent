#include "krr_agent/action_move_to_drop_location_t2.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace krr_agent
{

MoveToDropLocation::MoveToDropLocation(const std::string & node_name, const std::chrono::nanoseconds & rate)
: plansys2::ActionExecutorClient(node_name, rate)
{
}

MoveToDropLocation::~MoveToDropLocation()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveToDropLocation::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  callback_group_action_client_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  navigate_cli_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose", callback_group_action_client_);

  pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10, std::bind(&MoveToDropLocation::current_pos_callback, this, _1));

  return plansys2::ActionExecutorClient::on_configure(previous_state);
}

void MoveToDropLocation::current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pos_ = msg->pose.pose;
}

double MoveToDropLocation::getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
{
  return sqrt(
    (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
    (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveToDropLocation::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // --- 1. Get arguments from PDDL ---
  // move_to_drop_location(?i - item ?from - location ?r - room)
  std::string item_id = get_arguments()[0];
  
  send_feedback(0.0, "MoveToDropLocation: Starting perception for " + item_id);

  // Create temp node/executor to handle service calls without blocking
  auto temp_node = rclcpp::Node::make_shared("logic_client_" + std::to_string(now().nanoseconds()));
  auto temp_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  temp_executor->add_node(temp_node);


  // --- 3. KNOWLEDGE REFINEMENT: Update TypeDB ---
  auto db_client = temp_node->create_client<ros_typedb_msgs::srv::Query>("/ros_typedb/query");
  if (!db_client->wait_for_service(std::chrono::seconds(5))) {
    finish(false, 0.0, "TypeDB service not available");
    return CallbackReturn::FAILURE;
  }

  // --- 4. SEMANTIC REASONING: Query Target Location ---
  send_feedback(0.2, "MoveToDropLocation: Reasoning for destination");

  auto query_request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  query_request->query_type = ros_typedb_msgs::srv::Query::Request::GET;
  query_request->query = 
    "match "
    "$i has id \"" + item_id + "\"; "
    "(dropped-item: $i, target-location: $d) isa correct-drop; "
    "$d has id $target_id; "
    "(located-target: $d, location: $p) isa physical-location; "
    "$p has pos-x $x, has pos-y $y; "
    "get $target_id, $x, $y;";

  auto query_future = db_client->async_send_request(query_request);
  if (temp_executor->spin_until_future_complete(query_future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS) {
    finish(false, 0.0, "TypeDB reasoning timed out");
    return CallbackReturn::FAILURE;
  }

  auto query_res = query_future.get();
  if (!query_res->success || query_res->results.empty()) {
    finish(false, 0.0, "No semantic destination found in Knowledge Base");
    return CallbackReturn::FAILURE;
  }

  double target_x = 0.0, target_y = 0.0;
  std::string target_id = "";
  bool parsed = false;

  for (auto & tree : query_res->results) {
    bool got_x = false, got_y = false, got_id = false;
    for (auto & res : tree.results) {
      if (res.type == ros_typedb_msgs::msg::QueryResult::ATTRIBUTE) {
        if (res.attribute.variable_name == "x") { target_x = res.attribute.value.double_value; got_x = true; }
        else if (res.attribute.variable_name == "y") { target_y = res.attribute.value.double_value; got_y = true; }
        else if (res.attribute.variable_name == "target_id") { target_id = res.attribute.value.string_value; got_id = true; }
      }
    }
    if (got_x && got_y && got_id) { parsed = true; break; }
  }

  if (!parsed) {
    finish(false, 0.0, "Failed to parse destination from KB response");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Reasoning Result: %s belongs in %s", item_id.c_str(), target_id.c_str());

  // Mark the object in TypeDB as handled 
  auto update_request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  update_request->query_type = ros_typedb_msgs::srv::Query::Request::UPDATE;
  update_request->query = 
      "match $i isa item, has id \"" + item_id + "\", has handled $h; "
      "delete $i has $h; "
      "insert $i has handled true;";
  auto update_future = db_client->async_send_request(update_request);
  temp_executor->spin_until_future_complete(update_future, std::chrono::seconds(2));

  // --- 5. ACTION EXECUTION: Navigation ---
  nav2_msgs::action::NavigateToPose::Goal navigation_goal;
  navigation_goal.pose.header.frame_id = "map";
  navigation_goal.pose.header.stamp = now();
  navigation_goal.pose.pose.position.x = target_x;
  navigation_goal.pose.pose.position.y = target_y;
  navigation_goal.pose.pose.orientation.w = 1.0;

  dist_to_move_ = getDistance(navigation_goal.pose.pose, current_pos_);

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  
  send_goal_options.feedback_callback = [this](NavigationGoalHandle::SharedPtr, NavigationFeedback feedback) {
    send_feedback(std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move_))),
                  "MoveToDropLocation: Delivering item");
  };

  send_goal_options.result_callback = [this](const NavigationGoalHandle::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      finish(true, 1.0, "Object successfully delivered to semantic location");
    } else {
      finish(false, 0.0, "Navigation failed during delivery");
    }
  };

  future_navigation_goal_handle_ = navigate_cli_->async_send_goal(navigation_goal, send_goal_options);

  return plansys2::ActionExecutorClient::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveToDropLocation::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  navigate_cli_->async_cancel_all_goals();
  return plansys2::ActionExecutorClient::on_deactivate(previous_state);
}

} // namespace krr_agent

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<krr_agent::MoveToDropLocation>("move_to_drop_location_t2", 500ms);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}