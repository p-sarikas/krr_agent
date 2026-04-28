#include "krr_agent/action_next_room.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace krr_agent
{

NextRoom::NextRoom(const std::string & node_name,
  const std::chrono::nanoseconds & rate)
: plansys2::ActionExecutorClient(node_name, rate)
{
}

NextRoom::~NextRoom()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
NextRoom::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  callback_group_nav_client_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  navigate_cli_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose", callback_group_nav_client_);

  pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&NextRoom::current_pos_callback, this, _1));

  return plansys2::ActionExecutorClient::on_configure(previous_state);
}

void NextRoom::current_pos_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pos_ = msg->pose.pose;
}

double NextRoom::getDistance(
  const geometry_msgs::msg::Pose & pos1,
  const geometry_msgs::msg::Pose & pos2)
{
  return sqrt(
    (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
    (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
NextRoom::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  send_feedback(0.0, "NextRoom starting");

  // --- Wait for navigation server ---
  while (!navigate_cli_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_INFO(get_logger(), ">>> Waiting for navigation action server...");
  }
  RCLCPP_INFO(get_logger(), ">>> Nav server ready");


  // next_room parameters: (?r1 ?r2 - room ?l1 ?l2 - location)
  std::string to_room    = get_arguments()[1];
  std::string target_loc = get_arguments()[3];

  RCLCPP_INFO(get_logger(), "Moving to %s via %s",
    to_room.c_str(), target_loc.c_str());

  // --- Create temp node + executor to avoid deadlocks ---
  auto temp_node = rclcpp::Node::make_shared(
    "typedb_client_" + std::string(get_name()) + "_" +
    std::to_string(now().nanoseconds()));
  auto temp_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  temp_executor->add_node(temp_node);

  auto typedb_client = temp_node->create_client<ros_typedb_msgs::srv::Query>(
    "/ros_typedb/query");

  // --- Wait for TypeDB service ---
  RCLCPP_INFO(get_logger(), ">>> Waiting for TypeDB service...");
  if (!typedb_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), ">>> TypeDB query service NOT available");
    finish(false, 0.0, "TypeDB query service not available");
    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }
  RCLCPP_INFO(get_logger(), ">>> TypeDB service found");

  // --- Build and send query ---
  auto request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  request->query_type = ros_typedb_msgs::srv::Query::Request::GET;
  request->query =
    "match $loc isa scan-location, has id \"" + target_loc + "\"; "
    "$p isa pose, has pos-x $x, has pos-y $y, has pos-z $z, "
    "has rot-x $rx, has rot-y $ry, has rot-z $rz, has rot-w $rw; "
    "(located-target: $loc, location: $p) isa physical-location; "
    "get $x, $y, $z, $rx, $ry, $rz, $rw;";

  RCLCPP_INFO(get_logger(), ">>> Sending query: %s", request->query.c_str());

  auto future = typedb_client->async_send_request(request);
  auto status = temp_executor->spin_until_future_complete(
    future, std::chrono::seconds(5));

  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), ">>> TypeDB service call TIMED OUT");
    finish(false, 0.0, "TypeDB query timed out");
    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }
  RCLCPP_INFO(get_logger(), ">>> Got TypeDB response");

  auto response = future.get();
  RCLCPP_INFO(get_logger(), ">>> success=%d, results.size()=%zu",
    response->success, response->results.size());

  if (!response->success || response->results.empty()) {
    RCLCPP_ERROR(get_logger(), ">>> No TypeDB results for '%s'", target_loc.c_str());
    finish(false, 0.0, "Waypoint not found in database");
    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  // --- Parse pos-x and pos-y from results ---
  double obj_x = 0.0, obj_y = 0.0;
  bool got_x = false, got_y = false;

  for (auto & result_tree : response->results) {
    for (auto & result : result_tree.results) {
      if (result.type == ros_typedb_msgs::msg::QueryResult::ATTRIBUTE) {
        const auto & attr = result.attribute;
        if (attr.variable_name == "x") {
          obj_x = attr.value.double_value;
          got_x = true;
        } else if (attr.variable_name == "y") {
          obj_y = attr.value.double_value;
          got_y = true;
        }
      }
    }
    if (got_x && got_y) break;
  }

  if (!got_x || !got_y) {
    RCLCPP_ERROR(get_logger(), ">>> Could not parse x/y for '%s'", target_loc.c_str());
    finish(false, 0.0, "Failed to parse coordinates from TypeDB");
    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  RCLCPP_INFO(get_logger(), ">>> Found '%s' at (%.2f, %.2f)",
    target_loc.c_str(), obj_x, obj_y);


  // --- Build navigation goal ---
  nav2_msgs::action::NavigateToPose::Goal navigation_goal;
  navigation_goal.pose.header.frame_id = "map";
  navigation_goal.pose.header.stamp = now();
  navigation_goal.pose.pose.position.x = obj_x;
  navigation_goal.pose.pose.position.y = obj_y;
  navigation_goal.pose.pose.position.z = 0.0;
  navigation_goal.pose.pose.orientation.w = 1.0;

  RCLCPP_INFO(get_logger(), ">>> Navigating to %s at (%.2f, %.2f)",
    target_loc.c_str(), obj_x, obj_y);

  dist_to_move_ = getDistance(navigation_goal.pose.pose, current_pos_);

  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.feedback_callback = [this](
    NavigationGoalHandle::SharedPtr,
    NavigationFeedback feedback) {
      send_feedback(
        std::min(1.0, std::max(0.0,
          1.0 - (feedback->distance_remaining / dist_to_move_))),
        "NextRoom navigating");
    };

  send_goal_options.result_callback = [this, to_room](auto) {
    RCLCPP_INFO(get_logger(), "Arrived at %s.", to_room.c_str());
    finish(true, 1.0, "NextRoom completed");
  };

  future_navigation_goal_handle_ =
    navigate_cli_->async_send_goal(navigation_goal, send_goal_options);

  return plansys2::ActionExecutorClient::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
NextRoom::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  navigate_cli_->async_cancel_all_goals();
  return plansys2::ActionExecutorClient::on_deactivate(previous_state);
}

}  // namespace krr_agent

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<krr_agent::NextRoom>("next_room", 500ms);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}