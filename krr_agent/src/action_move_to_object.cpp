#include "krr_agent/action_move_to_object.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace krr_agent
{

  MoveToObject::MoveToObject(const std::string & node_name,
    const std::chrono::nanoseconds & rate)
  : plansys2::ActionExecutorClient(node_name, rate), goal_sent_(false)
  {
  }

  MoveToObject::~MoveToObject()
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MoveToObject::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    callback_group_action_client_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    navigate_cli_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this,
      "navigate_to_pose",
      callback_group_action_client_
    );

    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
     "/amcl_pose",
     10,
     std::bind(&MoveToObject::current_pos_callback, this, _1));

    return plansys2::ActionExecutorClient::on_configure(previous_state);
  }

  void MoveToObject::current_pos_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  double MoveToObject::getDistance(
    const geometry_msgs::msg::Pose & pos1,
    const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MoveToObject::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    // Solo reseteamos la bandera, no bloqueamos a PlanSys2
    goal_sent_ = false;
    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MoveToObject::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    navigate_cli_->async_cancel_all_goals();
    return plansys2::ActionExecutorClient::on_deactivate(previous_state);
  }

  void MoveToObject::do_work()
  {
    // Si ya enviamos el objetivo, simplemente esperamos a que los callbacks de Nav2 hagan su trabajo y llamen a finish()
    if (goal_sent_) {
        return; 
    }

    goal_sent_ = true; // Marcamos que ya empezamos a trabajar

    send_feedback(0.0, "MoveToObject starting");
    RCLCPP_INFO(get_logger(), ">>> do_work: Executing move to object");

    if (!navigate_cli_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), ">>> Waiting for navigation action server...");
      goal_sent_ = false; // Volveremos a intentarlo en el próximo ciclo
      return;
    }

    std::string item_id = get_arguments()[0];
    
    // --- Create temp node + executor for TypeDB ---
    auto temp_node = rclcpp::Node::make_shared(
      "typedb_client_" + std::string(get_name()) + "_" +
      std::to_string(now().nanoseconds()));
    auto temp_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    temp_executor->add_node(temp_node);

    auto typedb_client = temp_node->create_client<ros_typedb_msgs::srv::Query>("/ros_typedb/query");

    if (!typedb_client->wait_for_service(std::chrono::seconds(5))) {
      finish(false, 0.0, "TypeDB query service NOT available");
      return;
    }

    // --- Build and send query ---
    auto request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    request->query_type = ros_typedb_msgs::srv::Query::Request::GET;
    request->query =
      "match "
      "$obj isa item, has id \"" + item_id + "\"; "
      "(located-item: $obj, location: $pose) isa physical-location; "
      "$pose has pos-x $x, has pos-y $y; "
      "get $x, $y;";

    auto future = typedb_client->async_send_request(request);
    auto status = temp_executor->spin_until_future_complete(future, std::chrono::seconds(5));

    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      finish(false, 0.0, "TypeDB service call TIMED OUT");
      return;
    }

    auto response = future.get();
    if (!response->success || response->results.empty()) {
      finish(false, 0.0, "No TypeDB results found");
      return;
    }

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
      finish(false, 0.0, "Could not parse x/y coordinates");
      return;
    }

    // --- Build navigation goal ---
    nav2_msgs::action::NavigateToPose::Goal navigation_goal;
    navigation_goal.pose.header.frame_id = "map";
    navigation_goal.pose.header.stamp = now();
    navigation_goal.pose.pose.position.x = obj_x;
    navigation_goal.pose.pose.position.y = obj_y;
    navigation_goal.pose.pose.position.z = 0.0;
    navigation_goal.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(get_logger(), ">>> Navigating to %s at (%.2f, %.2f)", item_id.c_str(), obj_x, obj_y);

    dist_to_move_ = getDistance(navigation_goal.pose.pose, current_pos_);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move_))),
          "Navigating to object");
      };

    send_goal_options.result_callback = [this](auto) {
      finish(true, 1.0, "MoveToObject completed successfully");
    };

    future_navigation_goal_handle_ = navigate_cli_->async_send_goal(navigation_goal, send_goal_options);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<krr_agent::MoveToObject>(
    "move_to_object", 500ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}