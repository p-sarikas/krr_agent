#include "krr_agent/action_pick.hpp"

using namespace std::chrono_literals;

namespace krr_agent
{

Pick::Pick(const std::string & node_name,
  const std::chrono::nanoseconds & rate)
: plansys2::ActionExecutorClient(node_name, rate)
{
}

Pick::~Pick() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Pick::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  return plansys2::ActionExecutorClient::on_configure(previous_state);
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Pick::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Activating Pick action. Transitioning to do_work()...");
  // We simply acknowledge the activation here. The actual work happens in do_work().
  return plansys2::ActionExecutorClient::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Pick::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Deactivating Pick action.");
  return plansys2::ActionExecutorClient::on_deactivate(previous_state);
}

void Pick::do_work()
{
  std::string item_id = get_arguments()[0];

  RCLCPP_INFO(get_logger(), "Starting Pick action for %s", item_id.c_str());

  // All clients live on temp_node so temp_executor can spin their responses
  auto temp_node = rclcpp::Node::make_shared("pick_client_" + std::to_string(now().nanoseconds()));
  auto temp_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  temp_executor->add_node(temp_node);

  // ========================================================================
  // PHASE 1: PHYSICAL PICK
  // ========================================================================
  send_feedback(0.1, "Executing physical pick");

  auto pick_cli = temp_node->create_client<krr_mirte_skills_msgs::srv::PickObject>("/pick_object");
  if (!pick_cli->wait_for_service(std::chrono::seconds(5))) {
    finish(false, 0.0, "Service /pick_object not available");
    return;
  }

  auto pick_req = std::make_shared<krr_mirte_skills_msgs::srv::PickObject::Request>();
  pick_req->object_id = "";
  auto pick_future = pick_cli->async_send_request(pick_req);

  if (temp_executor->spin_until_future_complete(pick_future, std::chrono::seconds(5))
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    finish(false, 0.0, "Physical pick timed out");
    return;
  }

  if (!pick_future.get()->success) {
    finish(false, 0.0, "Failed to physically pick object!");
    return;
  }

  // ========================================================================
  // PHASE 2: PERCEPTION
  // ========================================================================
  send_feedback(0.4, "Identifying object");

  auto info_cli = temp_node->create_client<krr_mirte_skills_msgs::srv::GetObjectInfo>("/get_object_info");
  if (!info_cli->wait_for_service(std::chrono::seconds(5))) {
    finish(false, 0.0, "Service /get_object_info not available");
    return;
  }

  auto info_future = info_cli->async_send_request(
    std::make_shared<krr_mirte_skills_msgs::srv::GetObjectInfo::Request>());

  if (temp_executor->spin_until_future_complete(info_future, std::chrono::seconds(5))
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    finish(false, 0.0, "Object identification timed out");
    return;
  }

  auto info_obj = info_future.get();
  // if (!info_obj->success) {
  //   finish(false, 0.0, "Object identification failed");
  //   return;
  // }
  // # ------- #
  if (!info_obj->success) {
    RCLCPP_INFO(get_logger(),
      "[DEBUG] get_object_info for %s → success=%s, type='%s', attr='%s'",
      item_id.c_str(),
      info_obj->success ? "true" : "false",
      info_obj->object_type.c_str(),
      info_obj->attribute.c_str());
      // Don't hard-fail: just proceed with unknown type
      // The TypeDB update below will safely store it as 'item' under unknown type
  }
  // # ------- #

  std::string t = info_obj->object_type;
  std::string a = info_obj->attribute;
  RCLCPP_INFO(get_logger(), "Perception: %s is a %s with attribute %s",
              item_id.c_str(), t.c_str(), a.c_str());

  // ========================================================================
  // PHASE 3: KNOWLEDGE REFINEMENT
  // ========================================================================
  send_feedback(0.7, "Updating Knowledge Base");

  auto db_client = temp_node->create_client<ros_typedb_msgs::srv::Query>("/ros_typedb/query");
  if (!db_client->wait_for_service(std::chrono::seconds(5))) {
    finish(false, 0.0, "TypeDB service not available");
    return;
  }

  std::string typedb_type;
  std::string attr_segment;

  if (t == "cup" || t == "spoon" || t == "plate" || a == "clean" || a == "dirty") {
    typedb_type = t.empty() ? "tableware" : t;
    if (!a.empty()) attr_segment = ", has cleanliness \"" + a + "\"";
  } else if (t == "beer" || t == "bottle" || a == "full" || a == "empty") {
    typedb_type = (t == "beer") ? "beer" : "bottle";
    if (!a.empty()) attr_segment = ", has fullness \"" + a + "\"";
  } else if (t == "bread" || t == "food" || t == "tissue" || a == "disposable") {
    typedb_type = t.empty() ? "disposable" : t;
  } else if (t == "candle" || t == "tissue-box" || a == "decorative") {
    typedb_type = t.empty() ? "decorative" : t;
  } else if (t == "book" || t == "wooden-cube" || a == "bedroom-related") {
    typedb_type = t.empty() ? "bedroom-related" : t;
  } else if (t == "fidget-spinner" || t == "fidget_spinner" || t == "fidget spinner" 
           || t == "fidget"   // ← this is what /get_object_info actually returns
           || a == "spinner"  // ← and this is the attribute it returns
           || a == "toy") {
    typedb_type = "toy";

  } else {
    typedb_type = "unknown";
  }

  if (typedb_type.empty()) { typedb_type = "unknown"; }

  auto update_request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
  update_request->query_type = ros_typedb_msgs::srv::Query::Request::UPDATE;
  update_request->query =
    "match $i isa item, has id \"" + item_id + "\"; "
    "delete $i isa item; "
    "insert $i isa " + typedb_type + ", has id \"" + item_id + "\"" + attr_segment + ", has handled false;";

  RCLCPP_INFO(get_logger(), "Updating KB: %s", update_request->query.c_str());
  auto update_future = db_client->async_send_request(update_request);
  temp_executor->spin_until_future_complete(update_future, std::chrono::seconds(2));
  RCLCPP_INFO(get_logger(), "KB updated successfully.");
  finish(true, 1.0, "Picked and processed object successfully!");

}

}  // namespace krr_agent

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<krr_agent::Pick>("pick", 500ms);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}