#include "krr_agent/action_place_back.hpp"
#include <tf2/exceptions.h>
#include "ros_typedb_msgs/srv/query.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

namespace krr_agent
{

  PlaceBack::PlaceBack(const std::string & node_name,
    const std::chrono::nanoseconds & rate)
  : plansys2::ActionExecutorClient(node_name, rate), 
    moved_aside_(false), 
    spin_started_(false)
  {
  }

  PlaceBack::~PlaceBack()
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PlaceBack::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    callback_group_place_client_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    place_cli_ = this->create_client<krr_mirte_skills_msgs::srv::PlaceObject>(
        "place_object",
        rmw_qos_profile_services_default,
        callback_group_place_client_);

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    return plansys2::ActionExecutorClient::on_configure(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  PlaceBack::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    moved_aside_ = false;
    spin_started_ = false;
    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void PlaceBack::do_work(){
    // --- STEP 1: MOVE ASIDE ---
    if (!moved_aside_) {
        send_feedback(0.2, "Moving aside to unblock door...");
        
        if (!spin_started_) {
            start_spin_time_ = this->now();
            spin_started_ = true;
        }

        geometry_msgs::msg::Twist cmd;
        
        if ((this->now() - start_spin_time_).seconds() < 2.0) {
            cmd.angular.z = -1.0; 
            vel_pub_->publish(cmd);
            return; 
        } 
        else {
            cmd.angular.z = 0.0;
            vel_pub_->publish(cmd);
            moved_aside_ = true; 
        }
    }

    
    auto temp_node = rclcpp::Node::make_shared("place_back_temp_" + std::to_string(now().nanoseconds()));
    auto temp_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    temp_executor->add_node(temp_node);

    // --- STEP 2: PLACE OBJECT ---
    if(place_cli_->service_is_ready()){
        send_feedback(0.7, "Dropping obstacle...");
        auto request = std::make_shared<krr_mirte_skills_msgs::srv::PlaceObject::Request>();
        auto place_result_ = place_cli_->async_send_request(request);

        // Wait for the physical action result
        if (place_result_.wait_for(1s) == std::future_status::ready)
        {
            auto result_ = place_result_.get();
            if(!result_->success){
                finish(false, 1.0, "Failed to place object!");
                return;
            }

            if(result_->success){

              // --- STEP 3: KNOWLEDGE UPDATE ---
              send_feedback(0.9, "Updating Knowledge Base");
              std::string item_id = get_arguments()[0];

              // Get current pose of the robot
              double new_x = 0.0;
              double new_y = 0.0;

              try {
                  // Cuidado aquí: "base_link" o "base_footprint" dependiendo de tu robot
                  geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
                      "map", "Gripper", tf2::TimePointZero);
                  new_x = transformStamped.transform.translation.x;
                  new_y = transformStamped.transform.translation.y;
              } catch (tf2::TransformException &ex) {
                  RCLCPP_WARN(get_logger(), "Could not get robot pose: %s", ex.what());
              }

              auto db_client = temp_node->create_client<ros_typedb_msgs::srv::Query>("/ros_typedb/query");
              if (!db_client->wait_for_service(std::chrono::seconds(5))) {
                finish(false, 0.0, "TypeDB service not available for place_back");
                return;
              }

              auto update_request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
              update_request->query_type = ros_typedb_msgs::srv::Query::Request::INSERT;
              
              std::stringstream tql;
              tql << "match\n"
                  << "  $obj has id \"" << item_id << "\";\n"
                  << "insert\n"
                  << "  $new_p isa pose, has pos-x " << new_x << ", has pos-y " << new_y << ", has pos-z 0.0, has rot-x 0.0, has rot-y 0.0, has rot-z 0.0, has rot-w 1.0;\n"
                  << "  (located-item: $obj, location: $new_p) isa physical-location;";

              update_request->query = tql.str();

              RCLCPP_INFO(get_logger(), "Updating obstacle KB: %s", update_request->query.c_str());
              
              auto update_future = db_client->async_send_request(update_request);
              temp_executor->spin_until_future_complete(update_future, std::chrono::seconds(2));
              
              RCLCPP_INFO(get_logger(), "KB updated successfully after place_back.");

              finish(true, 1.0, "Placed object safely aside and updated KB!");
            }
        }
    }
  }

} // namespace krr_agent

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<krr_agent::PlaceBack>("place_back", 500ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}