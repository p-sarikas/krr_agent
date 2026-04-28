#include <chrono>
#include <functional>
#include <thread>

#include <krr_agent/krr_agent.hpp>


namespace krr_agent {


/**
 * @brief Constructor
 *
 * @param options node options
 */
KrrAgent::KrrAgent() : Node("krr_agent") {

}
 
} // namespace krr_agent


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<krr_agent::KrrAgent>());
  rclcpp::shutdown();

  return 0;
}
