cmake_minimum_required(VERSION 3.8)
project(krr_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)


install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/add_to_gz_model_path.dsv.in")

ament_package()
