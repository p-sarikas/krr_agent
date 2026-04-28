cmake_minimum_required(VERSION 3.8)
project(krr_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_msgs REQUIRED)

find_package(krr_mirte_skills_msgs REQUIRED)
find_package(ros_typedb_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav2_msgs
  lifecycle_msgs
  plansys2_executor
  plansys2_planner
  plansys2_domain_expert
  plansys2_problem_expert
  plansys2_msgs
  krr_mirte_skills_msgs
  ros_typedb_msgs
  tf2_ros        
  tf2_geometry_msgs
)

set(TARGET_NAME krr_agent)

add_executable(${TARGET_NAME} src/krr_agent.cpp)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  ${dependencies}
)

set(TARGET_NAME task_controller)

add_executable(${TARGET_NAME} src/task_controller.cpp)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  ${dependencies}
)

add_executable(action_move_to_object src/action_move_to_object.cpp)

target_include_directories(action_move_to_object PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_move_to_object PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(action_move_to_object
  ${dependencies}
)

add_executable(action_move_to_drop_location_t1 src/action_move_to_drop_location_t1.cpp)

target_include_directories(action_move_to_drop_location_t1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_move_to_drop_location_t1 PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(action_move_to_drop_location_t1
  ${dependencies}
)

add_executable(action_move_to_drop_location_t2 src/action_move_to_drop_location_t2.cpp)

target_include_directories(action_move_to_drop_location_t2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_move_to_drop_location_t2 PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(action_move_to_drop_location_t2
  ${dependencies}
)

add_executable(action_pick src/action_pick.cpp)

target_include_directories(action_pick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_pick PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(action_pick
  ${dependencies}
)


add_executable(action_place src/action_place.cpp)

target_include_directories(action_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_place PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(action_place
  ${dependencies}
)

add_executable(action_place_back src/action_place_back.cpp)

target_include_directories(action_place_back PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_place_back PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(action_place_back
  ${dependencies}
)

add_executable(action_next_room src/action_next_room.cpp)

target_include_directories(action_next_room PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_next_room PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(action_next_room
  ${dependencies}
)

# for the initial position
add_executable(set_initial_pose src/set_initial_pose.cpp)
ament_target_dependencies(set_initial_pose rclcpp geometry_msgs)


install(PROGRAMS
  scripts/task1_manager.py
  scripts/task2_manager.py
  scripts/task2_multiple_pddl_manager.py
  scripts/task3_manager.py
  scripts/task_manager_base.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${TARGET_NAME}
  action_move_to_object
  action_move_to_drop_location_t1
  action_move_to_drop_location_t2
  action_pick
  action_place
  action_place_back
  action_next_room
  set_initial_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY pddl
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
