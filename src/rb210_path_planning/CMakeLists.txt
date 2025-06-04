cmake_minimum_required(VERSION 3.8)
project(rb210_path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rb210_planning_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# library
add_library(path_planner SHARED src/path_planner.cpp)
# 设置头文件路径
target_include_directories(path_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(path_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
set(dependencies
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "action_msgs"
  "control_msgs"
  "geometry_msgs"
  "moveit_visual_tools"
  "moveit_core"
  "moveit_ros_planning"
  "pluginlib"
  "moveit_msgs"
  "rb210_planning_interfaces"
)
ament_target_dependencies(path_planner
  ${dependencies}
)
ament_export_libraries(path_planner)

# executable
add_executable(path_planning_server src/path_planning_server.cpp)

target_include_directories(path_planning_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(path_planning_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(
  path_planning_server
  path_planner
)
# client
add_executable(path_planning_client src/path_planning_client.cpp)
target_include_directories(path_planning_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(path_planning_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  path_planning_client
  ${dependencies}
)
# sim_visual_detection_node
add_executable(sim_visual_detection_node src/sim_visual_detection_node.cpp)
target_include_directories(sim_visual_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sim_visual_detection_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  sim_visual_detection_node
  ${dependencies}
)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

install(TARGETS path_planner
LIBRARY DESTINATION lib)

install(TARGETS path_planning_server
DESTINATION lib/${PROJECT_NAME})

install(TARGETS path_planning_client
DESTINATION lib/${PROJECT_NAME})

install(TARGETS sim_visual_detection_node
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
