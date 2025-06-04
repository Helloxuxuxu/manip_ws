cmake_minimum_required(VERSION 3.8)
project(rb210_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rb210_planning_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  "rclcpp"
  "rclcpp_action"
  "trajectory_msgs"
  "control_msgs"
  "sensor_msgs"
  "rb210_planning_interfaces"
  "tf2"
)

add_executable(rb210_driver src/rb210_driver.cpp)
target_include_directories(rb210_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rb210_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  rb210_driver
  ${dependencies}
)

add_executable(test_driver_node src/test_driver_node.cpp)
target_include_directories(test_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(test_driver_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  test_driver_node
  ${dependencies}
)
target_link_libraries(test_driver_node yaml-cpp)

install(TARGETS rb210_driver
DESTINATION lib/${PROJECT_NAME})

install(TARGETS test_driver_node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)
        
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
