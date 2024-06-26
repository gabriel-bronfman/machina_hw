cmake_minimum_required(VERSION 3.8)
project(sensor_srv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(sensor_server_node
  src/sensor_server_node.cpp)


ament_target_dependencies(sensor_server_node
  robot_interfaces
  rclcpp
  rclcpp_lifecycle
  )
add_executable(sensor_info_publisher
  src/sensor_info_publisher.cpp)

ament_target_dependencies(sensor_info_publisher
  robot_interfaces
  std_msgs
  rclcpp
  rclcpp_lifecycle
  )


install(TARGETS
  sensor_server_node
  sensor_info_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

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
