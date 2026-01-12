cmake_minimum_required(VERSION 3.8)
project(aerosimx_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# Find AeroSimX
find_package(AeroSimX QUIET)

# Include directories
include_directories(include)

# AeroSimX Bridge Node
add_executable(aerosimx_bridge_node
  src/aerosimx_bridge_node.cpp
  src/sensor_publishers.cpp
  src/control_subscribers.cpp
  src/tf2_broadcaster.cpp
)

ament_target_dependencies(aerosimx_bridge_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  image_transport
  cv_bridge
)

if(AeroSimX_FOUND)
  target_link_libraries(aerosimx_bridge_node AeroSimX::aerosimx_core)
endif()

# Install
install(TARGETS
  aerosimx_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
