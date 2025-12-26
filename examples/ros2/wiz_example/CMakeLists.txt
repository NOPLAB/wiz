cmake_minimum_required(VERSION 3.14)
project(wiz_example VERSION 0.1.0)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Demo marker publisher node
add_executable(demo_marker_publisher
  src/demo_marker_publisher.cpp
)
ament_target_dependencies(demo_marker_publisher
  rclcpp
  visualization_msgs
  geometry_msgs
)

# Demo pointcloud publisher node
add_executable(demo_pointcloud_publisher
  src/demo_pointcloud_publisher.cpp
)
ament_target_dependencies(demo_pointcloud_publisher
  rclcpp
  sensor_msgs
)

# Demo TF broadcaster node
add_executable(demo_tf_broadcaster
  src/demo_tf_broadcaster.cpp
)
ament_target_dependencies(demo_tf_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
)

# Install executables
install(TARGETS
  demo_marker_publisher
  demo_pointcloud_publisher
  demo_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
