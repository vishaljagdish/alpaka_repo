cmake_minimum_required(VERSION 3.8)
project(alpaka_welding_cobot_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add include directories
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(alpaka_welding_cobot_moveit
  src/ml_receiver_node.cpp
  src/ml_receiver_node_driver.cpp
)
target_include_directories(alpaka_welding_cobot_moveit PUBLIC include)
ament_target_dependencies(alpaka_welding_cobot_moveit "rclcpp" "moveit_ros_planning_interface" "tf2_ros" "tf2_geometry_msgs" "geometry_msgs" "sensor_msgs")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
alpaka_welding_cobot_moveit
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
