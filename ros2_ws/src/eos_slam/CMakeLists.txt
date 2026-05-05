cmake_minimum_required(VERSION 3.14)
project(eos_slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d)

find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}_core
  src/sensor_processors/rgb_processor.cpp
  src/sensor_processors/depth_processor.cpp
  src/sensor_processors/lidar_processor.cpp
  src/visual_odometry/rgbd_odometry.cpp
  src/utils/math_utils.cpp
  src/utils/ros_utils.cpp
)
target_link_libraries(${PROJECT_NAME}_core
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  cv_bridge image_transport tf2 tf2_ros pcl_conversions
)

add_executable(sensor_processor_node src/nodes/sensor_processor_node.cpp)
target_link_libraries(sensor_processor_node ${PROJECT_NAME}_core ${PCL_LIBRARIES})
ament_target_dependencies(sensor_processor_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  cv_bridge image_transport tf2 tf2_ros pcl_conversions
)

add_executable(rgbd_odometry_node src/nodes/rgbd_odometry_node.cpp)
target_link_libraries(rgbd_odometry_node ${PROJECT_NAME}_core)
ament_target_dependencies(rgbd_odometry_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  cv_bridge image_transport tf2 tf2_ros pcl_conversions
)

install(
  TARGETS ${PROJECT_NAME}_core sensor_processor_node rgbd_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/ launch/ rviz/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
