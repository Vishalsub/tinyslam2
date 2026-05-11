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
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d)

find_package(PCL REQUIRED COMPONENTS common filters registration)

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
  src/mapping/point_cloud_mapper.cpp
  src/loop_closure/loop_closure_detector.cpp
  src/utils/math_utils.cpp
  src/utils/ros_utils.cpp
)
target_link_libraries(${PROJECT_NAME}_core
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs
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

add_executable(mapping_node src/nodes/mapping_node.cpp)
target_link_libraries(mapping_node ${PROJECT_NAME}_core ${PCL_LIBRARIES})
ament_target_dependencies(mapping_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  tf2 tf2_ros pcl_conversions
)

add_executable(map_saver_node src/nodes/map_saver_node.cpp)
target_link_libraries(map_saver_node ${PROJECT_NAME}_core)
ament_target_dependencies(map_saver_node rclcpp nav_msgs std_srvs)

add_executable(loop_closure_node src/nodes/loop_closure_node.cpp)
target_link_libraries(loop_closure_node ${PROJECT_NAME}_core ${PCL_LIBRARIES})
ament_target_dependencies(loop_closure_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs pcl_conversions)

add_executable(nav2_bridge_node src/nodes/nav2_bridge_node.cpp)
target_link_libraries(nav2_bridge_node ${PROJECT_NAME}_core)
ament_target_dependencies(nav2_bridge_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

install(
  TARGETS
    ${PROJECT_NAME}_core
    sensor_processor_node
    rgbd_odometry_node
    mapping_node
    map_saver_node
    loop_closure_node
    nav2_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Directory names must not end with '/' or CMake installs *contents* into
# share/${PROJECT_NAME}/ and breaks get_package_share_directory()/launch/... paths.
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
