cmake_minimum_required(VERSION 3.0.2)
project(radar_odometry)

set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf_conversions
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(Backward REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    visualization_msgs
    tf_conversions
)

# include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_library(
  ${PROJECT_NAME}
  src/radar_odometry/radar_imu_fusion.cpp
  src/radar_odometry/transform_fusion.cpp
)

set(GTSAM_LIBRARY_DIRS "${GTSAM_DIR}/../..")
link_directories(${GTSAM_LIBRARY_DIRS})

# IMU Preintegration
add_executable(${PROJECT_NAME}_radar_imu_fusion_node src/radar_odometry/radar_imu_fusion_node.cpp)
target_link_libraries(${PROJECT_NAME}_radar_imu_fusion_node Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME} gtsam gtsam_unstable Backward::Backward)
