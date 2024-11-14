cmake_minimum_required(VERSION 3.0.2)
project(motion_segment_4d)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Backward REQUIRED)
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
)

add_executable(${PROJECT_NAME}_node
  src/motion_segment_4d_node.cpp
  src/motion_segment_4d.cpp
  src/conti_callback.cpp
)

set_target_properties(${PROJECT_NAME}_node
  PROPERTIES
    CXX_STANDARD 17
)
target_compile_options(${PROJECT_NAME}_node
  PRIVATE
    -Werror
    -Wall
    -Wno-literal-suffix
)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  Backward::Backward
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/ego_velocity_monitor.py
  scripts/curve_fitting_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
