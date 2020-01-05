cmake_minimum_required(VERSION 2.8.3)
project(dp_planner)

find_package(autoware_build_flags REQUIRED)

find_package(OpenCV REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  autoware_can_msgs
  geometry_msgs
  jsk_recognition_msgs
  libwaypoint_follower
  map_file
  op_planner
  op_simu
  op_utility
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
  vector_map_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include include/plannerh
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

add_executable(
  dp_planner
  nodes/dp_planner.cpp
  nodes/dp_planner_core.cpp
  nodes/ROSHelpers.cpp
  nodes/PolygonGenerator.cpp
)

target_link_libraries(
  dp_planner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_dependencies(
  dp_planner
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS dp_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
