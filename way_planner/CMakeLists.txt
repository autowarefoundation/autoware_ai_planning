cmake_minimum_required(VERSION 2.8.3)
project(way_planner)

find_package(autoware_build_flags REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
    autoware_can_msgs
    geometry_msgs
    jsk_recognition_msgs
    libwaypoint_follower
    message_generation
    op_planner
    op_simu
    op_utility
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    std_msgs
    tf
    vector_map_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES way_planner
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  include/plannerh
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  way_planner
  nodes/way_planner.cpp
  nodes/way_planner_core.cpp
  nodes/ROSHelpers.cpp
  nodes/SocketServer.cpp
)

target_link_libraries(way_planner ${catkin_LIBRARIES})

install(
  TARGETS way_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
        
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
