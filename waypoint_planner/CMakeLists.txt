cmake_minimum_required(VERSION 2.8.3)
project(waypoint_planner)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(autoware_build_flags REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
    astar_search
    autoware_config_msgs
    autoware_health_checker
    autoware_msgs
    lanelet2_extension
    libwaypoint_follower
    pcl_conversions
    pcl_ros
    roscpp
    roslint
    std_msgs
    tf
    vector_map
)

catkin_package()

# TODO add all codes to roslint
roslint_cpp(src/velocity_set_lanelet2/velocity_set_lanelet2.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  astar_avoid
  src/astar_avoid/astar_avoid.cpp
  src/astar_avoid/astar_avoid_node.cpp
)

target_link_libraries(
  astar_avoid
  ${catkin_LIBRARIES}
)

add_dependencies(
  astar_avoid
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  velocity_set
  src/velocity_set/velocity_set.cpp
  src/velocity_set/velocity_set_path.cpp
  src/velocity_set/velocity_set_info.cpp
  src/velocity_set/libvelocity_set.cpp
)

target_link_libraries(
  velocity_set
  ${catkin_LIBRARIES}
)

add_dependencies(
  velocity_set
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  velocity_set_lanelet2
  src/velocity_set_lanelet2/velocity_set_lanelet2.cpp
  src/velocity_set/velocity_set_path.cpp
  src/velocity_set/velocity_set_info.cpp
  src/velocity_set/libvelocity_set.cpp
)

target_link_libraries(
  velocity_set_lanelet2
  ${catkin_LIBRARIES}
)

add_dependencies(
  velocity_set_lanelet2
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS astar_avoid velocity_set velocity_set_lanelet2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
