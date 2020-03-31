cmake_minimum_required(VERSION 2.8.3)
project(twist_filter)

find_package(autoware_build_flags REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
    autoware_config_msgs
    autoware_health_checker
    autoware_msgs
    geometry_msgs
    roscpp
    roslint
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    autoware_config_msgs
    autoware_msgs
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  src/twist_filter_node_main.cpp
  src/twist_filter_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)

add_library(
  ${PROJECT_NAME}_core
  src/twist_filter.cpp
)
target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_core ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_twist_filter_node test/test_twist_filter_node.test test/src/test_twist_filter_node.cpp src/twist_filter_node.cpp src/twist_filter.cpp)
  add_dependencies(test_twist_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME} ${PROJECT_NAME}_core)
  target_link_libraries(test_twist_filter_node ${catkin_LIBRARIES} ${PROJECT_NAME}_core)
  roslint_add_test()
endif ()
