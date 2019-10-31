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
    rostest
    std_msgs
    roslint
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    autoware_config_msgs
    autoware_health_checker
    autoware_msgs
    geometry_msgs
    roscpp
    std_msgs
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_filter src/twist_filter.cpp)
target_link_libraries(twist_filter ${catkin_LIBRARIES})
add_dependencies(twist_filter ${catkin_EXPORTED_TARGETS})

install(
  TARGETS twist_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  roslint_add_test()
endif ()
