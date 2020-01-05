cmake_minimum_required(VERSION 2.8.3)
project(twist_gate)

find_package(autoware_build_flags REQUIRED)
find_package(
  catkin REQUIRED COMPONENTS
    autoware_config_msgs
    autoware_health_checker
    autoware_msgs
    geometry_msgs
    ros_observer
    roscpp
    roslint
    std_msgs
    tablet_socket_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
set(ROSLINT_CPP_OPTS "--filter=-build/c++11")

roslint_cpp()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_gate src/twist_gate_node.cpp src/twist_gate.cpp)
target_link_libraries(twist_gate ${catkin_LIBRARIES})
add_dependencies(twist_gate ${catkin_EXPORTED_TARGETS})

install(
  TARGETS twist_gate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    test-twist_gate
    test/test_twist_gate.test
    test/src/test_twist_gate.cpp
    src/twist_gate.cpp
  )
  add_dependencies(test-twist_gate ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test-twist_gate ${catkin_LIBRARIES})
endif()
