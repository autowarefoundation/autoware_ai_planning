cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

find_package(autoware_build_flags REQUIRED)
find_package(
  catkin REQUIRED COMPONENTS
    amathutils_lib
    autoware_config_msgs
    autoware_health_checker
    autoware_msgs
    geometry_msgs
    libwaypoint_follower
    roscpp
    roslint
    rostest
    rosunit
    std_msgs
    tf
    tf2
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

add_executable(
  pure_pursuit
  src/pure_pursuit_node.cpp
  src/pure_pursuit.cpp
  src/pure_pursuit_core.cpp
  src/pure_pursuit_viz.cpp
)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})
add_dependencies(pure_pursuit ${catkin_EXPORTED_TARGETS})

install(
  TARGETS pure_pursuit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    test-pure_pursuit
    test/test_pure_pursuit.test
    test/src/test_pure_pursuit.cpp
    src/pure_pursuit_core.cpp
    src/pure_pursuit.cpp src/pure_pursuit_viz.cpp
  )
  add_dependencies(test-pure_pursuit ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test-pure_pursuit ${catkin_LIBRARIES})

  roslint_add_test()
endif()
