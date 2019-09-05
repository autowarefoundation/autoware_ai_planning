cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        tf
        tf2
        geometry_msgs
        autoware_msgs
        autoware_config_msgs
        amathutils_lib
        autoware_health_checker
        autoware_build_flags
        rostest
        rosunit
        libwaypoint_follower
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES libwaypoint_follower
        CATKIN_DEPENDS roscpp
        std_msgs
        tf
        tf2
        geometry_msgs
        autoware_msgs
        autoware_config_msgs
        amathutils_lib
        autoware_health_checker
        libwaypoint_follower
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${autoware_config_msgs_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


add_executable(pure_pursuit
        src/pure_pursuit_node.cpp
        src/pure_pursuit.cpp
        src/pure_pursuit_core.cpp
        src/pure_pursuit_viz.cpp
        )
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})
add_dependencies(pure_pursuit ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(TARGETS pure_pursuit
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    
    add_rostest_gtest(test-pure_pursuit test/test_pure_pursuit.test test/src/test_pure_pursuit.cpp src/pure_pursuit_core.cpp 
                      src/pure_pursuit.cpp src/pure_pursuit_viz.cpp)
    add_dependencies(test-pure_pursuit ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test-pure_pursuit ${catkin_LIBRARIES})

endif ()
