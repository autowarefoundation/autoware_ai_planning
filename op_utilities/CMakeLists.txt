cmake_minimum_required(VERSION 2.8.3)
project(op_utilities)

find_package(autoware_build_flags REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS    
    autoware_msgs
    geometry_msgs    
    map_file
    op_planner
    op_ros_helpers
    op_utility
    pcl_conversions
    rosbag
    roscpp
    vector_map_msgs
)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(X11 REQUIRED)
find_package(Threads REQUIRED)
#find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

add_executable(
  op_pose2tf
  nodes/op_pose2tf/op_pose2tf.cpp
  nodes/op_pose2tf/op_pose2tf_core.cpp
)
target_link_libraries(op_pose2tf ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(
  op_data_logger
  nodes/op_data_logger/op_data_logger.cpp
  nodes/op_data_logger/op_data_logger_core.cpp
)
target_link_libraries(op_data_logger ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(
  op_bag_player 
  nodes/op_bag_player/op_bag_player.cpp
  nodes/op_bag_player/op_bag_player_core.cpp
  nodes/op_bag_player/DrawObjBase.cpp
  nodes/op_bag_player/DrawingHelpers.cpp
  nodes/op_bag_player/MainWindowWrapper.cpp
  nodes/op_bag_player/BagTopicPlayer.cpp
)

target_link_libraries(
  op_bag_player  
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${X11_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

add_dependencies(
  op_pose2tf
  op_data_logger
  op_bag_player
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    op_pose2tf
    op_data_logger
    op_bag_player
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
        
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
