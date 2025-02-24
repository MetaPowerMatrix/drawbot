cmake_minimum_required(VERSION 3.0.2)
project(ackermann_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  ackermann_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    ackermann_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ackermann_controller src/ackermann_controller.cpp)
target_link_libraries(ackermann_controller ${catkin_LIBRARIES})

add_executable(path_follower src/path_follower.cpp)
target_link_libraries(path_follower ${catkin_LIBRARIES})

install(TARGETS
  ackermann_controller
  path_follower
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)