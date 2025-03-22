cmake_minimum_required(VERSION 3.0.2)
project(ackermann_nav)

# 添加TinyXML2依赖
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  ackermann_msgs
  serial
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
  ${TINYXML2_INCLUDE_DIRS}  # 添加TinyXML2头文件路径
)

add_library(odom_updater src/odom_updater.cpp)
target_link_libraries(odom_updater ${catkin_LIBRARIES})

add_executable(ackermann_controller src/ackermann_controller.cpp)
target_link_libraries(ackermann_controller odom_updater ${catkin_LIBRARIES})

add_executable(path_follower src/path_follower.cpp)
target_link_libraries(path_follower ${catkin_LIBRARIES})

add_executable(move_distance src/move_distance.cpp)
target_link_libraries(move_distance 
  ${catkin_LIBRARIES}
  ${TINYXML2_LIBRARIES}  # 添加TinyXML2库链接
)

add_executable(motor_test src/motor_test.cpp)
target_link_libraries(motor_test ${catkin_LIBRARIES})

add_executable(arduino_interface src/arduino_interface.cpp)
target_link_libraries(arduino_interface ${catkin_LIBRARIES})

install(TARGETS
  ackermann_controller
  path_follower
  move_distance
  motor_test
  arduino_interface
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)