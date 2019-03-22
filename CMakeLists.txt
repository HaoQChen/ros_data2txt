cmake_minimum_required(VERSION 3.0.0)
project(ros_data2txt)

add_definitions(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  tf
  geometry_msgs
  nav_msgs
)
##FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)

catkin_package(
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(imu2txt src/imu2txt.cpp)
target_link_libraries(imu2txt
  ${catkin_LIBRARIES}
)

add_executable(odom2txt src/odom2txt.cpp)
target_link_libraries(odom2txt
  ${catkin_LIBRARIES}
)

add_executable(laser_scan2txt src/laser_scan2txt.cpp)
target_link_libraries(laser_scan2txt
  ${catkin_LIBRARIES}
)