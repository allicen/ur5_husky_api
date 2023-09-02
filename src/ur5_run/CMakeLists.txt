cmake_minimum_required(VERSION 3.0.2)
project(ur5_run)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)