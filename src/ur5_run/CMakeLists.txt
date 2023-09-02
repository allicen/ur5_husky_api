cmake_minimum_required(VERSION 3.0.2)
project(ur5_run)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)