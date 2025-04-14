cmake_minimum_required(VERSION 3.0.2)
project(distance_visualizer)

find_package(catkin
  REQUIRED COMPONENTS
  rospy
  geometry_msgs
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)