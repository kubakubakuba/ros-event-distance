cmake_minimum_required(VERSION 3.0.2)
project(dummy_led_publisher)

find_package(catkin
  REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

add_message_files(
  FILES
  LEDMarkers.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs 
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)