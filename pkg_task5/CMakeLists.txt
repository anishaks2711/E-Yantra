cmake_minimum_required(VERSION 3.0.2)
project(pkg_task5)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
)
add_message_files(
  FILES
  barcodes.msg
  boxes.msg
)
add_action_files(
  FILES
  belt.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs 
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pkg_task3
  CATKIN_DEPENDS
  geometry_msgs actionlib_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
  message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)