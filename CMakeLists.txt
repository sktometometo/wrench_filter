cmake_minimum_required(VERSION 2.8.3)
project(wrench_filter)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  rospy
  std_msgs
)

add_message_files(
  FILES
)

add_service_files(
  FILES
  Calibration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime message_generation rospy std_msgs geometry_msgs
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
