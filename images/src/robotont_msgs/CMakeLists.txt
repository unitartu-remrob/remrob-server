cmake_minimum_required(VERSION 2.8.3)
project(robotont_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# Specify message files
add_message_files(
  FILES
  ColorRGB.msg
  LedModulePixel.msg
  LedModuleSegment.msg
  PowerSupply.msg
  LaserScanSplit.msg
)

# Generate message files
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
