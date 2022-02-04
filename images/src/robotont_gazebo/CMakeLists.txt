cmake_minimum_required(VERSION 3.0.2)
project(robotont_gazebo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  joy
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    joy
    nav_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch media meshes urdf worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
