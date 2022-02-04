cmake_minimum_required(VERSION 2.8.3)
project(robotont_demos)

find_package(catkin REQUIRED COMPONENTS
  robotont_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  joy
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    joy
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_joy src/teleop_joy.cpp) 

target_link_libraries(teleop_joy ${catkin_LIBRARIES})


install(TARGETS teleop_joy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY config launch scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
