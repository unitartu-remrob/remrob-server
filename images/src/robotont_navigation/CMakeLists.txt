cmake_minimum_required(VERSION 3.0.2)
project(robotont_navigation)


find_package(catkin REQUIRED)

catkin_package()

include_directories()

# Install
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
