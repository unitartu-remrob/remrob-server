cmake_minimum_required(VERSION 3.0.2)
project(robotont_nuc_description)

find_package(catkin REQUIRED)

catkin_package()

# Install
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
