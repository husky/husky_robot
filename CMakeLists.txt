cmake_minimum_required(VERSION 2.8.3)
project(husky_base)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  nodes/basic_odom_publisher
  nodes/diagnostics_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

