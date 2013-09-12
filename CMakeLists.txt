cmake_minimum_required(VERSION 2.8.3)
project(husky_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS scripts/install
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

