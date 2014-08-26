cmake_minimum_required(VERSION 2.8.3)
project(husky_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

# These won't pass as long as we explicitly include a config file
# from an absolute /etc path. Probably need to specify that location
# in the environment instead.
#roslaunch_add_file_check(launch/imu_config)
#roslaunch_add_file_check(launch/navsat_config)

install(PROGRAMS scripts/install scripts/calibrate_compass
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch udev config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

