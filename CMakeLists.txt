cmake_minimum_required(VERSION 2.8.3)
project(husky_base)

find_package(catkin REQUIRED COMPONENTS controller_manager diff_drive_controller hardware_interface roscpp
    diagnostic_updater roslaunch roslint husky_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Dlinux_x86 -DTRANSPORT_AVAIL")

add_library(horizon_legacy
            src/horizon_legacy/crc.cpp
            src/horizon_legacy/cwrap.cpp
            src/horizon_legacy/Logger.cpp
            src/horizon_legacy/Message.cpp
            src/horizon_legacy/Message_data.cpp
            src/horizon_legacy/Message_request.cpp
            src/horizon_legacy/Message_cmd.cpp
            src/horizon_legacy/Transport.cpp
            src/horizon_legacy/Number.cpp
            src/horizon_legacy/linux_serial.cpp
            )

add_executable(husky_node src/husky_base.cpp src/husky_hardware.cpp src/husky_diagnostics.cpp)
target_link_libraries(husky_node horizon_legacy ${catkin_LIBRARIES})
add_dependencies(husky_node husky_msgs_gencpp)

roslint_cpp()

roslaunch_add_file_check(launch)
install(TARGETS husky_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

