cmake_minimum_required(VERSION 2.8.3)
project(husky_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface roscpp roslaunch roslint)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Dlinux_x86 -DTRANSPORT_AVAIL -DLOGGING_AVAIL")

add_library(horizon_legacy_api
            src/horizon_legacy_api/crc.cpp
            src/horizon_legacy_api/cwrap.cpp
            src/horizon_legacy_api/Logger.cpp
            src/horizon_legacy_api/Message.cpp
            src/horizon_legacy_api/Message_data.cpp
            src/horizon_legacy_api/Message_request.cpp
            src/horizon_legacy_api/Message_cmd.cpp
            src/horizon_legacy_api/Transport.cpp
            src/horizon_legacy_api/Number.cpp
            src/horizon_legacy_api/linux_serial.cpp
            )

add_executable(husky_base src/husky_base.cpp src/husky_hardware.cpp)
target_link_libraries(husky_base horizon_legacy_api ${catkin_LIBRARIES})
#add_dependencies(husky_base ...)

roslaunch_add_file_check(launch)

install(PROGRAMS
  nodes/basic_odom_publisher
  nodes/diagnostics_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

