cmake_minimum_required(VERSION 2.8.3)
project(wsg_force_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  wsg_50_driver
  wsg_50_common
)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++0x")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wsg_force_control
#  CATKIN_DEPENDS roscpp std_msgs wsg_50_driver
#  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS})

add_executable(wsg_force_control src/wsg.cpp)
target_link_libraries(wsg_force_control ${catkin_LIBRARIES})

add_executable(wsg_master_slave_serial src/wsg_master_slave_serial.cpp)
target_link_libraries(wsg_master_slave_serial ${catkin_LIBRARIES})

add_executable(wsg_master_slave_tcp src/wsg_master_slave_tcp.cpp)
target_link_libraries(wsg_master_slave_tcp ${catkin_LIBRARIES})

