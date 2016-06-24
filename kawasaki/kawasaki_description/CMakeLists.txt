cmake_minimum_required(VERSION 2.8.3)
project(kawasaki_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp pluginlib urdf kdl_parser)

## System dependencies are found with CMake's conventions
#find_package(orocos_kdl REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#   INCLUDE_DIRS include
#   DEPENDS eigen orocos_kdl
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)
