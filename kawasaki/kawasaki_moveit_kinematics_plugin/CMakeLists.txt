cmake_minimum_required(VERSION 2.8.3)
project(kawasaki_moveit_kinematics_plugin)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_package(catkin COMPONENTS
  moveit_core
  pluginlib
  roscpp
  kawasaki_kinematics
  rosconsole_bridge
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
    moveit_core
    )

include_directories(include)
add_library(kawasaki_moveit_kinematics_plugin src/kawasaki_moveit_kinematics_plugin.cpp)
target_link_libraries(kawasaki_moveit_kinematics_plugin ${catkin_LIBRARIES})

install(TARGETS kawasaki_moveit_kinematics_plugin LIBRARY 
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	)

install(FILES kawasaki_moveit_kinematics_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )
