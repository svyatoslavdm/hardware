cmake_minimum_required(VERSION 2.8.3)
project(kawasaki_hardware)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(catkin REQUIRED COMPONENTS 
    cmake_modules kawasaki_kinematics urdf hardware_interface
    composite_hardware_interface 
    emergency_hardware_interface 
    kdl_parser
    std_msgs)
    
find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)


catkin_package(
    CATKIN_DEPENDS composite_hardware_interface
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${kawasaki_kinematics_INCLUDE_DIRS}
)

# kawasaki hardware
add_library(${PROJECT_NAME} src/kawasaki_hw.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


# Install
#install(DIRECTORY include/${PROJECT_NAME}/
#DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(TARGETS ${PROJECT_NAME}
#ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
