cmake_minimum_required(VERSION 2.8.3)
project(kuka_hardware)

find_package(catkin REQUIRED COMPONENTS 
    cmake_modules 
    composite_hardware_interface 
    kdl_parser
    kuka_kinematics)
    
find_package(Boost REQUIRED COMPONENTS thread)
find_package(TinyXML REQUIRED)

catkin_package(
    CATKIN_DEPENDS composite_hardware_interface
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} 
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${kuka_kinematics_INCLUDE_DIRS}
)

# kuka hardware library
add_library(${PROJECT_NAME} src/kuka_hw.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
