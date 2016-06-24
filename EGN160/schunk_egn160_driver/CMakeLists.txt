cmake_minimum_required(VERSION 2.8.3)
project(schunk_egn160_driver)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             message_generation
)
add_message_files(FILES state.msg)

add_service_files(FILES move.srv
                        stop.srv
                        open.srv
                        close.srv)
                        
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

set(DRIVER_SOURCES 
    src/serial.cpp include/schunk_egn160_driver/serial.h
    src/functions.cpp include/schunk_egn160_driver/functions.h)

include_directories(include)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++0x")

add_executable(schunk_egn160_gripper_node src/gripper.cpp ${DRIVER_SOURCES})
add_dependencies(schunk_egn160_gripper_node schunk_egn160_driver_generate_messages_cpp)
target_link_libraries(schunk_egn160_gripper_node ${catkin_LIBRARIES})

add_executable(schunk_egn160_sensors_node src/sensors.cpp)
target_link_libraries(schunk_egn160_sensors_node ${catkin_LIBRARIES})
