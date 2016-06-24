cmake_minimum_required(VERSION 2.8.3)
project(ati_netft_hardware)

  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS roscpp pluginlib hardware_interface composite_hardware_interface emergency_hardware_interface netft_rdt_driver)

  include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
  add_library(${PROJECT_NAME} src/netft.cpp)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS roscpp pluginlib hardware_interface composite_hardware_interface emergency_hardware_interface netft_rdt_driver
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install(FILES ati_netft_device_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
