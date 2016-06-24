cmake_minimum_required(VERSION 2.8.3)
project(kuka_kinematics)

find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp pluginlib urdf kdl_parser)
find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   DEPENDS eigen urdf orocos_kdl
)

if (CATKIN_ENABLE_TESTING)
	include_directories(
	  ${CMAKE_CURRENT_SOURCE_DIR}/include
	  ${catkin_INCLUDE_DIRS}
      ${Boost_INCLUDE_DIRS}
      ${EIGEN_INCLUDE_DIRS}
	)

	#catkin_add_gtest(kawasaki_ik_test test/kawasaki_ik_test.cpp)
	#add_dependencies(kawasaki_ik_test ${catkin_EXPORTED_TARGETS})
	#target_link_libraries(kawasaki_ik_test ${catkin_LIBRARIES})
	#target_link_libraries(kawasaki_ik_test ${orocos_kdl_LIBRARIES})
endif()
