cmake_minimum_required(VERSION 2.8.3)
project(telepresence)

add_compile_options(-std=c++11)# -g)

find_package(Boost)
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	nav_msgs
	roscpp
	rospy
	sensor_msgs
	std_srvs
	message_generation
	tf
	tf2
	tf2_ros
	image_geometry
	image_transport
	message_filters
	ddynamic_reconfigure
	grid_map_ros
	cv_bridge
	visualization_msgs
	)

add_message_files(
	FILES
	)

add_service_files(
	FILES
	Click.srv
	)

generate_messages(
	DEPENDENCIES
	std_msgs
	telepresence
	)

catkin_package(
	INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
	CATKIN_DEPENDS
	roscpp
	rospy
	geometry_msgs
	nav_msgs
	roscpp rospy
	sensor_msgs
	actionlib_msgs
	cv_bridge
	std_srvs
	grid_map_ros
	message_runtime
	actionlib
	actionlib_msgs
	ddynamic_reconfigure
	tf
	tf2
	tf2_ros
	visualization_msgs
	# DEPENDS 
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	)

add_executable(${PROJECT_NAME}_node src/telepresence.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME telepresence PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
	${catkin_LIBRARIES}
	)
