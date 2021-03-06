cmake_minimum_required(VERSION 2.8.3)
project(ros_custom_estimator)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    ros_node_utils
	ros_custom_hardware_adapter
	robot_container
    roscpp
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
	INCLUDE_DIRS
	  include
	  ${EIGEN3_INCLUDE_DIR}
	  ${boost_INCLUDE_DIR}
	LIBRARIES
	CATKIN_DEPENDS
	  roscpp
	  ros_node_utils
	  ros_custom_hardware_adapter
	  robot_container
	  message_generation
	  std_msgs

)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
