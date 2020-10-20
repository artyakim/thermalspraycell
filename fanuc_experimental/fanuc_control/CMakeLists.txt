cmake_minimum_required(VERSION 2.8.3)
project(fanuc_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf
	tf_conversions
	trac_ik_lib
	moveit_visual_tools
	moveit_ros_perception
)
find_package(PCL 1.8 REQUIRED)
find_package(realsense2 REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
	tf
	moveit_core
	moveit_visual_tools
	moveit_ros_planning_interface
)

include_directories( ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}	src/trajectory_node.cpp)

target_link_libraries(
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	${realsense2_LIBRARY}
)

add_executable(fanuc_trajectory_node src/trajectory_node.cpp)
add_executable (fanuc_cv_node src/CV_RealSense.cpp)

target_link_libraries(
	fanuc_trajectory_node
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)
target_link_libraries(
	fanuc_cv_node
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)
