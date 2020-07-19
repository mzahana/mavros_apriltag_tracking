cmake_minimum_required(VERSION 3.0.2)
project(mavros_apriltag_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  mavros
  mavros_extras
  roscpp
  rospy
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   PIDGains.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/kf_tracker.cpp 
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
add_executable(kf_tracker_node
  src/kf_tracker_node.cpp
)
add_dependencies(kf_tracker_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kf_tracker_node ${PROJECT_NAME} ${catkin_LIBRARIES})
