cmake_minimum_required(VERSION 3.0.2)
project(mavros_apriltag_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
set (CMAKE_CXX_STANDARD 11)

find_package(Eigen3)
find_package(OsqpEigen)

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
  nlopt
  OsqpEigen
)

#include_directories(${osqp_INCLUDE_DIRS})

## Generate services in the 'srv' folder
add_service_files(
   FILES
   PIDGains.srv
)

add_message_files(
  FILES
  KFState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
  #${Eigen_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(kf_tracker
  src/kf_tracker.cpp 
)
add_dependencies(kf_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kf_tracker ${catkin_LIBRARIES})

add_library(mpc_tracker
  src/mpc_tracker.cpp 
)
add_dependencies(mpc_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_tracker ${catkin_LIBRARIES})

############
# BINARIES #
############

# KF tracker
add_executable(kf_tracker_node
  src/kf_tracker_node.cpp
)
add_dependencies(kf_tracker_node kf_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kf_tracker_node kf_tracker ${catkin_LIBRARIES})

# MPC tracker
add_executable(mpc_tracker_node
  src/mpc_tracker_node.cpp
)
add_dependencies(mpc_tracker_node mpc_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_tracker_node mpc_tracker ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)