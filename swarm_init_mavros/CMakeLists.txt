cmake_minimum_required(VERSION 3.0.2)
project(swarm_init_mavros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swarm_init_mavros
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sta_msgs tf tf2
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(swarm_init_mavros src/odom2mavros.cpp)

target_link_libraries(swarm_init_mavros
  ${catkin_LIBRARIES}
)
