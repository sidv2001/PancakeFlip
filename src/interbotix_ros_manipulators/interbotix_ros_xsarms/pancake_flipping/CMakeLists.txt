cmake_minimum_required(VERSION 2.8.3)
project(pancake_flipping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  interbotix_xs_msgs
  interbotix_xsarm_control
  rosbag
  roscpp
  tf2_ros
  rviz
  sensor_msgs
  std_msgs
  std_srvs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
 CATKIN_DEPENDS interbotix_xs_msgs interbotix_xsarm_control rosbag roscpp tf2_ros rviz sensor_msgs std_msgs std_srvs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable
add_executable(noisy_pub src/noisy_pub.cpp)
target_link_libraries(noisy_pub ${catkin_LIBRARIES})
add_dependencies(noisy_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(clean_pub src/clean_pub.cpp)
target_link_libraries(clean_pub ${catkin_LIBRARIES})
add_dependencies(clean_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(synchronizer src/synchronizer.cpp)
target_link_libraries(synchronizer ${catkin_LIBRARIES})
add_dependencies(synchronizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})