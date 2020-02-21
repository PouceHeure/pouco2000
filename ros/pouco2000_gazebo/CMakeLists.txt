cmake_minimum_required(VERSION 2.8.3)
project(pouco2000_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diff_drive_controller
  gazebo_ros
  gazebo_ros_control
  joint_state_controller
  position_controllers
  robot_state_publisher
  rqt_robot_steering
  xacro
  roscpp
  pouco2000_ros 
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pouco2000_gazebo
#  CATKIN_DEPENDS controller_manager diff_drive_controller gazebo_ros gazebo_ros_control joint_state_controller position_controllers robot_state_publisher rqt_robot_steering xacro
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_controller_node node/simple_controller.cpp)
add_dependencies(simple_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_controller_node ${catkin_LIBRARIES})



add_executable(controller_node node/controller.cpp)
add_dependencies(controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_node ${catkin_LIBRARIES})
