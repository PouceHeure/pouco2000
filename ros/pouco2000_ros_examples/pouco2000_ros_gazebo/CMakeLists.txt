cmake_minimum_required(VERSION 2.8.3)
project(pouco2000_ros_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


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
  pouco2000_ros_msgs
  pouco2000_ros_tools
)

###################################
## catkin specific configuration ##
###################################
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

add_executable(interface_node node/interface.cpp)
add_dependencies(interface_node ${catkin_EXPORTED_TARGETS} pouco2000_ros_msgs_generate_messages_cpp)
target_link_libraries(interface_node ${catkin_LIBRARIES})

add_executable(controller_gazebo_node node/controller.cpp)
add_dependencies(controller_gazebo_node ${catkin_EXPORTED_TARGETS} pouco2000_ros_msgs_generate_messages_cpp)
target_link_libraries(controller_gazebo_node ${catkin_LIBRARIES})

