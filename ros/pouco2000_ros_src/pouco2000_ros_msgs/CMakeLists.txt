cmake_minimum_required(VERSION 2.8.3)
project(pouco2000_ros_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
)
################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Controller.msg
  Buttons.msg
  SwitchsOnOff.msg
  SwitchsMode.msg
  Potentiometers.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}_extractor
  CATKIN_DEPENDS message_generation message_runtime std_msgs
  #DEPENDS system_lib
) 
  