cmake_minimum_required(VERSION 2.8.3)
project(pouco2000_ros_popup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pouco2000_ros_msgs
  pouco2000_ros_tools
  ros_popup_img
  roscpp
  std_msgs
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

add_executable(popup_node node/popup.cpp)
add_dependencies(popup_node ${catkin_EXPORTED_TARGETS} pouco2000_ros_msgs_generate_messages_cpp)
target_link_libraries(popup_node ${catkin_LIBRARIES})

