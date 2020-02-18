cmake_minimum_required(VERSION 2.8.3)
project(pouco2000_ros_example)

find_package(catkin REQUIRED COMPONENTS
  pouco2000_ros
  ros_popup_img
  roscpp
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pouco2000_ros_example
#  CATKIN_DEPENDS pouco2000_ros ros_popup_img roscpp std_msgs
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

add_executable(switch_img_node node/switch_img.cpp)
target_link_libraries(switch_img_node ${catkin_LIBRARIES})
