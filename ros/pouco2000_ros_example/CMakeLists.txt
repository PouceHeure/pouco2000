cmake_minimum_required(VERSION 2.8.3)
project(pouco2000_ros_example)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pouco2000_ros
  ros_popup_img
  roscpp
  std_msgs
  geometry_msgs
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
add_dependencies(switch_img_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(switch_img_node ${catkin_LIBRARIES})

add_executable(twist_node node/twist.cpp)
add_dependencies(twist_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_node ${catkin_LIBRARIES})
