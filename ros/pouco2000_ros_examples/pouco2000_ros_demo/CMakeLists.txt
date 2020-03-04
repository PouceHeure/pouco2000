cmake_minimum_required(VERSION 2.8.3)
project(pouco2000_ros_demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pouco2000_ros_msgs # need: pouco2000 msgs 
  pouco2000_ros_tools # need: pouco2000_extractor lib 
  roscpp
  std_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pouco2000_ros_demo
#  CATKIN_DEPENDS pouco2000_ros_msgs pouco2000_ros_tools roscpp std_msgs
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

add_executable(demo_01_simple_node node/demo_01_simple.cpp)
add_dependencies(demo_01_simple_node ${catkin_EXPORTED_TARGETS} pouco2000_ros_msgs_generate_messages_cpp)
target_link_libraries(demo_01_simple_node ${catkin_LIBRARIES})

add_executable(demo_02_extractor_node node/demo_02_extractor.cpp)
add_dependencies(demo_02_extractor_node ${catkin_EXPORTED_TARGETS} pouco2000_ros_msgs_generate_messages_cpp)
target_link_libraries(demo_02_extractor_node ${catkin_LIBRARIES})

add_executable(demo_03_handle_extractor_node node/demo_03_handle_extractor.cpp)
add_dependencies(demo_03_handle_extractor_node ${catkin_EXPORTED_TARGETS} pouco2000_ros_msgs_generate_messages_cpp)
target_link_libraries(demo_03_handle_extractor_node ${catkin_LIBRARIES})







