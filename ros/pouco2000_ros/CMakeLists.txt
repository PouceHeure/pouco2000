cmake_minimum_required(VERSION 2.8.3)
project(pouco2000_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
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

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pouco2000_ros
   CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)
 
###########
## Build ##
########### 

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_library(${PROJECT_NAME}
  src/pouco2000.cpp
)

add_library(${PROJECT_NAME}_debug
  src/pouco2000_debug.cpp
)

## Nodes release

add_executable(controller_node node/release/controller.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES} ${PROJECT_NAME})

## Nodes debug

add_executable(debug_fake_publisher_buttons_node node/debug/fake_publisher_buttons.cpp)
target_link_libraries(debug_fake_publisher_buttons_node ${catkin_LIBRARIES} ${PROJECT_NAME}_debug)

add_executable(debug_fake_publisher_switchs_onoff_node node/debug/fake_publisher_switchs_onoff.cpp)
target_link_libraries(debug_fake_publisher_switchs_onoff_node ${catkin_LIBRARIES} ${PROJECT_NAME}_debug)

add_executable(debug_fake_publisher_switchs_modes_node node/debug/fake_publisher_switchs_modes.cpp)
target_link_libraries(debug_fake_publisher_switchs_modes_node ${catkin_LIBRARIES} ${PROJECT_NAME}_debug)

add_executable(debug_fake_publisher_potentiometers_circle_node node/debug/fake_publisher_potentiometers_circle.cpp)
target_link_libraries(debug_fake_publisher_potentiometers_circle_node ${catkin_LIBRARIES} ${PROJECT_NAME}_debug)

add_executable(debug_fake_publisher_potentiometers_slider_node node/debug/fake_publisher_potentiometers_slider.cpp)
target_link_libraries(debug_fake_publisher_potentiometers_slider_node ${catkin_LIBRARIES} ${PROJECT_NAME}_debug)

 
## Generate package documentation with rosdoc_lite
execute_process(COMMAND rospack find ${PROJECT_NAME} 
                OUTPUT_VARIABLE PROJECT_DIR  
                OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rosdoc_lite -o ${PROJECT_DIR}/doc/ ${PROJECT_DIR})


if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(utest test/utest.cpp)
  target_link_libraries(utest ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_debug)
endif()