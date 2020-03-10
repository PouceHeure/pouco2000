cmake_minimum_required(VERSION 2.8.3)
project(pouco2000_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pouco2000_ros_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}_extractor
  CATKIN_DEPENDS roscpp std_msgs pouco2000_ros_msgs
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

## libs cpp 

add_library(${PROJECT_NAME}
  src/pouco2000.cpp
)

add_dependencies(${PROJECT_NAME} pouco2000_ros_msgs_generate_messages_cpp)

add_library(${PROJECT_NAME}_debug
  src/pouco2000_debug.cpp
)

add_dependencies(${PROJECT_NAME}_debug pouco2000_ros_msgs_generate_messages_cpp)

add_library(${PROJECT_NAME}_introspection
  src/pouco2000_introspection.cpp
)

add_dependencies(${PROJECT_NAME}_introspection pouco2000_ros_msgs_generate_messages_cpp)

add_library(${PROJECT_NAME}_monitor
  src/pouco2000_monitor.cpp
)

add_dependencies(${PROJECT_NAME}_monitor pouco2000_ros_msgs_generate_messages_cpp)

## Nodes release

add_executable(controller_node node/release/controller.cpp)
add_dependencies(controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(filter_node node/release/filter.cpp)
add_dependencies(filter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_node ${catkin_LIBRARIES} ${PROJECT_NAME}_introspection)

add_executable(monitor_node node/release/monitor.cpp)
add_dependencies(monitor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(monitor_node ${catkin_LIBRARIES} ${PROJECT_NAME}_monitor)

## Nodes debug

add_executable(debug_fake_publisher_buttons_node node/debug/fake_publisher_buttons.cpp)
add_dependencies(debug_fake_publisher_buttons_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_fake_publisher_buttons_node ${catkin_LIBRARIES} ${PROJECT_NAME}_debug)

add_executable(debug_fake_publisher_switchs_onoff_node node/debug/fake_publisher_switchs_onoff.cpp)
add_dependencies(debug_fake_publisher_switchs_onoff_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_fake_publisher_switchs_onoff_node ${catkin_LIBRARIES} ${PROJECT_NAME}_debug)

add_executable(debug_fake_publisher_switchs_modes_node node/debug/fake_publisher_switchs_modes.cpp)
add_dependencies(debug_fake_publisher_switchs_modes_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_fake_publisher_switchs_modes_node ${catkin_LIBRARIES} ${PROJECT_NAME}_debug)

add_executable(debug_fake_publisher_potentiometers_circle_node node/debug/fake_publisher_potentiometers_circle.cpp)
add_dependencies(debug_fake_publisher_potentiometers_circle_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_fake_publisher_potentiometers_circle_node ${catkin_LIBRARIES} ${PROJECT_NAME}_debug)

add_executable(debug_fake_publisher_potentiometers_slider_node node/debug/fake_publisher_potentiometers_slider.cpp)
add_dependencies(debug_fake_publisher_potentiometers_slider_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_fake_publisher_potentiometers_slider_node ${catkin_LIBRARIES} ${PROJECT_NAME}_debug)

## Generate package documentation with rosdoc_lite
execute_process(COMMAND rospack find ${PROJECT_NAME} 
                OUTPUT_VARIABLE PROJECT_DIR  
                OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rosdoc_lite -o ${PROJECT_DIR}/doc/ ${PROJECT_DIR})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(utest_pouco2000_ros test/utest.cpp)
  target_link_libraries(utest_pouco2000_ros ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_debug)
endif()