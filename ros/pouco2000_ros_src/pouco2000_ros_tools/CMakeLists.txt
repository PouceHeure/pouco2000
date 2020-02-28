cmake_minimum_required(VERSION 2.8.3)
project(pouco2000_ros_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pouco2000_ros_msgs
  roscpp
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_extractor
  CATKIN_DEPENDS pouco2000_ros_msgs roscpp std_msgs
# DEPENDS system_lib
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

add_library(${PROJECT_NAME}_extractor
  src/pouco2000_extractor.cpp
) 

add_dependencies(${PROJECT_NAME}_extractor pouco2000_ros_msgs_generate_messages_cpp)

## Generate package documentation with rosdoc_lite
execute_process(COMMAND rospack find ${PROJECT_NAME} 
                OUTPUT_VARIABLE PROJECT_DIR  
                OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rosdoc_lite -o ${PROJECT_DIR}/doc/ ${PROJECT_DIR})

##################
## Installation ##
##################

install(
  TARGETS ${PROJECT_NAME}_extractor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(utest test/utest.cpp)
  target_link_libraries(utest ${catkin_LIBRARIES} ${PROJECT_NAME}_extractor)
endif()
