cmake_minimum_required(VERSION 3.0.2)
project(delay_checker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  ERP42_input.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
 LIBRARIES ${PROJECT_NAME} ${EXTERNAL_LIBS}
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/delay_checker.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${EXTERNAL_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(delay_checker_node src/delay_checker_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${EXTERNAL_LIBS}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   #FILES_MATCHING PATTERN "*.h"
)
# install(DIRECTORY
#   launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )